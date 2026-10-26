/*
 * SPDX-FileCopyrightText: 2026 fmauNeko
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file main.c
 * @brief display_demo — Panda Touch showcase
 * @details Four-tab LVGL UI demonstrating backlight control, USB file browser, optional AHT30 sensor data, and display sleep/wake.
 *
 * Four-tab LVGL UI:
 *  - Backlight  : Interactive slider to control PWM backlight brightness
 *  - USB        : File browser — lists files/directories from an inserted USB drive
 *  - Sleep      : One-button test of bsp_display_enter_sleep / bsp_display_exit_sleep
 *
 * Threading model:
 *  - LVGL task (no affinity): drives lv_timer_handler; never blocks on I/O
 *                             LVGL heap routed to PSRAM so large sub-layer buffers can be
 *                             allocated without exhausting the small internal SRAM heap.
 *  - msc_app_task (CPU0, p5): USB mount/unmount; reads dir BEFORE taking LVGL lock
 *  - sensor_task (CPU0, p4) : reads AHT30 every 2 s; posts results to s_sensor_queue
 *  - LVGL sensor timer      : drains s_sensor_queue; zero blocking I2C inside LVGL task
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <dirent.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <time.h>
#include <math.h>

#include "esp_heap_caps.h"
#include "bsp/esp-bsp.h"

#define LV_LVGL_H_INCLUDE_SIMPLE
#include "../../assets/steam_logo.c"

static const char *TAG = "demo";

/* ── USB file list snapshot (read outside LVGL lock) ───────────────────── */
#define USB_MAX_FILES  32
#define USB_NAME_MAX   256

typedef struct {
    char     name[USB_NAME_MAX];
    uint8_t  is_dir;
} usb_entry_t;

typedef struct {
    usb_entry_t entries[USB_MAX_FILES];
    int         count;
    bool        open_ok;
    bool        mounted;
} usb_snapshot_t;

/* ── LVGL widget refs (set during ui_create, used in callbacks) ─────────── */
#define CPU_TEMP_COUNT 1
#define CPU_TEMP_UART_PORT UART_NUM_0
#define CPU_TEMP_UART_BAUD 115200
#define CPU_TEMP_UART_RX_PIN GPIO_NUM_44
#define CPU_TEMP_UART_TX_PIN GPIO_NUM_43
#define DISPLAY_SLEEP_TIMEOUT_MS (5U * 60U * 1000U)

static lv_obj_t *s_brightness_label = NULL;
static lv_obj_t *s_usb_list         = NULL;
static lv_obj_t *s_usb_status       = NULL;
static lv_obj_t *s_sleep_btn        = NULL;
static lv_obj_t *s_sleep_label      = NULL;
static TaskHandle_t volatile s_sleep_task_handle = NULL;
static volatile bool s_display_is_sleeping = false;
static volatile TickType_t s_last_activity_tick = 0;
static int       s_brightness_value = 80;
static lv_obj_t *s_cpu_temp_label = NULL;
static lv_obj_t *s_gpu_temp_label = NULL;
static lv_obj_t *s_liquid_temp_label = NULL;
static lv_obj_t *s_cpu_fan_label = NULL;
static lv_obj_t *s_cpu_fan_gauge = NULL;
static lv_obj_t *s_cpu_fan_needle = NULL;
static lv_obj_t *s_gpu_fan_label = NULL;
static lv_obj_t *s_gpu_fan_gauge = NULL;
static lv_obj_t *s_gpu_fan_needle = NULL;
static uint8_t   s_cpu_temp_value = 0;
static uint8_t   s_gpu_temp_value = 0;
static uint8_t   s_liquid_temp_value = 0;
static uint16_t  s_cpu_fan_value = 0;
static uint16_t  s_gpu_fan_value = 0;
static char      s_cpu_temp_uart_line[128];
static int       s_cpu_temp_uart_line_len = 0;
static bool      s_cpu_temp_uart_line_overflow = false;
/* Host metrics */
static uint8_t   s_host_cpu_usage = 0; /* percent 0-100 */
static double    s_host_ram_used_gb = 0.0;
static double    s_host_ram_total_gb = 0.0;
static double    s_host_hdd_used_gb = 0.0;
static double    s_host_hdd_total_gb = 0.0;

/* UI objects for host metrics */
static lv_obj_t *s_host_cpu_label = NULL;
static lv_obj_t *s_host_cpu_gauge = NULL;
static lv_obj_t *s_host_ram_label = NULL;
static lv_obj_t *s_host_ram_bar = NULL;
static lv_obj_t *s_host_hdd_label = NULL;
static lv_obj_t *s_host_hdd_bar = NULL;
static lv_obj_t *s_clock_label = NULL;
static lv_obj_t *s_host_gpu_label = NULL;
static lv_obj_t *s_host_gpu_gauge = NULL;
/* Host-provided clock (HH:MM). -1 when not provided yet */
static int s_host_clock_h = -1;
static int s_host_clock_m = -1;
/* Host GPU usage percent 0-100 */
static uint8_t s_host_gpu_usage = 0;

static void sleep_start(void);
static void sleep_wake(void);
static void sleep_task(void *arg);

static void cpu_temp_uart_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = CPU_TEMP_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(CPU_TEMP_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(CPU_TEMP_UART_PORT,
                                CPU_TEMP_UART_TX_PIN,
                                CPU_TEMP_UART_RX_PIN,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(CPU_TEMP_UART_PORT,
                                       1024,
                                       0,
                                       0,
                                       NULL,
                                       0));
    s_last_activity_tick = xTaskGetTickCount();
}

static bool cpu_temp_uart_parse_line(const char *line, int *values, int *value_count)
{
    const char *cursor = line;
    int count = 0;

    while (*cursor != '\0') {
        if (count >= 13) {
            return false;
        }

        char *end = NULL;
        double parsed = strtod(cursor, &end);
        if (end == cursor || !isfinite(parsed) || parsed < INT_MIN || parsed > INT_MAX) {
            return false;
        }

        while (*end == ' ' || *end == '\t') {
            end++;
        }
        values[count++] = (int)lrint(parsed);

        if (*end == '\0') {
            break;
        }
        if (*end != ',' && *end != ';') {
            return false;
        }

        cursor = end + 1;
        if (*cursor == '\0') {
            return false;
        }
    }

    if (count != 11 && count != 13) {
        return false;
    }

    bool has_clock = count == 12 || count == 13;
    bool has_liquid_temp = count == 11 || count == 13;
    if (values[0] < 0 || values[0] > 100 ||
        values[1] < 0 || values[1] > 100 ||
        values[2] < 0 || values[2] > 20000 ||
        values[3] < 0 || values[3] > 20000 ||
        values[4] < 0 || values[4] > 100 ||
        values[5] < 0 || values[5] > 100 ||
        values[6] < 0 || values[7] < 0 ||
        values[8] < 0 || values[9] < 0) {
        return false;
    }

    if (has_clock) {
        if (values[10] < 0 || values[10] > 23 ||
            values[11] < 0 || values[11] > 59) {
            return false;
        }
    }
    if (has_liquid_temp) {
        int liquid_temp_index = has_clock ? 12 : 10;
        if (values[liquid_temp_index] < 0 || values[liquid_temp_index] > 100) {
            return false;
        }
    }

    *value_count = count;
    return true;
}

static void cpu_temp_uart_apply_line(const char *line)
{
    int values[13];
    int value_count = 0;
    if (!cpu_temp_uart_parse_line(line, values, &value_count)) {
        ESP_LOGW(TAG, "Ignoring malformed CPU telemetry record");
        return;
    }

    bool has_clock = value_count == 12 || value_count == 13;
    bool has_liquid_temp = value_count == 11 || value_count == 13;

    s_cpu_temp_value = (uint8_t)values[0];
    s_gpu_temp_value = (uint8_t)values[1];
    s_cpu_fan_value = (uint16_t)LV_CLAMP(values[2], 0, 10000);
    s_gpu_fan_value = (uint16_t)LV_CLAMP(values[3], 0, 10000);
    s_host_cpu_usage = (uint8_t)values[4];
    s_host_gpu_usage = (uint8_t)values[5];

    if (values[7] > 0) {
        s_host_ram_used_gb = (double)values[6];
        s_host_ram_total_gb = (double)values[7];
    }
    if (values[9] > 0) {
        s_host_hdd_used_gb = (double)values[8];
        s_host_hdd_total_gb = (double)values[9];
    }
    if (has_liquid_temp) {
        int liquid_temp_index = has_clock ? 12 : 10;
        s_liquid_temp_value = (uint8_t)values[liquid_temp_index];
    }
    if (has_clock) {
        s_host_clock_h = values[10];
        s_host_clock_m = values[11];
    }
}

static bool cpu_temp_uart_update(void)
{
    uint8_t raw[64];
    bool received = false;
    int len;
    while ((len = uart_read_bytes(CPU_TEMP_UART_PORT, raw, sizeof(raw), 0)) > 0) {
        if (!received) {
            s_last_activity_tick = xTaskGetTickCount();
            sleep_wake();
            received = true;
        }

        for (int i = 0; i < len; i++) {
            char ch = (char)raw[i];

            if (ch == '\r') {
                continue;
            }
            if (ch == '\n') {
                if (!s_cpu_temp_uart_line_overflow && s_cpu_temp_uart_line_len > 0) {
                    s_cpu_temp_uart_line[s_cpu_temp_uart_line_len] = '\0';
                    cpu_temp_uart_apply_line(s_cpu_temp_uart_line);
                }
                s_cpu_temp_uart_line_len = 0;
                s_cpu_temp_uart_line_overflow = false;
                continue;
            }

            if (!s_cpu_temp_uart_line_overflow) {
                if (s_cpu_temp_uart_line_len < (int)sizeof(s_cpu_temp_uart_line) - 1) {
                    s_cpu_temp_uart_line[s_cpu_temp_uart_line_len++] = ch;
                } else {
                    s_cpu_temp_uart_line_overflow = true;
                }
            }
        }
    }

    return received;
}

/* ── Colour palette ─────────────────────────────────────────────────────── */
#define COL_BG        lv_color_hex(0x1a1a2e)
#define COL_CARD      lv_color_hex(0x16213e)
#define COL_ACCENT    lv_color_hex(0xe94560)
#define COL_CYAN      lv_color_hex(0x4cc9f0)
#define COL_BORDER    lv_color_hex(0x0f3460)
#define COL_TEXT      lv_color_hex(0xeeeeee)
#define COL_MUTED     lv_color_hex(0x888888)

/* ════════════════════════════════════════════════════════════════════════════
 *  USB file list — two-phase update
 *
 *  usb_snapshot_read()   : block I/O OUTSIDE the LVGL lock
 *  usb_snapshot_render() : LVGL calls INSIDE the LVGL lock
 *
 *  Keeping filesystem I/O outside the LVGL mutex prevents the LVGL task
 *  from starving IDLE0 during USB block reads (root cause of WDT crash).
 * ════════════════════════════════════════════════════════════════════════════ */
static void usb_snapshot_read(usb_snapshot_t *snap)
{
    snap->count   = 0;
    snap->open_ok = false;
    snap->mounted = bsp_usb_is_mounted();

    if (!snap->mounted) {
        return;
    }

    DIR *dir = opendir("/usb");
    if (!dir) {
        return;
    }
    snap->open_ok = true;

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && snap->count < USB_MAX_FILES) {
        if (ent->d_name[0] == '.') {
            continue;
        }
        usb_entry_t *e = &snap->entries[snap->count++];
        strlcpy(e->name, ent->d_name, sizeof(e->name));
        e->is_dir = (ent->d_type == DT_DIR);
    }
    closedir(dir);
}

static void usb_snapshot_render(const usb_snapshot_t *snap)
{
    lv_obj_clean(s_usb_list);

    if (!snap->mounted) {
        lv_label_set_text(s_usb_status, LV_SYMBOL_USB "  No USB drive connected");
        lv_obj_clear_flag(s_usb_status, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_usb_list, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (!snap->open_ok) {
        lv_label_set_text(s_usb_status, LV_SYMBOL_WARNING "  Failed to open /usb");
        lv_obj_clear_flag(s_usb_status, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_usb_list, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (snap->count == 0) {
        lv_label_set_text(s_usb_status, LV_SYMBOL_USB "  Drive is empty");
        lv_obj_clear_flag(s_usb_status, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_usb_list, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    char line[USB_NAME_MAX + 8];
    for (int i = 0; i < snap->count; i++) {
        const char *icon = snap->entries[i].is_dir
                           ? LV_SYMBOL_DIRECTORY "  "
                           : LV_SYMBOL_FILE      "  ";
        snprintf(line, sizeof(line), "%s%s", icon, snap->entries[i].name);
        lv_list_add_text(s_usb_list, line);
    }
    lv_obj_add_flag(s_usb_status, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_usb_list, LV_OBJ_FLAG_HIDDEN);
}

static void usb_update(void)
{
    usb_snapshot_t *snap = malloc(sizeof(usb_snapshot_t));
    if (!snap) {
        ESP_LOGE(TAG, "USB snapshot alloc failed");
        return;
    }
    usb_snapshot_read(snap);
    bsp_display_lock(portMAX_DELAY);
    usb_snapshot_render(snap);
    bsp_display_unlock();
    free(snap);
}

static void on_usb_mount(void)
{
    ESP_LOGI(TAG, "USB mounted");
    usb_update();
}

static void on_usb_unmount(void)
{
    ESP_LOGI(TAG, "USB removed");
    usb_update();
}

/* ════════════════════════════════════════════════════════════════════════════
 *  LVGL event / timer callbacks
 *  All run inside the LVGL task — no extra lock needed.
 * ════════════════════════════════════════════════════════════════════════════ */
static void brightness_store(int value)
{
    esp_err_t err;
    nvs_handle_t handle;

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u8(handle, "brightness", (uint8_t)value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_u8 failed: %s", esp_err_to_name(err));
    }
    nvs_close(handle);
}

static int brightness_load(void)
{
    esp_err_t err;
    nvs_handle_t handle;
    uint8_t value = 80;

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return 80;
    }

    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return 80;
    }

    err = nvs_get_u8(handle, "brightness", &value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        value = 80;
        nvs_set_u8(handle, "brightness", value);
        nvs_commit(handle);
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_get_u8 failed: %s", esp_err_to_name(err));
        value = 80;
    }

    nvs_close(handle);
    s_brightness_value = value;
    return value;
}

static void brightness_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int val = (int)lv_slider_get_value(slider);
    s_brightness_value = val;
    brightness_store(val);
    bsp_display_brightness_set(val);
    lv_label_set_text_fmt(s_brightness_label, "%d%%", val);
}

static void stream_deck_send_cmd(const char *cmd)
{
    if (!cmd) {
        return;
    }

    char pkt[32];
    int len = snprintf(pkt, sizeof(pkt), "CMD:%s\n", cmd);
    if (len > 0) {
        uart_write_bytes(CPU_TEMP_UART_PORT, pkt, (size_t)len);
    }
}

static void stream_deck_btn_cb(lv_event_t *e)
{
    const char *cmd = (const char *)lv_event_get_user_data(e);
    stream_deck_send_cmd(cmd);
}

static void temp_value_update(lv_obj_t *label, uint8_t value, const char *name)
{
    if (label == NULL) {
        return;
    }

    if (value == 0) {
        lv_label_set_text_fmt(label, "%s\nN/A", name);
        return;
    }

    lv_label_set_text_fmt(label, "%s\n%d°C", name, value);
}

static void fan_arc_update(lv_obj_t *gauge, lv_obj_t *needle, uint16_t value)
{
    if (gauge == NULL) {
        return;
    }

    if (needle != NULL) {
        lv_scale_set_line_needle_value(gauge, needle, 52, value == 0 ? 0 : value);
    }
}

static void cpu_temp_timer_cb(lv_timer_t *t)
{
    (void)t;
    cpu_temp_uart_update();
    if ((TickType_t)(xTaskGetTickCount() - s_last_activity_tick) >=
        pdMS_TO_TICKS(DISPLAY_SLEEP_TIMEOUT_MS)) {
        sleep_start();
    }

    int next = s_cpu_temp_value;
    if (next < 0) next = 0;
    if (next > 100) next = 100;

    temp_value_update(s_cpu_temp_label, (uint8_t)next, "CPU");
    temp_value_update(s_gpu_temp_label, s_gpu_temp_value, "GPU");
    temp_value_update(s_liquid_temp_label, s_liquid_temp_value, "LIQ");
    fan_arc_update(s_cpu_fan_gauge, s_cpu_fan_needle, s_cpu_fan_value);
    fan_arc_update(s_gpu_fan_gauge, s_gpu_fan_needle, s_gpu_fan_value);

    if (s_cpu_fan_label) {
        lv_label_set_text_fmt(s_cpu_fan_label, "CPU_FAN\n%d RPM", s_cpu_fan_value == 0 ? 0 : s_cpu_fan_value);
    }
    if (s_gpu_fan_label) {
        lv_label_set_text_fmt(s_gpu_fan_label, "GPU_FAN\n%d RPM", s_gpu_fan_value == 0 ? 0 : s_gpu_fan_value);
    }

    /* Update host metrics UI */
    if (s_host_cpu_label) {
        lv_label_set_text_fmt(s_host_cpu_label, "CPU USAGE\n%d%%", s_host_cpu_usage);
    }
    if (s_host_cpu_gauge) {
        int cpu_bar = (int)s_host_cpu_usage * 10; /* map 0-100 -> 0-1000 */
        lv_bar_set_value(s_host_cpu_gauge, cpu_bar, LV_ANIM_OFF);
    }

    if (s_host_gpu_label) {
        lv_label_set_text_fmt(s_host_gpu_label, "GPU USAGE\n%d%%", s_host_gpu_usage);
    }
    if (s_host_gpu_gauge) {
        int gpu_bar = (int)s_host_gpu_usage * 10; /* map 0-100 -> 0-1000 */
        lv_bar_set_value(s_host_gpu_gauge, gpu_bar, LV_ANIM_OFF);
    }

    if (s_host_ram_label && s_host_ram_bar) {
        if (s_host_ram_total_gb > 0.0) {
            int ram_val = (int)((s_host_ram_used_gb / s_host_ram_total_gb) * 1000.0 + 0.5);
            lv_bar_set_value(s_host_ram_bar, ram_val, LV_ANIM_OFF);
            char ram_buf[32];
            int used_whole = (int)s_host_ram_used_gb;
            int used_tenth = (int)((s_host_ram_used_gb - used_whole) * 10.0 + 0.5);
            int total_whole = (int)s_host_ram_total_gb;
            int total_tenth = (int)((s_host_ram_total_gb - total_whole) * 10.0 + 0.5);
            snprintf(ram_buf, sizeof(ram_buf), "RAM\n%d.%d / %d.%d GB", used_whole, used_tenth, total_whole, total_tenth);
            lv_label_set_text(s_host_ram_label, ram_buf);
        } else {
            lv_bar_set_value(s_host_ram_bar, 0, LV_ANIM_OFF);
            lv_label_set_text(s_host_ram_label, "RAM\nN/A");
        }
    }

    if (s_host_hdd_label && s_host_hdd_bar) {
        if (s_host_hdd_total_gb > 0.0) {
            int hdd_val = (int)((s_host_hdd_used_gb / s_host_hdd_total_gb) * 1000.0 + 0.5);
            lv_bar_set_value(s_host_hdd_bar, hdd_val, LV_ANIM_OFF);
            char hdd_buf[32];
            int h_used_whole = (int)s_host_hdd_used_gb;
            int h_used_tenth = (int)((s_host_hdd_used_gb - h_used_whole) * 10.0 + 0.5);
            int h_total_whole = (int)s_host_hdd_total_gb;
            int h_total_tenth = (int)((s_host_hdd_total_gb - h_total_whole) * 10.0 + 0.5);
            snprintf(hdd_buf, sizeof(hdd_buf), "HDD\n%d.%d / %d.%d GB", h_used_whole, h_used_tenth, h_total_whole, h_total_tenth);
            lv_label_set_text(s_host_hdd_label, hdd_buf);
        } else {
            lv_bar_set_value(s_host_hdd_bar, 0, LV_ANIM_OFF);
            lv_label_set_text(s_host_hdd_label, "HDD\nN/A");
        }
    }

    /* Update clock HH:MM — prefer host-provided values, fall back to device local time */
    if (s_clock_label) {
        int display_h = s_host_clock_h;
        int display_m = s_host_clock_m;
        if (display_h < 0) {
            time_t now = time(NULL);
            struct tm tm;
            localtime_r(&now, &tm);
            display_h = tm.tm_hour;
            display_m = tm.tm_min;
        }
        lv_label_set_text_fmt(s_clock_label, "%02d:%02d", display_h, display_m);
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Display sleep
 *  The sleep/wake cycle runs in a dedicated FreeRTOS task so the LVGL thread
 *  remains available to poll touch input while display refresh is paused.
 * ════════════════════════════════════════════════════════════════════════════ */
static void sleep_touch_cb(lv_event_t *e)
{
    (void)e;
    s_last_activity_tick = xTaskGetTickCount();
    sleep_wake();
}

static void sleep_wake(void)
{
    TaskHandle_t task = s_sleep_task_handle;
    if (s_display_is_sleeping && task) {
        xTaskNotifyGive(task);
    }
}

static void sleep_start(void)
{
    if (s_sleep_task_handle) {
        return;
    }

    lv_obj_add_flag(s_sleep_btn, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(s_sleep_label,
                      LV_SYMBOL_EYE_CLOSE "  Touch screen or send UART data to wake");

    BaseType_t rc = xTaskCreatePinnedToCore(sleep_task, "sleep", 4096, NULL, 3, NULL, 0);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sleep task");
        lv_label_set_text(s_sleep_label, "Sleep unavailable");
        lv_obj_remove_flag(s_sleep_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

static void sleep_task(void *arg)
{
    (void)arg;
    s_sleep_task_handle = xTaskGetCurrentTaskHandle();

    s_display_is_sleeping = true;
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "Entering display sleep until touch or UART data");
    esp_err_t err;
    do {
        err = bsp_display_enter_sleep();
        if (err == ESP_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    } while (err == ESP_ERR_TIMEOUT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bsp_display_enter_sleep failed: %s", esp_err_to_name(err));
        s_display_is_sleeping = false;
        goto done;
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "Exiting display sleep");
    s_display_is_sleeping = false;
    do {
        err = bsp_display_exit_sleep();
        if (err == ESP_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    } while (err == ESP_ERR_TIMEOUT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bsp_display_exit_sleep failed: %s", esp_err_to_name(err));
    }
    bsp_display_brightness_set(s_brightness_value);

done:
    if (bsp_display_lock(portMAX_DELAY)) {
        lv_label_set_text(s_sleep_label,
                          err == ESP_OK ? "Awake"
                                       : "Sleep unavailable");
        s_sleep_task_handle = NULL;
        lv_obj_remove_flag(s_sleep_btn, LV_OBJ_FLAG_HIDDEN);
        bsp_display_unlock();
    } else {
        s_sleep_task_handle = NULL;
    }

    vTaskDelete(NULL);
}

static void sleep_btn_cb(lv_event_t *e)
{
    (void)e;
    s_last_activity_tick = xTaskGetTickCount();
    sleep_start();
}

/* ════════════════════════════════════════════════════════════════════════════
 *  UI builder
 *  Must be called inside bsp_display_lock() / bsp_display_unlock().
 * ════════════════════════════════════════════════════════════════════════════ */
static void ui_create(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* ── Tab view ─────────────────────────────────────────────────────────── */
    lv_obj_t *tv = lv_tabview_create(scr);
    lv_tabview_set_tab_bar_position(tv, LV_DIR_TOP);
    lv_tabview_set_tab_bar_size(tv, 52);
    lv_obj_set_size(tv, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_set_style_bg_color(tv, COL_BG, 0);

    /* Tab bar */
    lv_obj_t *tab_bar = lv_tabview_get_tab_bar(tv);
    lv_obj_set_style_bg_color(tab_bar, COL_BORDER, 0);
    lv_obj_set_style_bg_opa(tab_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(tab_bar, COL_TEXT, 0);
    lv_obj_set_style_text_font(tab_bar, &lv_font_montserrat_18, 0);

    lv_obj_t *tab_temp       = lv_tabview_add_tab(tv, LV_SYMBOL_HOME    " System");
    lv_obj_t *tab_stream_deck  = lv_tabview_add_tab(tv, LV_SYMBOL_PLAY     " Stream Deck");
    lv_obj_t *tab_settings   = lv_tabview_add_tab(tv, LV_SYMBOL_SETTINGS " Settings");
    lv_obj_t *tab_usb        = lv_tabview_add_tab(tv, LV_SYMBOL_USB      " USB");

    lv_obj_t *tabs[] = {tab_temp, tab_stream_deck, tab_settings, tab_usb};
    for (int i = 0; i < 4; i++) {
        lv_obj_set_style_bg_color(tabs[i], COL_BG, 0);
        lv_obj_set_style_bg_opa(tabs[i], LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(tabs[i], 20, 0);
    }

    /* ── Tab 1: Stream Deck ───────────────────────────────────────────────────── */
    lv_obj_set_flex_flow(tab_stream_deck, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tab_stream_deck, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(tab_stream_deck, 18, 0);
    lv_obj_set_style_bg_color(tab_stream_deck, COL_BG, 0);
    lv_obj_set_style_bg_opa(tab_stream_deck, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(tab_stream_deck, 0, 0);
    lv_obj_set_style_outline_width(tab_stream_deck, 0, 0);
    lv_obj_clear_flag(tab_stream_deck, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(tab_stream_deck, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *stream_deck_grid = lv_obj_create(tab_stream_deck);
    lv_obj_set_size(stream_deck_grid, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(stream_deck_grid, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(stream_deck_grid, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(stream_deck_grid, COL_BG, 0);
    lv_obj_set_style_bg_opa(stream_deck_grid, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(stream_deck_grid, 0, 0);
    lv_obj_set_style_outline_width(stream_deck_grid, 0, 0);
    lv_obj_clear_flag(stream_deck_grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(stream_deck_grid, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_row(stream_deck_grid, 18, 0);
    lv_obj_set_style_pad_column(stream_deck_grid, 18, 0);

    lv_obj_t *vol_col = lv_obj_create(stream_deck_grid);
    lv_obj_set_size(vol_col, lv_pct(25), lv_pct(100));
    lv_obj_set_flex_flow(vol_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(vol_col, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(vol_col, COL_BG, 0);
    lv_obj_set_style_bg_opa(vol_col, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(vol_col, 0, 0);
    lv_obj_set_style_outline_width(vol_col, 0, 0);
    lv_obj_clear_flag(vol_col, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(vol_col, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_row(vol_col, 16, 0);

    lv_obj_t *mute_col = lv_obj_create(stream_deck_grid);
    lv_obj_set_size(mute_col, lv_pct(25), lv_pct(100));
    lv_obj_set_flex_flow(mute_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(mute_col, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(mute_col, COL_BG, 0);
    lv_obj_set_style_bg_opa(mute_col, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(mute_col, 0, 0);
    lv_obj_set_style_outline_width(mute_col, 0, 0);
    lv_obj_clear_flag(mute_col, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(mute_col, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_row(mute_col, 16, 0);

    lv_obj_t *bright_col = lv_obj_create(stream_deck_grid);
    lv_obj_set_size(bright_col, lv_pct(25), lv_pct(100));
    lv_obj_set_flex_flow(bright_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(bright_col, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(bright_col, COL_BG, 0);
    lv_obj_set_style_bg_opa(bright_col, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(bright_col, 0, 0);
    lv_obj_set_style_outline_width(bright_col, 0, 0);
    lv_obj_clear_flag(bright_col, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(bright_col, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_row(bright_col, 16, 0);

    lv_obj_t *apps_col = lv_obj_create(stream_deck_grid);
    lv_obj_set_size(apps_col, lv_pct(25), lv_pct(100));
    lv_obj_set_flex_flow(apps_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(apps_col, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(apps_col, COL_BG, 0);
    lv_obj_set_style_bg_opa(apps_col, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(apps_col, 0, 0);
    lv_obj_set_style_outline_width(apps_col, 0, 0);
    lv_obj_clear_flag(apps_col, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(apps_col, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_row(apps_col, 42, 0);

    struct {
        const char *label;
        const char *cmd;
        lv_color_t color;
        lv_obj_t *parent;
    } deck_actions[] = {
        { LV_SYMBOL_VOLUME_MAX "  Vol", "VOL_UP", lv_color_hex(0x69AD00), vol_col },
        { LV_SYMBOL_VOLUME_MID "  Vol", "VOL_DOWN", lv_color_hex(0x69AD00), vol_col },
        { LV_SYMBOL_CLOSE " Mic", "MUTE_MIC", lv_color_hex(0x8F89FA), mute_col },
        { LV_SYMBOL_CLOSE " Speaker", "MUTE_SPEAKER", lv_color_hex(0x8F89FA), mute_col },
        { LV_SYMBOL_PLUS "  Bright", "BRIGHT_UP", lv_color_hex(0xFFAC14), bright_col },
        { LV_SYMBOL_MINUS "  Bright", "BRIGHT_DOWN", lv_color_hex(0xFFAC14), bright_col },
        { LV_SYMBOL_KEYBOARD, "STEAM", lv_color_hex(0x0088C7), apps_col },
        { LV_SYMBOL_PLAY, "YOUTUBE", lv_color_hex(0xFF0000), apps_col },
        { LV_SYMBOL_AUDIO, "SPOTIFY", lv_color_hex(0x69AD00), apps_col }
    };

    for (size_t i = 0; i < sizeof(deck_actions) / sizeof(deck_actions[0]); i++) {
        lv_obj_t *btn = lv_btn_create(deck_actions[i].parent);
        lv_obj_set_size(btn, lv_pct(100), 100);
        lv_obj_set_style_bg_color(btn, deck_actions[i].color, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(btn, 18, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scrollbar_mode(btn, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_event_cb(btn, stream_deck_btn_cb, LV_EVENT_CLICKED, (void *)deck_actions[i].cmd);

        if (strcmp(deck_actions[i].cmd, "STEAM") == 0) {
            lv_obj_t *steam_icon = lv_img_create(btn);
            lv_img_set_src(steam_icon, &steam_logo);
            lv_obj_set_size(steam_icon, 72, 24);
            lv_obj_center(steam_icon);
            lv_obj_set_style_image_recolor_opa(steam_icon, LV_OPA_TRANSP, 0);
        } else {
            lv_obj_t *btn_label = lv_label_create(btn);
            lv_label_set_text(btn_label, deck_actions[i].label);
            lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_18, 0);
            lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);
            lv_obj_clear_flag(btn_label, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_scrollbar_mode(btn_label, LV_SCROLLBAR_MODE_OFF);
            lv_obj_center(btn_label);
        }
    }

    /* ── Tab 2: Settings ───────────────────────────────────────────────────── */
    lv_obj_set_flex_flow(tab_settings, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tab_settings, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(tab_settings, 28, 0);

    lv_obj_t *bl_title = lv_label_create(tab_settings);
    lv_label_set_text(bl_title, "Screen Brightness");
    lv_obj_set_style_text_font(bl_title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(bl_title, COL_MUTED, 0);

    int saved_brightness = brightness_load();

    lv_obj_t *slider = lv_slider_create(tab_settings);
    lv_slider_set_range(slider, 1, 100);
    lv_slider_set_value(slider, saved_brightness, LV_ANIM_OFF);
    lv_obj_set_width(slider, 640);
    lv_obj_set_height(slider, 28);
    lv_obj_set_style_bg_color(slider, COL_CARD,   LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, COL_BORDER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, COL_ACCENT, LV_PART_KNOB);
    lv_obj_set_style_pad_all(slider, 8, LV_PART_KNOB);
    lv_obj_add_event_cb(slider, brightness_cb, LV_EVENT_VALUE_CHANGED, NULL);

    s_brightness_label = lv_label_create(tab_settings);
    lv_label_set_text_fmt(s_brightness_label, "%d%%", saved_brightness);
    lv_obj_set_style_text_font(s_brightness_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_brightness_label, COL_ACCENT, 0);

    lv_obj_t *sleep_title = lv_label_create(tab_settings);
    lv_label_set_text(sleep_title, "Display Sleep");
    lv_obj_set_style_text_font(sleep_title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(sleep_title, COL_MUTED, 0);

    s_sleep_btn = lv_btn_create(tab_settings);
    lv_obj_set_size(s_sleep_btn, 280, 64);
    lv_obj_set_style_bg_color(s_sleep_btn, COL_ACCENT, 0);

    lv_obj_t *btn_lbl = lv_label_create(s_sleep_btn);
    lv_label_set_text(btn_lbl, "Sleep");
    lv_obj_set_style_text_font(btn_lbl, &lv_font_montserrat_18, 0);
    lv_obj_center(btn_lbl);
    lv_obj_add_event_cb(s_sleep_btn, sleep_btn_cb, LV_EVENT_CLICKED, NULL);

    s_sleep_label = lv_label_create(tab_settings);
    lv_label_set_text(s_sleep_label, "Auto-sleep after 5 minutes without activity");
    lv_obj_set_style_text_font(s_sleep_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_sleep_label, COL_MUTED, 0);

    lv_indev_t *touch_indev = bsp_display_get_input_dev();
    if (touch_indev) {
        lv_indev_add_event_cb(touch_indev, sleep_touch_cb, LV_EVENT_PRESSED, NULL);
    }

    /* ── Tab 2: USB ───────────────────────────────────────────────────────── */
    lv_obj_set_flex_flow(tab_usb, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tab_usb, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(tab_usb, 10, 0);

    lv_obj_t *usb_title = lv_label_create(tab_usb);
    lv_label_set_text(usb_title, "USB Drive Contents");
    lv_obj_set_style_text_font(usb_title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(usb_title, COL_MUTED, 0);

    s_usb_status = lv_label_create(tab_usb);
    lv_label_set_text(s_usb_status, LV_SYMBOL_USB "  No USB drive connected");
    lv_obj_set_style_text_color(s_usb_status, COL_MUTED, 0);
    lv_obj_set_style_text_font(s_usb_status, &lv_font_montserrat_16, 0);

    s_usb_list = lv_list_create(tab_usb);
    lv_obj_set_width(s_usb_list, lv_pct(100));
    lv_obj_set_flex_grow(s_usb_list, 1);
    lv_obj_set_style_bg_color(s_usb_list, COL_CARD, 0);
    lv_obj_set_style_bg_opa(s_usb_list, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_usb_list, COL_BORDER, 0);
    lv_obj_set_style_border_width(s_usb_list, 1, 0);
    lv_obj_set_style_text_color(s_usb_list, COL_TEXT, 0);
    lv_obj_set_style_text_font(s_usb_list, &lv_font_montserrat_16, 0);
    lv_obj_add_flag(s_usb_list, LV_OBJ_FLAG_HIDDEN);

    /* ── Tab 3: System ────────────────────────────────────────────────── */
    lv_obj_set_flex_flow(tab_temp, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tab_temp, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(tab_temp, 14, 0);
    lv_obj_clear_flag(tab_temp, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(tab_temp, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *row_top = lv_obj_create(tab_temp);
    lv_obj_set_size(row_top, lv_pct(100), 180);
    lv_obj_set_flex_flow(row_top, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row_top, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(row_top, COL_BG, 0);
    lv_obj_set_style_bg_opa(row_top, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(row_top, 0, 0);
    lv_obj_clear_flag(row_top, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(row_top, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *gpu_card = lv_obj_create(row_top);
    lv_obj_set_size(gpu_card, 150, 150);
    lv_obj_set_flex_flow(gpu_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(gpu_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(gpu_card, COL_CARD, 0);
    lv_obj_set_style_bg_opa(gpu_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(gpu_card, COL_BORDER, 0);
    lv_obj_set_style_border_width(gpu_card, 1, 0);
    lv_obj_set_style_pad_all(gpu_card, 8, 0);
    lv_obj_remove_flag(gpu_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(gpu_card, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_set_scrollbar_mode(gpu_card, LV_SCROLLBAR_MODE_OFF);

    s_gpu_temp_label = lv_label_create(gpu_card);
    lv_obj_set_style_text_font(s_gpu_temp_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_gpu_temp_label, COL_CYAN, 0);
    lv_obj_set_style_text_align(s_gpu_temp_label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *cpu_card = lv_obj_create(row_top);
    lv_obj_set_size(cpu_card, 150, 150);
    lv_obj_set_flex_flow(cpu_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cpu_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(cpu_card, COL_CARD, 0);
    lv_obj_set_style_bg_opa(cpu_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(cpu_card, COL_BORDER, 0);
    lv_obj_set_style_border_width(cpu_card, 1, 0);
    lv_obj_set_style_pad_all(cpu_card, 8, 0);
    lv_obj_remove_flag(cpu_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(cpu_card, LV_OBJ_FLAG_CLICK_FOCUSABLE);

    s_cpu_temp_label = lv_label_create(cpu_card);
    lv_obj_set_style_text_font(s_cpu_temp_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_cpu_temp_label, COL_ACCENT, 0);
    lv_obj_set_style_text_align(s_cpu_temp_label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *liquid_temp_card = lv_obj_create(row_top);
    lv_obj_set_size(liquid_temp_card, 150, 150);
    lv_obj_set_flex_flow(liquid_temp_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(liquid_temp_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(liquid_temp_card, COL_CARD, 0);
    lv_obj_set_style_bg_opa(liquid_temp_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(liquid_temp_card, COL_BORDER, 0);
    lv_obj_set_style_border_width(liquid_temp_card, 1, 0);
    lv_obj_set_style_pad_all(liquid_temp_card, 8, 0);
    lv_obj_remove_flag(liquid_temp_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(liquid_temp_card, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_set_scrollbar_mode(liquid_temp_card, LV_SCROLLBAR_MODE_OFF);

    s_liquid_temp_label = lv_label_create(liquid_temp_card);
    lv_obj_set_style_text_font(s_liquid_temp_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_liquid_temp_label, COL_ACCENT, 0);
    lv_obj_set_style_text_align(s_liquid_temp_label, LV_TEXT_ALIGN_CENTER, 0);

    /* Secondary row: GPU fan, CPU fan, storage usage, and clock. */
    lv_obj_t *row_extra = lv_obj_create(tab_temp);
    lv_obj_set_size(row_extra, lv_pct(100), 180);
    lv_obj_set_flex_flow(row_extra, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row_extra, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(row_extra, COL_BG, 0);
    lv_obj_set_style_bg_opa(row_extra, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(row_extra, 0, 0);
    lv_obj_clear_flag(row_extra, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(row_extra, LV_SCROLLBAR_MODE_OFF);

    /* Host CPU usage card */
    lv_obj_t *host_cpu_card = lv_obj_create(row_top);
    lv_obj_set_size(host_cpu_card, 150, 150);
    lv_obj_set_flex_flow(host_cpu_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(host_cpu_card, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(host_cpu_card, COL_CARD, 0);
    lv_obj_set_style_bg_opa(host_cpu_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(host_cpu_card, COL_BORDER, 0);
    lv_obj_set_style_border_width(host_cpu_card, 1, 0);
    lv_obj_set_style_pad_all(host_cpu_card, 8, 0);
    lv_obj_remove_flag(host_cpu_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(host_cpu_card, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_set_scrollbar_mode(host_cpu_card, LV_SCROLLBAR_MODE_OFF);

    s_host_cpu_label = lv_label_create(host_cpu_card);
    lv_label_set_text(s_host_cpu_label, "CPU USAGE \n0%");
    lv_obj_set_style_text_font(s_host_cpu_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(s_host_cpu_label, COL_MUTED, 0);
    lv_obj_set_style_text_align(s_host_cpu_label, LV_TEXT_ALIGN_CENTER, 0);

    s_host_cpu_gauge = lv_bar_create(host_cpu_card);
    lv_obj_set_size(s_host_cpu_gauge, 120, 12);
    lv_bar_set_range(s_host_cpu_gauge, 0, 1000);
    lv_bar_set_value(s_host_cpu_gauge, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_host_cpu_gauge, COL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_host_cpu_gauge, COL_ACCENT, LV_PART_INDICATOR);

    /* GPU usage (same card) */
    s_host_gpu_label = lv_label_create(host_cpu_card);
    lv_label_set_text(s_host_gpu_label, "GPU USAGE\n0%");
    lv_obj_set_style_text_font(s_host_gpu_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(s_host_gpu_label, COL_MUTED, 0);
    lv_obj_set_style_text_align(s_host_gpu_label, LV_TEXT_ALIGN_CENTER, 0);

    s_host_gpu_gauge = lv_bar_create(host_cpu_card);
    lv_obj_set_size(s_host_gpu_gauge, 120, 12);
    lv_bar_set_range(s_host_gpu_gauge, 0, 1000);
    lv_bar_set_value(s_host_gpu_gauge, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_host_gpu_gauge, COL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_host_gpu_gauge, COL_CYAN, LV_PART_INDICATOR);

    lv_obj_t *host_storage_card = lv_obj_create(row_extra);
    lv_obj_set_size(host_storage_card, 150, 150);
    lv_obj_set_flex_flow(host_storage_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(host_storage_card, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(host_storage_card, COL_CARD, 0);
    lv_obj_set_style_bg_opa(host_storage_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(host_storage_card, COL_BORDER, 0);
    lv_obj_set_style_border_width(host_storage_card, 1, 0);
    lv_obj_set_style_pad_all(host_storage_card, 8, 0);

    s_host_ram_label = lv_label_create(host_storage_card);
    lv_label_set_text(s_host_ram_label, "RAM\n0.0 / 0.0 GB");
    lv_obj_set_style_text_font(s_host_ram_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_host_ram_label, COL_MUTED, 0);
    lv_obj_set_style_text_align(s_host_ram_label, LV_TEXT_ALIGN_CENTER, 0);

    s_host_ram_bar = lv_bar_create(host_storage_card);
    lv_obj_set_size(s_host_ram_bar, 120, 10);
    lv_bar_set_range(s_host_ram_bar, 0, 1000);
    lv_bar_set_value(s_host_ram_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_host_ram_bar, COL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_host_ram_bar, COL_ACCENT, LV_PART_INDICATOR);

    s_host_hdd_label = lv_label_create(host_storage_card);
    lv_label_set_text(s_host_hdd_label, "HDD\n0.0 / 0.0 GB");
    lv_obj_set_style_text_font(s_host_hdd_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_host_hdd_label, COL_MUTED, 0);
    lv_obj_set_style_text_align(s_host_hdd_label, LV_TEXT_ALIGN_CENTER, 0);

    s_host_hdd_bar = lv_bar_create(host_storage_card);
    lv_obj_set_size(s_host_hdd_bar, 120, 10);
    lv_bar_set_range(s_host_hdd_bar, 0, 1000);
    lv_bar_set_value(s_host_hdd_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_host_hdd_bar, COL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_host_hdd_bar, COL_CYAN, LV_PART_INDICATOR);
    lv_obj_set_style_border_color(s_host_hdd_bar, COL_BORDER, 0);

    /* Clock card */
    lv_obj_t *clock_card = lv_obj_create(row_extra);
    lv_obj_set_size(clock_card, 150, 150);
    lv_obj_set_flex_flow(clock_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(clock_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(clock_card, COL_CARD, 0);
    lv_obj_set_style_bg_opa(clock_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(clock_card, COL_BORDER, 0);
    lv_obj_set_style_border_width(clock_card, 1, 0);
    lv_obj_set_style_pad_all(clock_card, 8, 0);
    lv_obj_set_scrollbar_mode(clock_card, LV_SCROLLBAR_MODE_OFF);

    s_clock_label = lv_label_create(clock_card);
    lv_label_set_text(s_clock_label, "00:00");
    lv_obj_set_style_text_font(s_clock_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_clock_label, COL_TEXT, 0);
    lv_obj_set_style_text_align(s_clock_label, LV_TEXT_ALIGN_CENTER, 0);

    // lv_obj_t *row_bottom = lv_obj_create(tab_temp);
    // lv_obj_set_size(row_bottom, lv_pct(100), 180);
    // lv_obj_set_flex_flow(row_bottom, LV_FLEX_FLOW_ROW);
    // lv_obj_set_flex_align(row_bottom, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    // lv_obj_set_style_bg_color(row_bottom, COL_BG, 0);
    // lv_obj_set_style_bg_opa(row_bottom, LV_OPA_COVER, 0);
    // lv_obj_set_style_border_width(row_bottom, 0, 0);
    // lv_obj_clear_flag(row_bottom, LV_OBJ_FLAG_SCROLLABLE);
    // lv_obj_set_scrollbar_mode(row_bottom, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *gpu_fan_card = lv_obj_create(row_extra);
    lv_obj_set_size(gpu_fan_card, 150, 150);
    lv_obj_set_flex_flow(gpu_fan_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(gpu_fan_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(gpu_fan_card, COL_CARD, 0);
    lv_obj_set_style_bg_opa(gpu_fan_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(gpu_fan_card, COL_BORDER, 0);
    lv_obj_set_style_border_width(gpu_fan_card, 1, 0);
    lv_obj_set_style_pad_all(gpu_fan_card, 8, 0);
    lv_obj_remove_flag(gpu_fan_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(gpu_fan_card, LV_OBJ_FLAG_CLICK_FOCUSABLE);

    s_gpu_fan_label = lv_label_create(gpu_fan_card);
    lv_obj_set_style_text_font(s_gpu_fan_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_gpu_fan_label, COL_CYAN, 0);
    lv_obj_set_style_text_align(s_gpu_fan_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_clear_flag(gpu_fan_card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(gpu_fan_card, LV_SCROLLBAR_MODE_OFF);

    s_gpu_fan_gauge = lv_scale_create(gpu_fan_card);
    lv_obj_clear_flag(s_gpu_fan_gauge, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(s_gpu_fan_gauge, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(s_gpu_fan_gauge, 100, 100);
    lv_scale_set_mode(s_gpu_fan_gauge, LV_SCALE_MODE_ROUND_INNER);
    lv_scale_set_range(s_gpu_fan_gauge, 0, 10000);
    lv_scale_set_total_tick_count(s_gpu_fan_gauge, 12);
    lv_scale_set_major_tick_every(s_gpu_fan_gauge, 2);
    lv_scale_set_angle_range(s_gpu_fan_gauge, 260);
    lv_scale_set_rotation(s_gpu_fan_gauge, 130);
    lv_scale_set_label_show(s_gpu_fan_gauge, false);
    lv_obj_set_style_arc_width(s_gpu_fan_gauge, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_gpu_fan_gauge, COL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_gpu_fan_gauge, COL_CYAN, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_gpu_fan_gauge, LV_OPA_TRANSP, 0);
    lv_obj_remove_flag(s_gpu_fan_gauge, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(s_gpu_fan_gauge, LV_OBJ_FLAG_CLICK_FOCUSABLE);

    s_gpu_fan_needle = lv_line_create(s_gpu_fan_gauge);
    lv_obj_set_style_line_color(s_gpu_fan_needle, COL_CYAN, LV_PART_MAIN);
    lv_obj_set_style_line_width(s_gpu_fan_needle, 8, LV_PART_MAIN);
    lv_obj_set_style_length(s_gpu_fan_needle, 42, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(s_gpu_fan_needle, false, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_gpu_fan_needle, 26, LV_PART_MAIN);
    lv_obj_set_style_pad_left(s_gpu_fan_needle, 8, LV_PART_MAIN);
    lv_scale_set_line_needle_value(s_gpu_fan_gauge, s_gpu_fan_needle, 52, 0);

    lv_obj_t *gpu_fan_hub = lv_obj_create(s_gpu_fan_gauge);
    lv_obj_set_size(gpu_fan_hub, 14, 14);
    lv_obj_set_style_radius(gpu_fan_hub, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(gpu_fan_hub, COL_CYAN, 0);
    lv_obj_set_style_bg_opa(gpu_fan_hub, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(gpu_fan_hub, 0, 0);
    lv_obj_align(gpu_fan_hub, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *cpu_fan_card = lv_obj_create(row_extra);
    lv_obj_set_size(cpu_fan_card, 150, 150);
    lv_obj_set_flex_flow(cpu_fan_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cpu_fan_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(cpu_fan_card, COL_CARD, 0);
    lv_obj_set_style_bg_opa(cpu_fan_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(cpu_fan_card, COL_BORDER, 0);
    lv_obj_set_style_border_width(cpu_fan_card, 1, 0);
    lv_obj_set_style_pad_all(cpu_fan_card, 8, 0);
    lv_obj_remove_flag(cpu_fan_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(cpu_fan_card, LV_OBJ_FLAG_CLICK_FOCUSABLE);

    s_cpu_fan_label = lv_label_create(cpu_fan_card);
    lv_obj_set_style_text_font(s_cpu_fan_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_cpu_fan_label, COL_ACCENT, 0);
    lv_obj_set_style_text_align(s_cpu_fan_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_clear_flag(cpu_fan_card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(cpu_fan_card, LV_SCROLLBAR_MODE_OFF);

    s_cpu_fan_gauge = lv_scale_create(cpu_fan_card);
    lv_obj_clear_flag(s_cpu_fan_gauge, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(s_cpu_fan_gauge, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(s_cpu_fan_gauge, 100, 100);
    lv_scale_set_mode(s_cpu_fan_gauge, LV_SCALE_MODE_ROUND_INNER);
    lv_scale_set_range(s_cpu_fan_gauge, 0, 10000);
    lv_scale_set_total_tick_count(s_cpu_fan_gauge, 12);
    lv_scale_set_major_tick_every(s_cpu_fan_gauge, 2);
    lv_scale_set_angle_range(s_cpu_fan_gauge, 260);
    lv_scale_set_rotation(s_cpu_fan_gauge, 130);
    lv_scale_set_label_show(s_cpu_fan_gauge, false);
    lv_obj_set_style_arc_width(s_cpu_fan_gauge, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_cpu_fan_gauge, COL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_cpu_fan_gauge, COL_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_cpu_fan_gauge, LV_OPA_TRANSP, 0);
    lv_obj_remove_flag(s_cpu_fan_gauge, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(s_cpu_fan_gauge, LV_OBJ_FLAG_CLICK_FOCUSABLE);

    s_cpu_fan_needle = lv_line_create(s_cpu_fan_gauge);
    lv_obj_set_style_line_color(s_cpu_fan_needle, COL_ACCENT, LV_PART_MAIN);
    lv_obj_set_style_line_width(s_cpu_fan_needle, 8, LV_PART_MAIN);
    lv_obj_set_style_length(s_cpu_fan_needle, 42, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(s_cpu_fan_needle, false, LV_PART_MAIN);
    lv_obj_set_style_pad_right(s_cpu_fan_needle, 26, LV_PART_MAIN);
    lv_obj_set_style_pad_left(s_cpu_fan_needle, 8, LV_PART_MAIN);
    lv_scale_set_line_needle_value(s_cpu_fan_gauge, s_cpu_fan_needle, 52, 0);

    lv_obj_t *cpu_fan_hub = lv_obj_create(s_cpu_fan_gauge);
    lv_obj_set_size(cpu_fan_hub, 14, 14);
    lv_obj_set_style_radius(cpu_fan_hub, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(cpu_fan_hub, COL_ACCENT, 0);
    lv_obj_set_style_bg_opa(cpu_fan_hub, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cpu_fan_hub, 0, 0);
    lv_obj_align(cpu_fan_hub, LV_ALIGN_CENTER, 0, 0);

    lv_obj_move_to_index(gpu_fan_card, 0);
    lv_obj_move_to_index(cpu_fan_card, 1);

    temp_value_update(s_cpu_temp_label, s_cpu_temp_value, "CPU");
    temp_value_update(s_gpu_temp_label, s_gpu_temp_value, "GPU");
    temp_value_update(s_liquid_temp_label, s_liquid_temp_value, "LIQ");
    lv_label_set_text_fmt(s_cpu_fan_label, "CPU_FAN\n%d RPM", s_cpu_fan_value == 0 ? 0 : s_cpu_fan_value);
    lv_label_set_text_fmt(s_gpu_fan_label, "GPU_FAN\n%d RPM", s_gpu_fan_value == 0 ? 0 : s_gpu_fan_value);
    fan_arc_update(s_cpu_fan_gauge, s_cpu_fan_needle, s_cpu_fan_value);
    fan_arc_update(s_gpu_fan_gauge, s_gpu_fan_needle, s_gpu_fan_value);

    lv_timer_create(cpu_temp_timer_cb, 2000, NULL);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  LVGL custom allocator — routes LVGL heap to PSRAM
 *
 *  LVGL's default internal TLSF heap is ~64 KB of SRAM, which is too small
 *  for the 768 KB sub-layer buffer it needs when compositing the USB list.
 *  Without this, lv_draw_layer_alloc_buf() fails, the draw task stays WAITING
 *  forever, draw_buf_flush() spins infinitely and the WDT fires.
 *
 *  Enabled by CONFIG_LV_USE_CUSTOM_MALLOC=y in sdkconfig.defaults.
 * ════════════════════════════════════════════════════════════════════════════ */
void lv_mem_init(void)    { }
void lv_mem_deinit(void)  { }

void *lv_malloc_core(size_t size)
{
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return p;
}

void *lv_realloc_core(void *p, size_t new_size)
{
    if (new_size == 0) {
        heap_caps_free(p);
        return NULL;
    }
    void *new_p = heap_caps_realloc(p, new_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!new_p) {
        new_p = heap_caps_realloc(p, new_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return new_p;
}

void lv_free_core(void *p)
{
    heap_caps_free(p);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Entry point
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    cpu_temp_uart_init();

    lv_display_t *disp = bsp_display_start();
    assert(disp);
    int boot_brightness = brightness_load();
    bsp_display_brightness_set(boot_brightness);

    bsp_display_lock(portMAX_DELAY);
    ui_create();
    bsp_display_unlock();

    bsp_usb_on_mount(on_usb_mount);
    bsp_usb_on_unmount(on_usb_unmount);
    if (bsp_usb_start() != ESP_OK) {
        ESP_LOGW(TAG, "USB MSC host init failed — USB tab will show 'not connected'");
    }
}
