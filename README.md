# PandaTouch PlatformIO

This project is based on the original [BigTreeTech PandaTouch PlatformIO template](https://github.com/bigtreetech/PandaTouch_PlatformIO), which provides the starting point for working with the PandaTouch hardware. The template's starter demo has been adapted into a system-monitoring and desktop-control application.

The firmware targets the ESP32-S3 PandaTouch and uses ESP-IDF through PlatformIO. It drives the 800×480 RGB display, GT911 touch panel, and USB host, with an LVGL interface.

## What the project does

The touchscreen UI has four tabs:

- **System** — displays CPU, GPU, and liquid temperatures; CPU and GPU fan speeds; host CPU and GPU usage; RAM and disk usage; and the host clock.
- **Stream Deck** — sends button commands to the connected computer for volume, microphone/speaker mute, brightness, and launching Steam, YouTube, or Spotify.
- **Settings** — adjusts and saves screen brightness, and provides manual display sleep. The display also sleeps after five minutes without touch or UART activity and can wake on touch or UART data.
- **USB** — browses files and directories on a USB drive connected to the PandaTouch USB-A host port.

The companion [`send_cpu_temp_cpp.cpp`](./send_cpu_temp_cpp.cpp) program runs on a Linux host. It reads available host and hardware sensor metrics, sends telemetry to the PandaTouch over serial, and listens for Stream Deck commands to execute locally.

## UART connection

The firmware expects the host serial link at **115200 baud**:

| Signal | PandaTouch GPIO |
| --- | --- |
| RX | GPIO 44 |
| TX | GPIO 43 |
| Ground | GND |

The host utility defaults to `/dev/ttyUSB0`; update `SERIAL_PORT` in the source if your serial device uses a different path. Telemetry is sent as newline-terminated CSV records.

## Build and flash the firmware

Install [PlatformIO](https://platformio.org/install/ide?install=vscode), connect the PandaTouch, then run:

```bash
pio run -e pandatouch
pio run -e pandatouch -t upload --upload-port /dev/ttyUSB0
```

Use the appropriate serial device for your system. To open the PlatformIO serial monitor:

```bash
pio device monitor -b 115200
```

The PlatformIO project is configured in [`platformio.ini`](./platformio.ini), with the firmware entry point under [`src/main`](./src/main/).

## Build the Linux host utility

On a Linux host with a C++17 compiler:

```bash
g++ -std=c++17 -Wall -Wextra -O2 send_cpu_temp_cpp.cpp -o send_cpu_temp_cpp
./send_cpu_temp_cpp
```

Some metrics depend on hardware and installed utilities. Liquid-cooler readings, for example, use `liquidctl` when supported hardware is available.

## Hardware and resources

- [PandaTouch product and documentation](https://bttwiki.com/PandaTouch.html)
- [PandaTouch pinout](./docs/PINOUT.md)
- [LCD datasheet](./docs/QX05ZJGI70N-05545Y.pdf)
- [LVGL documentation](https://docs.lvgl.io/)
- [PlatformIO documentation](https://docs.platformio.org/)
