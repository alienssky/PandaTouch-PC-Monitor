#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <signal.h>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <sys/statvfs.h>
#include <cstdio>
#include <regex>

namespace fs = std::filesystem;

static const std::string SERIAL_PORT = "/dev/ttyUSB0";
static const int BAUDRATE = 115200;

static std::string trim(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

static int toIntFromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        return 0;
    }

    std::string text;
    std::getline(in, text);
    text = trim(text);
    if (text.empty()) {
        return 0;
    }

    try {
        return std::stoi(text);
    } catch (...) {
        return 0;
    }
}

static std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return s;
}

static std::vector<std::string> listThermalZones() {
    std::vector<std::string> out;
    const fs::path base = "/sys/class/thermal";
    if (!fs::exists(base)) {
        return out;
    }

    for (const auto& entry : fs::directory_iterator(base)) {
        if (!entry.is_directory()) {
            continue;
        }
        std::string name = entry.path().filename().string();
        if (name.rfind("thermal_zone", 0) == 0) {
            out.push_back(name);
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

static int readFirstValue(const std::vector<std::string>& paths) {
    for (const auto& path : paths) {
        const int value = toIntFromFile(path);
        if (value > 0) {
            return value;
        }
    }
    return 0;
}

static int readCpuHwmonTemp() {
    const std::vector<std::string> preferredDrivers = {
        "k10temp", "coretemp", "zenpower"
    };
    const fs::path base = "/sys/class/hwmon";
    if (!fs::exists(base)) {
        return 0;
    }

    for (const auto& entry : fs::directory_iterator(base)) {
        if (!entry.is_directory()) {
            continue;
        }

        std::ifstream nameFile(entry.path() / "name");
        std::string driver;
        if (!nameFile || !std::getline(nameFile, driver)) {
            continue;
        }
        driver = lower(trim(driver));

        bool isCpuDriver = false;
        for (const auto& preferred : preferredDrivers) {
            if (driver == preferred) {
                isCpuDriver = true;
                break;
            }
        }
        if (!isCpuDriver) {
            continue;
        }

        std::vector<std::string> temperaturePaths;
        for (const auto& sensor : fs::directory_iterator(entry.path())) {
            const std::string filename = sensor.path().filename().string();
            if (filename.rfind("temp", 0) == 0 &&
                filename.find("_input") != std::string::npos) {
                temperaturePaths.push_back(sensor.path().string());
            }
        }
        std::sort(temperaturePaths.begin(), temperaturePaths.end());
        const int value = readFirstValue(temperaturePaths);
        if (value > 0) {
            return value;
        }
    }

    return 0;
}

static int readCpuTemp() {
    std::vector<std::string> candidates = listThermalZones();
    const std::vector<std::string> preferred = {
        "coretemp", "cpu", "pkg", "x86", "k10temp", "zenpower", "cpu_thermal"
    };

    const int hwmonTemp = readCpuHwmonTemp();
    if (hwmonTemp > 0) {
        return hwmonTemp;
    }

    for (const auto& zone : candidates) {
        const std::string typePath = "/sys/class/thermal/" + zone + "/type";
        std::ifstream in(typePath);
        std::string type = "";
        if (in) {
            std::getline(in, type);
            type = lower(trim(type));
        }

        bool matchesPreferred = false;
        for (const auto& token : preferred) {
            if (type.find(token) != std::string::npos) {
                matchesPreferred = true;
                break;
            }
        }

        if (matchesPreferred) {
            const std::string tempPath = "/sys/class/thermal/" + zone + "/temp";
            const int v = readFirstValue({tempPath});
            if (v > 0) {
                return v;
            }
        }
    }

    for (const auto& zone : candidates) {
        const std::string typePath = "/sys/class/thermal/" + zone + "/type";
        std::ifstream in(typePath);
        std::string type;
        if (in) {
            std::getline(in, type);
            type = lower(trim(type));
        }
        if (type == "acpitz" || type == "thermal_sys") {
            continue;
        }

        const int v = readFirstValue({"/sys/class/thermal/" + zone + "/temp"});
        if (v > 0) {
            return v;
        }
    }

    return 0;
}

static int readGpuTemp() {
    const fs::path hwmonBase = "/sys/class/hwmon";
    const std::vector<std::string> gpuDrivers = {
        "amdgpu", "nvidia", "nouveau", "radeon", "i915"
    };
    std::vector<std::pair<bool, std::string>> candidates;

    if (fs::exists(hwmonBase)) {
        for (const auto& entry : fs::directory_iterator(hwmonBase)) {
            if (!entry.is_directory()) {
                continue;
            }

            std::ifstream nameFile(entry.path() / "name");
            std::string driver;
            if (!nameFile || !std::getline(nameFile, driver)) {
                continue;
            }
            driver = lower(trim(driver));
            if (std::find(gpuDrivers.begin(), gpuDrivers.end(), driver) == gpuDrivers.end()) {
                continue;
            }

            bool isBootGpu = false;
            const fs::path bootVgaPath = entry.path() / "device" / "boot_vga";
            if (fs::exists(bootVgaPath)) {
                isBootGpu = toIntFromFile(bootVgaPath.string()) == 1;
            }

            const fs::path primaryTemp = entry.path() / "temp1_input";
            if (fs::exists(primaryTemp)) {
                candidates.emplace_back(isBootGpu, primaryTemp.string());
                continue;
            }

            for (const auto& sensor : fs::directory_iterator(entry.path())) {
                const std::string filename = sensor.path().filename().string();
                if (filename.rfind("temp", 0) == 0 &&
                    filename.find("_input") != std::string::npos) {
                    candidates.emplace_back(isBootGpu, sensor.path().string());
                    break;
                }
            }
        }
    }

    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const auto& left, const auto& right) {
                         return left.first && !right.first;
                     });
    for (const auto& [isBootGpu, path] : candidates) {
        (void)isBootGpu;
        const int value = readFirstValue({path});
        if (value > 0) {
            return value;
        }
    }

    for (const auto& zone : listThermalZones()) {
        const std::string typePath = "/sys/class/thermal/" + zone + "/type";
        std::ifstream in(typePath);
        std::string type = "";
        if (in) {
            std::getline(in, type);
            type = lower(trim(type));
        }

        const std::vector<std::string> gpuTokens = {"gpu", "nvidia", "radeon", "amdgpu", "vga"};
        bool isGpu = false;
        for (const auto& token : gpuTokens) {
            if (type.find(token) != std::string::npos) {
                isGpu = true;
                break;
            }
        }

        if (isGpu) {
            const std::string tempPath = "/sys/class/thermal/" + zone + "/temp";
            const int value = readFirstValue({tempPath});
            if (value > 0) {
                return value;
            }
        }
    }

    return 0;
}

static int readFanRpm(const std::vector<std::string>& prefixes, bool fallbackToAny) {
    std::vector<std::string> candidates;

    const fs::path hwmonBase = "/sys/class/hwmon";
    if (!fs::exists(hwmonBase)) {
        return 0;
    }

    for (const auto& p : fs::directory_iterator(hwmonBase)) {
        if (!p.is_directory()) {
            continue;
        }
        const std::string base = p.path().string();
        for (const auto& entry : fs::directory_iterator(base)) {
            const std::string name = entry.path().filename().string();
            if (name.rfind("fan", 0) == 0 && name.find("_input") != std::string::npos) {
                std::string path = entry.path().string();
                candidates.push_back(path);
            }
        }
    }

    for (const auto& path : candidates) {
        const std::string parent = fs::path(path).parent_path().string();
        const std::string namePath = parent + "/name";
        std::ifstream in(namePath);
        std::string name = "";
        if (in) {
            std::getline(in, name);
            name = lower(trim(name));
        }

        const std::string fileName = lower(fs::path(path).filename().string());
        std::string label;
        std::ifstream labelFile(parent + "/" + fs::path(path).stem().string() + "_label");
        if (labelFile) {
            std::getline(labelFile, label);
            label = lower(trim(label));
        }

        bool matchesPrefix = false;
        for (const auto& prefix : prefixes) {
            const std::string pref = lower(prefix);
            if (name.find(pref) != std::string::npos ||
                label.find(pref) != std::string::npos ||
                fileName.find(pref) != std::string::npos) {
                matchesPrefix = true;
                break;
            }
        }

        if (matchesPrefix) {
            const int v = toIntFromFile(path);
            if (v > 0) {
                return v;
            }
        }
    }

    if (!fallbackToAny) {
        return 0;
    }

    for (const auto& path : candidates) {
        const int v = toIntFromFile(path);
        if (v > 0) {
            return v;
        }
    }

    return 0;
}

static int readKrakenStatus(int *liquidTemp) {
    if (liquidTemp) {
        *liquidTemp = 0;
    }

    FILE *fp = popen("liquidctl status 2>/dev/null", "r");
    if (!fp) {
        return 0;
    }

    const std::regex rpmPattern(
        R"((pump|fan)\s+speed[^0-9]*([0-9]+)\s*rpm)",
        std::regex_constants::icase);
    const std::regex tempPattern(
        R"(liquid\s+temperature[^0-9]*([0-9]+(?:\.[0-9]+)?))",
        std::regex_constants::icase);
    int fanRpm = 0;
    int pumpRpm = 0;
    char buffer[512];
    while (fgets(buffer, sizeof(buffer), fp)) {
        const std::string line(buffer);

        std::smatch match;
        if (std::regex_search(line, match, rpmPattern)) {
            try {
                const int rpm = std::stoi(match[2].str());
                if (lower(match[1].str()) == "fan" && rpm > 0) {
                    fanRpm = rpm;
                } else if (rpm > 0) {
                    pumpRpm = rpm;
                }
            } catch (...) {
            }
        }
        if (liquidTemp && std::regex_search(line, match, tempPattern)) {
            try {
                *liquidTemp = (int)std::round(std::stod(match[1].str()));
            } catch (...) {
            }
        }
    }

    pclose(fp);
    return fanRpm > 0 ? fanRpm : pumpRpm;
}

static int readCpuFanRpm(int *liquidTemp) {
    const int krakenRpm = readKrakenStatus(liquidTemp);
    if (krakenRpm > 0) {
        return krakenRpm;
    }
    return readFanRpm({"chassis", "case", "system", "sys"}, false);
}

static int readGpuFanRpm() {
    return readFanRpm({"gpu", "nvidia", "radeon", "amdgpu", "vga"}, false);
}

static int readHostGpuPercent() {
    const fs::path drmBase = "/sys/class/drm";
    std::vector<std::pair<bool, fs::path>> gpuDevices;
    if (fs::exists(drmBase)) {
        for (const auto& entry : fs::directory_iterator(drmBase)) {
            const std::string name = entry.path().filename().string();
            if (name.rfind("card", 0) != 0 ||
                name.size() == 4 ||
                !std::all_of(name.begin() + 4, name.end(), [](unsigned char ch) {
                    return std::isdigit(ch);
                })) {
                continue;
            }

            const fs::path device = entry.path() / "device";
            const fs::path busyPath = device / "gpu_busy_percent";
            if (!fs::exists(busyPath)) {
                continue;
            }

            const int bootVga = toIntFromFile((device / "boot_vga").string());
            gpuDevices.emplace_back(bootVga == 1, busyPath);
        }
    }

    std::stable_sort(gpuDevices.begin(), gpuDevices.end(),
                     [](const auto& left, const auto& right) {
                         return left.first && !right.first;
                     });
    for (const auto& [isBootGpu, busyPath] : gpuDevices) {
        (void)isBootGpu;
        const int value = toIntFromFile(busyPath.string());
        if (value >= 0) {
            return std::min(100, value);
        }
    }

    FILE *fp = popen("nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null", "r");
    if (!fp) {
        return 0;
    }

    char buf[128];
    if (!fgets(buf, sizeof(buf), fp)) {
        pclose(fp);
        return 0;
    }
    pclose(fp);

    try {
        const int value = std::stoi(trim(std::string(buf)));
        return std::max(0, std::min(100, value));
    } catch (...) {
        return 0;
    }
}

static std::vector<std::string> buildCommandArgs(const std::string& command) {
    static const std::map<std::string, std::vector<std::string>> commands = {
        {"VOL_UP", {"pactl", "set-sink-volume", "@DEFAULT_SINK@", "+5%"}},
        {"VOL_DOWN", {"pactl", "set-sink-volume", "@DEFAULT_SINK@", "-5%"}},
        {"BRIGHT_UP", {"bash", "-lc", "brightnessctl set +10% || light -A 10 || true"}},
        {"BRIGHT_DOWN", {"bash", "-lc", "brightnessctl set 10%- || light -U 10 || true"}},
        {"MUTE_MIC", {"pactl", "set-source-mute", "@DEFAULT_SOURCE@", "toggle"}},
        {"MUTE_SPEAKER", {"pactl", "set-sink-mute", "@DEFAULT_SINK@", "toggle"}},
        {"STEAM", {"bash", "-lc", "(steam >/dev/null 2>&1 & || flatpak run com.valvesoftware.Steam >/dev/null 2>&1 & || xdg-open steam://open/main >/dev/null 2>&1 &)"}},
        {"YOUTUBE", {"bash", "-lc", "xdg-open https://www.youtube.com >/dev/null 2>&1 &"}},
        {"SPOTIFY", {"bash", "-lc", "(spotify >/dev/null 2>&1 & || flatpak run com.spotify.Client >/dev/null 2>&1 & || xdg-open https://open.spotify.com >/dev/null 2>&1 &)"}}
    };

    auto it = commands.find(command);
    if (it == commands.end()) {
        return {};
    }
    return it->second;
}

static void runLocalCommand(const std::string& command) {
    std::vector<std::string> args = buildCommandArgs(command);
    if (args.empty()) {
        return;
    }

    pid_t pid = fork();
    if (pid == -1) {
        return;
    }

    if (pid == 0) {
        setsid();

        int devNull = open("/dev/null", O_RDWR);
        if (devNull >= 0) {
            dup2(devNull, STDOUT_FILENO);
            dup2(devNull, STDERR_FILENO);
            close(devNull);
        }

        std::vector<char*> argv;
        for (auto& arg : args) {
            argv.push_back(arg.data());
        }
        argv.push_back(nullptr);

        execvp(argv[0], argv.data());
        _exit(127);
    }
}

static std::string buildPayload(bool include_time) {
    int cpuTemp = readCpuTemp();
    int gpuTemp = readGpuTemp();
    int liquidTemp = 0;
    int cpuFan = readCpuFanRpm(&liquidTemp);
    int gpuFan = readGpuFanRpm();

    int host_gpu_percent = readHostGpuPercent();

    if (cpuTemp > 0) {
        cpuTemp = std::max(0, std::min(100, (int)std::round(cpuTemp / 1000.0)));
    } else {
        cpuTemp = 0;
    }

    if (gpuTemp > 0) {
        gpuTemp = std::max(0, std::min(100, (int)std::round(gpuTemp / 1000.0)));
    } else {
        gpuTemp = 0;
    }

    cpuFan = std::max(0, std::min(20000, cpuFan));
    gpuFan = std::max(0, std::min(20000, gpuFan));
    liquidTemp = std::max(0, std::min(100, liquidTemp));

    // Host metrics: cpu percent, ram used MB, ram total MB, hdd used MB, hdd total MB
    // Compute host CPU percent based on /proc/stat
    static unsigned long long prev_idle = 0, prev_total = 0;
    unsigned long long idle = 0, total = 0;
    {
        std::ifstream in("/proc/stat");
        std::string line;
        if (in && std::getline(in, line)) {
            std::istringstream ss(line);
            std::string cpu_label;
            ss >> cpu_label;
            unsigned long long user=0,nice=0,system=0,irq=0,softirq=0,steal=0,iowait=0,idle_v=0;
            ss >> user >> nice >> system >> idle_v >> iowait >> irq >> softirq >> steal;
            idle = idle_v + iowait;
            total = user + nice + system + idle + irq + softirq + steal;
        }
    }

    int host_cpu_percent = 0;
    if (prev_total != 0 && total > prev_total && total - prev_total > 0) {
        unsigned long long idle_delta = (idle > prev_idle) ? (idle - prev_idle) : 0;
        unsigned long long total_delta = total - prev_total;
        double busy = (double)(total_delta - idle_delta) * 100.0 / (double)total_delta;
        host_cpu_percent = (int)std::round(busy);
        if (host_cpu_percent < 0) host_cpu_percent = 0;
        if (host_cpu_percent > 100) host_cpu_percent = 100;
    }
    prev_idle = idle;
    prev_total = total;

    // RAM: read /proc/meminfo
    int ram_total_mb = 0;
    int ram_available_mb = 0;
    {
        std::ifstream in("/proc/meminfo");
        std::string k;
        unsigned long val = 0;
        while (in >> k >> val) {
            if (k == "MemTotal:") {
                ram_total_mb = (int)(val / 1024);
            } else if (k == "MemAvailable:") {
                ram_available_mb = (int)(val / 1024);
            }
            // skip rest of line
            std::string rest;
            std::getline(in, rest);
        }
    }
    int ram_used_mb = 0;
    if (ram_total_mb > 0) {
        ram_used_mb = ram_total_mb - ram_available_mb;
        if (ram_used_mb < 0) ram_used_mb = 0;
    }
    // Convert RAM to GB (rounded integer)
    int ram_used_gb = 0;
    int ram_total_gb = 0;
    if (ram_total_mb > 0) {
        ram_used_gb = (int)std::round(ram_used_mb / 1024.0);
        ram_total_gb = (int)std::round(ram_total_mb / 1024.0);
    }

    // HDD: use statvfs on '/'
    int hdd_total_mb = 0;
    int hdd_free_mb = 0;
    {
        struct statvfs st{};
        if (statvfs("/", &st) == 0) {
            unsigned long long total_bytes = (unsigned long long)st.f_blocks * st.f_frsize;
            unsigned long long free_bytes = (unsigned long long)st.f_bfree * st.f_frsize;
            hdd_total_mb = (int)(total_bytes / 1024 / 1024);
            hdd_free_mb = (int)(free_bytes / 1024 / 1024);
        }
    }
    int hdd_used_mb = 0;
    if (hdd_total_mb > 0) {
        hdd_used_mb = hdd_total_mb - hdd_free_mb;
        if (hdd_used_mb < 0) hdd_used_mb = 0;
    }
    // Convert HDD to GB (rounded integer)
    int hdd_used_gb = 0;
    int hdd_total_gb = 0;
    if (hdd_total_mb > 0) {
        hdd_used_gb = (int)std::round(hdd_used_mb / 1024.0);
        hdd_total_gb = (int)std::round(hdd_total_mb / 1024.0);
    }

    std::ostringstream oss;
    // Send RAM/HDD in GB (integers)
    oss << cpuTemp << "," << gpuTemp << "," << cpuFan << "," << gpuFan << ",";
    // Include host GPU percent token (always present) so payload layout is stable.
    oss << host_cpu_percent << "," << host_gpu_percent << "," << ram_used_gb << "," << ram_total_gb << "," << hdd_used_gb << "," << hdd_total_gb;
    if (include_time) {
        // append hour and minute as two integer tokens (HH,MM)
        time_t now = time(NULL);
        struct tm tm;
        localtime_r(&now, &tm);
        int hh = tm.tm_hour;
        int mm = tm.tm_min;
        oss << "," << hh << "," << mm;
    }
    oss << "," << liquidTemp;
    oss << "\n";
    return oss.str();
}

static bool shouldSendPayload(const std::string& previousPayload, const std::string& newPayload) {
    return previousPayload.empty() || previousPayload != newPayload;
}

static bool openSerialPort(const std::string& portPath, int baudrate, int& fd) {
    fd = open(portPath.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        return false;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return false;
    }

    speed_t speed = B115200;
    switch (baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default: speed = B115200; break;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return false;
    }

    return true;
}

static std::string readAvailableSerial(int fd) {
    std::string out;
    char buffer[256];
    //int last_minute = -1;
    while (true) {
        ssize_t count = read(fd, buffer, sizeof(buffer));
        if (count <= 0) {
            break;
        }
        out.append(buffer, count);
    }
    return out;
}

static bool writeSerial(int fd, const std::string& data) {
    ssize_t written = 0;
    while (written < static_cast<ssize_t>(data.size())) {
        ssize_t count = write(fd, data.c_str() + written, data.size() - written);
        if (count <= 0) {
            return false;
        }
        written += count;
    }
    return true;
}

int main() {
    int serialFd = -1;
    int last_minute = -1;
    if (!openSerialPort(SERIAL_PORT, BAUDRATE, serialFd)) {
        std::cerr << "Failed to open serial port: " << SERIAL_PORT << std::endl;
        return 1;
    }

    std::string buffer;
    auto lastTx = std::chrono::steady_clock::now();
    std::string lastPayload;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTx);

        if (elapsed.count() >= 500) {
            // only append time tokens when the minute changed (reduce churn)
            time_t tt = time(NULL);
            struct tm tm_now;
            localtime_r(&tt, &tm_now);
            bool include_time = (tm_now.tm_min != last_minute);

            const std::string payload = buildPayload(include_time);
            if (shouldSendPayload(lastPayload, payload)) {
                if (!writeSerial(serialFd, payload)) {
                    std::cerr << "Failed to write payload" << std::endl;
                } else {
                    std::cout << "Sent: " << payload;
                    std::cout.flush();
                    lastPayload = payload;
                    if (include_time) last_minute = tm_now.tm_min;
                }
            }
            lastTx = now;
        }

        std::string incoming = readAvailableSerial(serialFd);
        if (!incoming.empty()) {
            buffer += incoming;
            std::size_t whilePos = 0;
            while ((whilePos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, whilePos);
                buffer.erase(0, whilePos + 1);
                line = trim(line);
                if (line.empty()) {
                    continue;
                }

                std::string command = line;
                std::transform(command.begin(), command.end(), command.begin(), [](unsigned char ch) {
                    return static_cast<char>(std::toupper(ch));
                });

                if (command.rfind("CMD:", 0) == 0) {
                    command = command.substr(4);
                    command = trim(command);
                }

                if (!command.empty()) {
                    std::cout << "Executing: " << command << std::endl;
                    runLocalCommand(command);
                }
            }
        }

        usleep(100000);
    }

    close(serialFd);
    return 0;
}
