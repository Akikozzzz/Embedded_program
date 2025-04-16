#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <functional>
#include <atomic>
#include <chrono>
#include <sys/timerfd.h>
#include <poll.h>
#include <cstring>
#include <errno.h>
#include <csignal>
#include <atomic>
#include <termios.h>
#include <mariadb/mysql.h>

// SCD41 Configuration
#define SCD41_ADDR 0x62
#define I2C_DEVICE "/dev/i2c-1"

// SDS011 Configuration
#define SERIAL_PORT "/dev/ttyUSB0"

// Database connection info
#define DB_HOST "127.20.10.3"
#define DB_USER "team6"
#define DB_PASS "JvTzkQZN"
#define DB_NAME "EnvironmentMonitor"
#define DB_PORT 3306

// Global flag for graceful shutdown
std::atomic<bool> g_run_flag{true};

// Signal handler
void signalHandler(int signum) {
    g_run_flag = false;
}

// SDS011 Driver (unchanged from original)
class SDS011Driver {
public:
    using SensorCallback = std::function<void(float, float)>;

    SDS011Driver(const char* port = SERIAL_PORT)
        : fd_(-1), callback_(nullptr) {
        fd_ = open_serial(port);
        if (fd_ == -1) {
            std::cerr << "Failed to open SDS011 sensor on port: " << port << std::endl;
        }
    }

    ~SDS011Driver() {
        if (fd_ != -1)
            close(fd_);
    }

    void register_callback(SensorCallback callback) {
        callback_ = callback;
    }

    void start() {
        if (fd_ != -1) {
            tcflush(fd_, TCIFLUSH);
        }
    }

    int getSerialFd() const {
        return fd_;
    }

    void process() {
        if (fd_ == -1) return;

        unsigned char buffer[10];
        int n = read(fd_, buffer, sizeof(buffer));
        if (n >= 6 && buffer[0] == 0xAA && buffer[1] == 0xC0) {
            int raw_pm25 = buffer[2] | (buffer[3] << 8);
            int raw_pm10 = buffer[4] | (buffer[5] << 8);
            if (callback_) {
                callback_(raw_pm25 / 10.0f, raw_pm10 / 10.0f);
            }
        }
    }

private:
    int open_serial(const char* device) {
        int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1) {
            perror("Unable to open serial port");
            return -1;
        }

        struct termios options;
        tcgetattr(fd, &options);

        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);

        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag |= CREAD | CLOCAL;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        tcsetattr(fd, TCSANOW, &options);
        return fd;
    }

    int fd_;
    SensorCallback callback_;
};

// SCD41 Driver (unchanged from original)
class SCD41Driver {
public:
    using SensorCallback = std::function<void(int, float, float)>;

    SCD41Driver(const char* device, uint8_t address)
        : fd_(-1), timer_fd_(-1), callback_(nullptr) {
        fd_ = open(device, O_RDWR | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "Cannot open I2C device." << std::endl;
            return;
        }
        if (ioctl(fd_, I2C_SLAVE, address) < 0) {
            std::cerr << "Cannot connect to SCD41 sensor." << std::endl;
            close(fd_);
            fd_ = -1;
        }

        timer_fd_ = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
        if (timer_fd_ < 0) {
            std::cerr << "Failed to create timerfd: " << strerror(errno) << std::endl;
        } else {
            struct itimerspec newValue;
            newValue.it_interval.tv_sec = 0;
            newValue.it_interval.tv_nsec = 500 * 1000000; // 500 ms
            newValue.it_value = newValue.it_interval;
            if (timerfd_settime(timer_fd_, 0, &newValue, NULL) < 0) {
                std::cerr << "Failed to set timerfd: " << strerror(errno) << std::endl;
                close(timer_fd_);
                timer_fd_ = -1;
            }
        }
    }

    ~SCD41Driver() {
        if (fd_ >= 0) {
            send_command(0x3F86);
            close(fd_);
        }
        if (timer_fd_ >= 0) close(timer_fd_);
    }

    void register_callback(SensorCallback callback) {
        callback_ = callback;
    }

    void start() {
        if(fd_ < 0) return;
        send_command(0x21B1);
        send_command(0xF4F3);
    }

    void stop() {
        // No action needed
    }

    int getTimerFd() const {
        return timer_fd_;
    }

    void handleTimerEvent() {
        if (fd_ < 0) return;

        uint64_t expirations;
        ssize_t s = read(timer_fd_, &expirations, sizeof(expirations));
        if (s != sizeof(expirations)) {
            std::cerr << "Timer event read error." << std::endl;
            return;
        }

        if (check_data_ready()) {
            read_and_invoke_callback();
        }
    }

private:
    void send_command(uint16_t cmd) {
        uint8_t buffer[2] = {
            static_cast<uint8_t>(cmd >> 8),
            static_cast<uint8_t>(cmd & 0xFF)
        };

        ssize_t ret = write(fd_, buffer, 2);
        if (ret != 2) {
            std::cerr << "Command write failed: " << strerror(errno) << std::endl;
        }
    }

    bool check_data_ready() {
        send_command(0xE4B8);
        uint8_t buffer[3];
        ssize_t n = read(fd_, buffer, 3);
        if (n != 3) {
            std::cerr << "Failed to check data ready." << std::endl;
            return false;
        }
        return buffer[1] & 0x07;
    }

    void read_and_invoke_callback() {
        uint8_t buffer[9];
        send_command(0xEC05);

        ssize_t n = read(fd_, buffer, 9);
        if (n != 9) {
            std::cerr << "Failed to read sensor data." << std::endl;
            return;
        }
        int co2 = (buffer[0] << 8) | buffer[1];
        uint16_t raw_temp = (buffer[3] << 8) | buffer[4];
        float temperature = -45 + 175 * ((float)raw_temp / 65535.0f);
        uint16_t raw_humidity = (buffer[6] << 8) | buffer[7];
        float humidity = 100 * ((float)raw_humidity / 65535.0f);

        if (callback_)
            callback_(co2, temperature, humidity);
    }

    int fd_;
    int timer_fd_;
    SensorCallback callback_;
};

// 用于存入数据库的函数，不存 pm10 数据
void storeToDatabase(int co2, float temperature, float humidity, float pm25) {
    MYSQL *conn = mysql_init(NULL);
    if (!conn) {
        std::cerr << "mysql_init() failed" << std::endl;
        return;
    }
    // 建立数据库连接
    if (!mysql_real_connect(conn, DB_HOST, DB_USER, DB_PASS, DB_NAME, DB_PORT, NULL, 0)) {
        std::cerr << "mysql_real_connect() failed: " << mysql_error(conn) << std::endl;
        mysql_close(conn);
        return;
    }

    // 构造 SQL 语句，这里假设 sensor_data 表有字段 co2, temperature, humidity, pm25
    char query[512];
    snprintf(query, sizeof(query),
             "INSERT INTO sensor_data (co2, temperature, humidity, pm25) "
             "VALUES (%d, %.2f, %.2f, %.2f)",
             co2, temperature, humidity, pm25);

    if (mysql_query(conn, query)) {
        std::cerr << "INSERT error: " << mysql_error(conn) << std::endl;
    }
    mysql_close(conn);
}

// Combined callback for both sensors
void combinedCallback(int co2, float temperature, float humidity, float pm25, float pm10) {
    std::cout << "=== Sensor Readings ===" << std::endl;
    std::cout << "CO2: " << co2 << " ppm" << std::endl;
    std::cout << "Temperature: " << temperature << " °C" << std::endl;
    std::cout << "Humidity: " << humidity << "%" << std::endl;
    std::cout << "PM2.5: " << pm25 << " ug/m3" << std::endl;
    std::cout << "PM10: " << pm10 << " ug/m3" << std::endl;
    std::cout << "=======================" << std::endl;
    
    storeToDatabase(co2, temperature, humidity, pm25);
}

int main() {
    // Register signal handler
    signal(SIGINT, signalHandler);

    // Initialize sensors
    SCD41Driver scd41(I2C_DEVICE, SCD41_ADDR);
    SDS011Driver sds011;

    // Variables to store sensor data
    struct SensorData {
        int co2 = 0;
        float temperature = 0.0f;
        float humidity = 0.0f;
        float pm25 = 0.0f;
        float pm10 = 0.0f;
    } sensorData;

    // Register individual callbacks
    scd41.register_callback([&sensorData](int co2, float temp, float hum) {
        sensorData.co2 = co2;
        sensorData.temperature = temp;
        sensorData.humidity = hum;
    });

    sds011.register_callback([&sensorData](float pm25, float pm10) {
        sensorData.pm25 = pm25;
        sensorData.pm10 = pm10;
    });

    // Start sensors
    scd41.start();
    sds011.start();

    std::cout << "Sensor reading started. Press Ctrl+C to stop." << std::endl;

    // Create a 1-second interval timer
    int one_sec_timer = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (one_sec_timer < 0) {
        std::cerr << "Failed to create 1-second timer: " << strerror(errno) << std::endl;
        return 1;
    }

    struct itimerspec timer_spec;
    timer_spec.it_interval.tv_sec = 10;  // 1 second interval
    timer_spec.it_interval.tv_nsec = 0;
    timer_spec.it_value = timer_spec.it_interval;

    if (timerfd_settime(one_sec_timer, 0, &timer_spec, NULL) < 0) {
        std::cerr << "Failed to set 1-second timer: " << strerror(errno) << std::endl;
        close(one_sec_timer);
        return 1;
    }

    // Set up poll for both sensors and the 1-second timer
    struct pollfd pfds[3];
    pfds[0].fd = scd41.getTimerFd();
    pfds[0].events = POLLIN;
    pfds[1].fd = sds011.getSerialFd();
    pfds[1].events = POLLIN;
    pfds[2].fd = one_sec_timer;
    pfds[2].events = POLLIN;

    // Main event loop
    while (g_run_flag) {
        int ret = poll(pfds, 3, 10000);  // 1 second timeout
        if (ret < 0) {
            if (errno == EINTR) continue;
            std::cerr << "Poll error: " << strerror(errno) << std::endl;
            break;
        }

        if (ret > 0) {
            // Handle SCD41 events
            if (pfds[0].revents & POLLIN) {
                scd41.handleTimerEvent();
            }

            // Handle SDS011 events
            if (pfds[1].revents & POLLIN) {
                sds011.process();
            }

            // Handle 1-second timer event
            if (pfds[2].revents & POLLIN) {
                uint64_t expirations;
                read(one_sec_timer, &expirations, sizeof(expirations));
                // Print combined data every second
                combinedCallback(sensorData.co2, sensorData.temperature, 
                                sensorData.humidity, sensorData.pm25, sensorData.pm10);
            }
        }
    }

    // Cleanup
    close(one_sec_timer);
    std::cout << "Sensor reading stopped." << std::endl;
    return 0;
}
