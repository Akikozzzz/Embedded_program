#include <iostream>
#include <functional>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/timerfd.h>
#include <linux/i2c-dev.h>
#include <poll.h>
#include <cstring>
#include <atomic>
#include <errno.h>

#define ADS1115_ADDRESS 0x48
#define CONVERSION_REG 0x00
#define CONFIG_REG 0x01
#define I2C_DEV "/dev/i2c-2"

class PHSensorReader {
public:
    using Callback = std::function<void(double voltage, double pH)>;

    PHSensorReader(const std::string& device, int channel = 0)
        : device_path_(device), channel_(channel), fd_(-1), timer_fd_(-1), running_(false) {
    }

    bool init() {
        fd_ = open(device_path_.c_str(), O_RDWR | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "Failed to open I2C device\n";
            return false;
        }

        if (ioctl(fd_, I2C_SLAVE, ADS1115_ADDRESS) < 0) {
            std::cerr << "Failed to communicate with ADS1115\n";
            close(fd_);
            return false;
        }

        timer_fd_ = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
        if (timer_fd_ < 0) {
            std::cerr << "Failed to create timerfd\n";
            close(fd_);
            return false;
        }

        struct itimerspec interval;
        interval.it_interval.tv_sec = 1;
        interval.it_interval.tv_nsec = 0;
        interval.it_value = interval.it_interval;

        if (timerfd_settime(timer_fd_, 0, &interval, nullptr) < 0) {
            std::cerr << "Failed to set timerfd\n";
            close(fd_);
            close(timer_fd_);
            return false;
        }

        return true;
    }

    void registerCallback(Callback cb) {
        callback_ = cb;
    }

    void run() {
        if (fd_ < 0 || timer_fd_ < 0) return;

        running_ = true;
        struct pollfd pfd;
        pfd.fd = timer_fd_;
        pfd.events = POLLIN;

        while (running_) {
            int ret = poll(&pfd, 1, -1);
            if (ret < 0) {
                if (errno == EINTR) continue;
                std::cerr << "Poll error: " << strerror(errno) << std::endl;
                break;
            }

            if (pfd.revents & POLLIN) {
                uint64_t expirations;
                read(timer_fd_, &expirations, sizeof(expirations));

                double voltage = 0.0;
                if (tryReadADC(voltage)) {
                    double pH = voltageToPH(voltage);
                    if (callback_) callback_(voltage, pH);
                }
            }
        }
    }

    void stop() {
        running_ = false;
    }

    ~PHSensorReader() {
        if (timer_fd_ >= 0) close(timer_fd_);
        if (fd_ >= 0) close(fd_);
    }

private:
    bool tryReadADC(double& voltage_out) {
        uint8_t config[3] = { CONFIG_REG, 0xC3, 0x83 }; // A0 ???
        if (channel_ == 1) config[1] = 0xD3;

        if (write(fd_, config, 3) != 3) return false;
        uint8_t reg = CONVERSION_REG;
        if (write(fd_, &reg, 1) != 1) return false;

        uint8_t data[2];
        ssize_t n = read(fd_, data, 2);
        if (n != 2) return false;

        int16_t raw_adc = (data[0] << 8) | data[1];
        voltage_out = raw_adc * 4.096 / 32767.0;
        return true;
    }

    double voltageToPH(double voltage) {
        double pH7_voltage = 2.5;
        double slope = -0.18;
        return 7.0 + (voltage - pH7_voltage) / slope;
    }

private:
    std::string device_path_;
    int channel_;
    int fd_;
    int timer_fd_;
    std::atomic<bool> running_;
    Callback callback_;
};

int main() {
    PHSensorReader sensor(I2C_DEV);

    if (!sensor.init()) {
        return 1;
    }

    sensor.registerCallback([](double voltage, double pH) {
        std::cout << "Voltage: " << voltage << " V, pH: " << pH << std::endl;
        });

    sensor.run();  // ??????
    return 0;
}
