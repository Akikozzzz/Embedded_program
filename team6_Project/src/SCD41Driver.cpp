#include "SCD41Driver.h"

SCD41Driver::SCD41Driver(const char* device, uint8_t address)
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
        newValue.it_interval.tv_nsec = 500 * 1000000; // 500ms
        newValue.it_value = newValue.it_interval;

        if (timerfd_settime(timer_fd_, 0, &newValue, nullptr) < 0) {
            std::cerr << "Failed to set timerfd: " << strerror(errno) << std::endl;
            close(timer_fd_);
            timer_fd_ = -1;
        }
    }
}

SCD41Driver::~SCD41Driver() {
    if (fd_ >= 0) {
        send_command(0x3F86);  // Stop periodic measurement
        close(fd_);
    }
    if (timer_fd_ >= 0) close(timer_fd_);
}

void SCD41Driver::register_callback(SensorCallback callback) {
    callback_ = callback;
}

void SCD41Driver::start() {
    if (fd_ < 0) return;

    send_command(0x21B1); // reinit
    send_command(0xF4F3); // start periodic measurement
}

void SCD41Driver::stop() {
    send_command(0x3F86); // stop periodic measurement
}

int SCD41Driver::getTimerFd() const {
    return timer_fd_;
}

void SCD41Driver::handleTimerEvent() {
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

void SCD41Driver::send_command(uint16_t cmd) {
    uint8_t buffer[2] = {
        static_cast<uint8_t>(cmd >> 8),
        static_cast<uint8_t>(cmd & 0xFF)
    };

    ssize_t ret = write(fd_, buffer, 2);
    if (ret != 2) {
        std::cerr << "Command write failed: " << strerror(errno) << std::endl;
    }
}

bool SCD41Driver::check_data_ready() {
    send_command(0xE4B8);  // read data ready status
    uint8_t buffer[3];
    ssize_t n = read(fd_, buffer, 3);
    if (n != 3) {
        std::cerr << "Failed to check data ready." << std::endl;
        return false;
    }

    return buffer[1] & 0x07;
}

void SCD41Driver::read_and_invoke_callback() {
    uint8_t buffer[9];
    send_command(0xEC05); // read measurement

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

    if (callback_) {
        callback_(co2, temperature, humidity);
    }
}
