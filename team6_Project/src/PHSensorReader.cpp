#include "PHSensorReader.h"

PHSensorReader::PHSensorReader(const std::string& device, int channel)
    : device_path_(device), channel_(channel), fd_(-1), timer_fd_(-1) {}

PHSensorReader::~PHSensorReader() {
    if (timer_fd_ >= 0) close(timer_fd_);
    if (fd_ >= 0) close(fd_);
}

bool PHSensorReader::init() {
    fd_ = open(device_path_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) return false;
    if (ioctl(fd_, I2C_SLAVE, ADS1115_ADDRESS) < 0) return false;

    timer_fd_ = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (timer_fd_ < 0) return false;

    struct itimerspec interval = {{1, 0}, {1, 0}};
    return timerfd_settime(timer_fd_, 0, &interval, nullptr) == 0;
}

void PHSensorReader::registerCallback(Callback cb) {
    callback_ = cb;
}

void PHSensorReader::handleTimerEvent() {
    uint64_t expirations;
    read(timer_fd_, &expirations, sizeof(expirations));
    double voltage = 0.0;
    if (tryReadADC(voltage)) {
        double pH = voltageToPH(voltage);
        if (callback_) callback_(voltage, pH);
    }
}

int PHSensorReader::getTimerFd() const {
    return timer_fd_;
}

bool PHSensorReader::tryReadADC(double& voltage_out) {
    uint8_t config[3] = { CONFIG_REG, 0xC3, 0x83 };
    if (channel_ == 1) config[1] = 0xD3;
    if (write(fd_, config, 3) != 3) return false;
    uint8_t reg = CONVERSION_REG;
    if (write(fd_, &reg, 1) != 1) return false;
    uint8_t data[2];
    if (read(fd_, data, 2) != 2) return false;
    int16_t raw_adc = (data[0] << 8) | data[1];
    voltage_out = raw_adc * 4.096 / 32767.0;
    return true;
}

double PHSensorReader::voltageToPH(double voltage) {
    double pH7_voltage = 2.5;
    double slope = -0.18;
    return 7.0 + (voltage - pH7_voltage) / slope;
}
