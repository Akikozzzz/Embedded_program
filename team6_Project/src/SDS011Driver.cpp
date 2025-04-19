#include "SDS011Driver.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

SDS011Driver::SDS011Driver(const char* serial_path)
    : serial_fd_(-1), callback_(nullptr) {
    serial_fd_ = open(serial_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        std::cerr << "Failed to open SDS011 serial port." << std::endl;
    } else {
        configure_serial();
    }
}

SDS011Driver::~SDS011Driver() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
    }
}

void SDS011Driver::configure_serial() {
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        std::cerr << "Error getting serial attributes." << std::endl;
        return;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8 data bits
    tty.c_cflag &= ~PARENB;   // no parity
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;  // no flow control
    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;

    tcflush(serial_fd_, TCIFLUSH);
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial attributes." << std::endl;
    }
}

void SDS011Driver::register_callback(SensorCallback callback) {
    callback_ = callback;
}

void SDS011Driver::start() {
    // SDS011 auto-sends data, no start command required for passive mode
}

void SDS011Driver::stop() {
    // Optionally implement command to stop sensor
}

int SDS011Driver::getSerialFd() const {
    return serial_fd_;
}

void SDS011Driver::process() {
    if (serial_fd_ < 0) return;

    uint8_t buffer[10];
    ssize_t n = read(serial_fd_, buffer, sizeof(buffer));
    if (n != 10) return;

    // Check SDS011 packet header
    if (buffer[0] != 0xAA || buffer[1] != 0xC0 || buffer[9] != 0xAB) return;

    uint16_t pm25_raw = buffer[2] | (buffer[3] << 8);
    uint16_t pm10_raw = buffer[4] | (buffer[5] << 8);

    float pm25 = pm25_raw / 10.0f;
    float pm10 = pm10_raw / 10.0f;

    if (callback_) {
        callback_(pm25, pm10);
    }
}
