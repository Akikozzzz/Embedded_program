// OLEDDriver.cpp
#include "OLEDDriver.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <cstring>

OLEDDriver::OLEDDriver(const char* device, uint8_t address)
    : address_(address), fd_(-1) {
    fd_ = open(device, O_RDWR);
    if (fd_ < 0) {
        std::cerr << "Failed to open I2C device: " << device << std::endl;
    } else if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        std::cerr << "Failed to set I2C address: " << int(address_) << std::endl;
        close(fd_);
        fd_ = -1;
    }
}

OLEDDriver::~OLEDDriver() {
    if (fd_ >= 0) {
        close(fd_);
    }
}

void OLEDDriver::sendCommand(uint8_t cmd) {
    uint8_t buffer[2] = {0x00, cmd};
    write(fd_, buffer, 2);
}

void OLEDDriver::sendData(uint8_t data) {
    uint8_t buffer[2] = {0x40, data};
    write(fd_, buffer, 2);
}

void OLEDDriver::init() {
    const uint8_t cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00,
        0x40, 0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8,
        0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB,
        0x40, 0xA4, 0xA6, 0xAF
    };
    for (uint8_t c : cmds) {
        sendCommand(c);
    }
    clear();
}

void OLEDDriver::clear() {
    for (int page = 0; page < 8; ++page) {
        sendCommand(0xB0 + page);     // Set page address
        sendCommand(0x00);            // Set lower column start address
        sendCommand(0x10);            // Set higher column start address
        for (int i = 0; i < 128; ++i) {
            sendData(0x00);
        }
    }
}

void OLEDDriver::drawText(const char* text, int x, int y) {
    int page = y / 8;
    sendCommand(0xB0 + page);
    sendCommand(0x00 + (x & 0x0F));
    sendCommand(0x10 + ((x >> 4) & 0x0F));
    while (*text) {
        if (*text < 32 || *text > 127) {
            ++text;
            continue;
        }
        const uint8_t* chr = font5x8[*text - 32];
        for (int i = 0; i < 5; ++i) {
            sendData(chr[i]);
        }
        sendData(0x00);  
        ++text;
    }
}
