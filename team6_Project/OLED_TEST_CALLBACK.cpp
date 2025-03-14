#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <functional>
#include <thread>

#define I2C_ADDR 0x3C        // OLED Ä¬ÈÏ I2C µØÖ·
#define I2C_DEVICE "/dev/i2c-3"

class OLEDDriver {
public:
    using CallbackFunction = std::function<void()>;

    OLEDDriver(const char* device, uint8_t address)
        : address_(address), fd_(-1), callback_(nullptr) {
        fd_ = open(device, O_RDWR);
        if (fd_ < 0) {
            std::cerr << "Failed to open the I2C bus." << std::endl;
        }
        else if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
            std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
            close(fd_);
            fd_ = -1;
        }
    }

    ~OLEDDriver() {
        if (fd_ >= 0) close(fd_);
    }

    void register_callback(CallbackFunction callback) {
        callback_ = callback;
    }

    void init() {
        sendCommand(0xAE);
        sendCommand(0xD5);
        sendCommand(0x80);
        sendCommand(0xA8);
        sendCommand(0x3F);
        sendCommand(0xD3);
        sendCommand(0x00);
        sendCommand(0x40);
        sendCommand(0x8D);
        sendCommand(0x14);
        sendCommand(0x20);
        sendCommand(0x00);
        sendCommand(0xA1);
        sendCommand(0xC8);
        sendCommand(0xDA);
        sendCommand(0x12);
        sendCommand(0x81);
        sendCommand(0xCF);
        sendCommand(0xD9);
        sendCommand(0xF1);
        sendCommand(0xDB);
        sendCommand(0x40);
        sendCommand(0xA4);
        sendCommand(0xA6);
        sendCommand(0xAF);

        if (callback_) callback_();
    }

    void fillScreen(uint8_t data) {
        for (int i = 0; i < 1024; i++) {
            sendData(data);
        }
    }

private:
    void sendCommand(uint8_t cmd) {
        uint8_t buffer[2] = { 0x00, cmd };
        if (write(fd_, buffer, 2) != 2) {
            std::cerr << "Failed to write command." << std::endl;
        }
    }

    void sendData(uint8_t data) {
        uint8_t buffer[2] = { 0x40, data };
        if (write(fd_, buffer, 2) != 2) {
            std::cerr << "Failed to write data." << std::endl;
        }
    }

private:
    uint8_t address_;
    int fd_;
    CallbackFunction callback_;
};

void oledInitializedCallback() {
    std::cout << "OLED initialization complete, callback invoked!" << std::endl;
}

int main() {
    OLEDDriver oled(I2C_DEVICE, I2C_ADDR);
    oled.register_callback(oledInitializedCallback);
    oled.init();

    oled.fillScreen(0xFF);

    std::cout << "OLED Test Completed!" << std::endl;

    return 0;
}
