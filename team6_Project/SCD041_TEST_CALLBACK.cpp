#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <functional>
#include <thread>

#define SCD41_ADDR 0x62
#define I2C_DEVICE "/dev/i2c-1"

class SCD41Driver {
public:
    using SensorCallback = std::function<void(int, float, float)>;

    SCD41Driver(const char* device, uint8_t address)
        : fd_(-1), callback_(nullptr) {
        fd_ = open(device, O_RDWR);
        if (fd_ < 0) {
            std::cerr << "Cannot open I2C device." << std::endl;
        }
        else if (ioctl(fd_, I2C_SLAVE, address) < 0) {
            std::cerr << "Cannot connect to SCD41 sensor." << std::endl;
            close(fd_);
            fd_ = -1;
        }
    }

    ~SCD41Driver() {
        if (fd_ >= 0) close(fd_);
    }

    void register_callback(SensorCallback callback) {
        callback_ = callback;
    }

    void start() {
        send_command(0x21B1);  // Start periodic measurement
        std::this_thread::sleep_for(std::chrono::seconds(5));
        send_command(0xF4F3);  // Perform forced recalibration
        std::this_thread::sleep_for(std::chrono::seconds(5));

        running_ = true;
        polling_thread_ = std::thread([this]() {
            while (running_) {
                read_and_invoke_callback();
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            });
    }

    void stop() {
        running_ = false;
        if (polling_thread_.joinable()) polling_thread_.join();
    }

private:
    void send_command(uint16_t cmd) {
        uint8_t buffer[2] = { static_cast<uint8_t>(cmd >> 8), static_cast<uint8_t>(cmd & 0xFF) };
        write(fd_, buffer, 2);
    }

    void read_and_invoke_callback() {
        uint8_t buffer[9];
        send_command(0xEC05);
        usleep(50000);

        if (read(fd_, buffer, 9) != 9) {
            std::cerr << "Failed to read sensor data." << std::endl;
            return;
        }

        int co2 = (buffer[0] << 8) | buffer[1];
        uint16_t raw_temp = (buffer[3] << 8) | buffer[4];
        float temperature = -45 + 175 * ((float)raw_temp / 65535.0);
        uint16_t raw_humidity = (buffer[6] << 8) | buffer[7];
        float humidity = 100 * ((float)raw_humidity / 65535.0);

        if (callback_) callback_(co2, temperature, humidity);
    }

private:
    int fd_;
    bool running_;
    std::thread polling_thread_;
    SensorCallback callback_;
};

// 回调函数示例
void sensorCallback(int co2, float temperature, float humidity) {
    std::cout << "CO2: " << co2 << " ppm, Temperature: " << temperature
        << " °C, Humidity: " << humidity << "%" << std::endl;
}

int main() {
    SCD41Driver sensor(I2C_DEVICE, SCD41_ADDR);
    sensor.register_callback(sensorCallback);
    sensor.start();

    std::cout << "Press Ctrl+C to stop the sensor reading." << std::endl;
    pause();

    sensor.stop();

    return 0;
}
