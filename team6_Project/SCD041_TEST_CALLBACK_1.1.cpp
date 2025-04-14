#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <functional>
#include <thread>
#include <atomic>
#include <chrono>

#define SCD41_ADDR 0x62
#define I2C_DEVICE "/dev/i2c-1"

class SCD41Driver {
public:
    using SensorCallback = std::function<void(int, float, float)>;

    SCD41Driver(const char* device, uint8_t address)
        : fd_(-1), callback_(nullptr), running_(false) {
        fd_ = open(device, O_RDWR);
        if (fd_ < 0) {
            std::cerr << "Cannot open I2C device." << std::endl;
            return;
        }
        if (ioctl(fd_, I2C_SLAVE, address) < 0) {
            std::cerr << "Cannot connect to SCD41 sensor." << std::endl;
            close(fd_);
            fd_ = -1;
        }
    }

    ~SCD41Driver() {
        stop();
        if (fd_ >= 0) close(fd_);
    }

    void register_callback(SensorCallback callback) {
        callback_ = callback;
    }

    void start() {
        send_command(0x21B1); // 启动周期测量
        wait_until_ready();

        send_command(0xF4F3); // 强制校准（非阻塞等待）
        wait_until_ready();

        running_ = true;
        polling_thread_ = std::thread([this]() {
            while (running_) {
                if (check_data_ready()) {
                    read_and_invoke_callback();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 适当的轮询间隔
            }
            });
    }

    void stop() {
        running_ = false;
        if (polling_thread_.joinable())
            polling_thread_.join();
    }

private:
    void send_command(uint16_t cmd) {
        uint8_t buffer[2] = { static_cast<uint8_t>(cmd >> 8), static_cast<uint8_t>(cmd & 0xFF) };
        write(fd_, buffer, 2);
    }

    bool check_data_ready() {
        send_command(0xE4B8); // 获取data ready状态命令
        uint8_t buffer[3];
        if (read(fd_, buffer, 3) != 3) {
            std::cerr << "Failed to check data ready." << std::endl;
            return false;
        }
        return buffer[1] & 0x07; // bit 0 为1时表示数据准备就绪
    }

    void wait_until_ready() {
        constexpr auto timeout = std::chrono::seconds(5);
        auto start = std::chrono::steady_clock::now();
        while (!check_data_ready()) {
            if (std::chrono::steady_clock::now() - start > timeout) {
                std::cerr << "Timeout waiting for sensor ready." << std::endl;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void read_and_invoke_callback() {
        uint8_t buffer[9];
        send_command(0xEC05); // 读取测量数据

        if (read(fd_, buffer, 9) != 9) {
            std::cerr << "Failed to read sensor data." << std::endl;
            return;
        }

        int co2 = (buffer[0] << 8) | buffer[1];
        uint16_t raw_temp = (buffer[3] << 8) | buffer[4];
        float temperature = -45 + 175 * ((float)raw_temp / 65535.0f);
        uint16_t raw_humidity = (buffer[6] << 8) | buffer[7];
        float humidity = 100 * ((float)raw_humidity / 65535.0f);

        if (callback_) callback_(co2, temperature, humidity);
    }

private:
    int fd_;
    std::atomic<bool> running_;
    std::thread polling_thread_;
    SensorCallback callback_;
};

// 示例回调函数
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