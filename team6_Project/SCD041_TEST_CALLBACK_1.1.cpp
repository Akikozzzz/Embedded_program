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


#define SCD41_ADDR 0x62
#define I2C_DEVICE "/dev/i2c-1"

// 全局原子标志用于终止事件循环
std::atomic<bool> g_run_flag{true};

// 信号处理函数 (Ctrl+C 终止)
void signalHandler(int signum) {
    g_run_flag = false;
}
// SCD41Driver 负责与传感器通信和数据采集，采用 timerfd 与 poll 构建单线程事件循环
class SCD41Driver {
public:
    // 回调函数：参数为 CO2（int），温度和湿度（float）
    using SensorCallback = std::function<void(int, float, float)>;

    SCD41Driver(const char* device, uint8_t address)
        : fd_(-1), timer_fd_(-1), callback_(nullptr)
    {
        // 打开 I2C 设备，设置为非阻塞模式
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

        // 创建周期性 timerfd（例如：每 500 毫秒触发一次）
        timer_fd_ = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
        if (timer_fd_ < 0) {
            std::cerr << "Failed to create timerfd: " << strerror(errno) << std::endl;
        } else {
            struct itimerspec newValue;
            newValue.it_interval.tv_sec = 0;
            newValue.it_interval.tv_nsec = 500 * 1000000; // 500 ms
            newValue.it_value = newValue.it_interval;      // 立即启动
            if (timerfd_settime(timer_fd_, 0, &newValue, NULL) < 0) {
                std::cerr << "Failed to set timerfd: " << strerror(errno) << std::endl;
                close(timer_fd_);
                timer_fd_ = -1;
            }
        }
    }

    ~SCD41Driver() {
        stop();
        if (timer_fd_ >= 0) close(timer_fd_);
        if (fd_ >= 0) close(fd_);
    }
  ~SCD41Driver() {
    if (fd_ >= 0) {
        send_command(0x3F86); // 发送停止命令
        usleep(500000);
        close(fd_);           // 直接关闭，不再调用 stop()
    }
    if (timer_fd_ >= 0) close(timer_fd_);
}
    // 注册回调
    void register_callback(SensorCallback callback) {
        callback_ = callback;
    }

    // 启动传感器采集：发送启动测量和校准命令（采用非阻塞方式），后续依靠 timerfd 事件驱动数据采集
    void start() {
        if(fd_ < 0) return;
        send_command(0x21B1); // 启动周期测量
        send_command(0xF4F3); // 强制校准（非阻塞方式，不做等待）
    }

    // 停止操作（在本设计中，事件循环由主线程控制退出）
    void stop() {
        // 无需额外动作
    }

    // 返回 timerfd，用于单线程事件循环 poll 监听
    int getTimerFd() const {
        return timer_fd_;
    }

    // 当 timerfd 触发时调用此函数采集数据
    void handleTimerEvent() {
        if (fd_ < 0) return;

        // 清除定时器触发次数
        uint64_t expirations;
        ssize_t s = read(timer_fd_, &expirations, sizeof(expirations));
        if (s != sizeof(expirations)) {
            std::cerr << "Timer event read error." << std::endl;
            return;
        }

        // 检查传感器数据是否就绪
        if (check_data_ready()) {
            read_and_invoke_callback();
        }
    }

private:
    // 发送 16 位命令到传感器
    void send_command(uint16_t cmd) {
        uint8_t buffer[2] = { ... };
        ssize_t ret = write(fd_, buffer, 2);
        if (ret != 2) {
            std::cerr << "Command write failed: " << strerror(errno) << std::endl;
    }
}

    // 检查传感器 data ready 状态（通过命令 0xE4B8 读取寄存器状态）
    bool check_data_ready() {
        send_command(0xE4B8);
        uint8_t buffer[3];
        ssize_t n = read(fd_, buffer, 3);
        if (n != 3) {
            std::cerr << "Failed to check data ready." << std::endl;
            return false;
        }
        // 当 buffer[1] 的最低 3 位中 bit0 为 1 时，认为数据就绪
        return buffer[1] & 0x07;
    }

    // 发送读取数据命令并解析返回数据，调用回调函数
    void read_and_invoke_callback() {
        uint8_t buffer[9];
        send_command(0xEC05); // 读取测量数据命令

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

private:
    int fd_;         // I2C 设备文件描述符
    int timer_fd_;   // 定时器文件描述符
    SensorCallback callback_;
};

//
// 示例回调函数，用于打印测量数据
//
void sensorCallback(int co2, float temperature, float humidity) {
    std::cout << "CO2: " << co2 << " ppm, Temperature: " << temperature
              << " °C, Humidity: " << humidity << "%" << std::endl;
}

//
// 主函数实现单线程事件循环，通过 poll() 监听 timerfd 事件
//
int main() {
    signal(SIGINT, signalHandler);  // 注册SIGINT信号捕获 (Ctrl+C)

    SCD41Driver sensor(I2C_DEVICE, SCD41_ADDR);
    sensor.register_callback(sensorCallback);
    sensor.start();

    std::cout << "Press Ctrl+C to stop the sensor reading." << std::endl;

    struct pollfd pfd;
    pfd.fd = sensor.getTimerFd();
    pfd.events = POLLIN;

    // 单线程事件循环，有限超时，每次循环检查g_run_flag状态
    while (g_run_flag) {
        int ret = poll(&pfd, 1, 500); // 500ms超时等待
        if (ret < 0) {
            if (errno == EINTR) continue; // 信号中断，重新poll
            std::cerr << "Poll error: " << strerror(errno) << std::endl;
            break;
        } else if (ret == 0) {
            continue; // 超时，无事件，继续检查g_run_flag
        }

        if (pfd.revents & POLLIN) {
            sensor.handleTimerEvent();
        }
    }

    sensor.stop();
    std::cout << "Sensor stopped gracefully." << std::endl;

    return 0;
}
