#include <iostream>
#include <functional>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <poll.h>
#include <csignal>
#include <atomic>
#include <errno.h>

#define SERIAL_PORT "/dev/ttyUSB0"

using namespace std;

std::atomic<bool> run_flag{true};  // 控制主循环退出

// Ctrl+C 信号处理器
void signalHandler(int) {
    run_flag = false;
}

// 打开并配置串口
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

// 定义回调函数类型
using DataCallback = std::function<void(float pm25, float pm10)>;

// 读取并解析传感器数据
void read_sensor_data(int fd, DataCallback callback) {
    unsigned char buffer[10];
    int n = read(fd, buffer, sizeof(buffer));
    if (n >= 6 && buffer[0] == 0xAA && buffer[1] == 0xC0) {
        int pm25 = buffer[2] | (buffer[3] << 8);
        int pm10 = buffer[4] | (buffer[5] << 8);
        if (callback) {
            callback(pm25 / 10.0f, pm10 / 10.0f);  // ✅ 触发回调
        }
    }
}

// 回调函数：处理数据
void sensorCallback(float pm25, float pm10) {
    cout << "PM2.5: " << pm25 << " ug/m3, PM10: " << pm10 << " ug/m3" << endl;
}

int main() {
    signal(SIGINT, signalHandler);  // 注册 Ctrl+C 处理器

    int fd = open_serial(SERIAL_PORT);
    if (fd == -1) return 1;

    cout << "Connect success, reading data now..." << endl;

    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;

    while (run_flag) {
        int ret = poll(&pfd, 1, 500);  // 500ms 超时
        if (ret < 0) {
            if (errno == EINTR) continue; // 信号打断继续
            cerr << "poll error: " << strerror(errno) << endl;
            break;
        }

        if (ret > 0 && (pfd.revents & POLLIN)) {
            read_sensor_data(fd, sensorCallback);
        }
    }

    cout << "Stopped." << endl;
    close(fd);
    return 0;
}
