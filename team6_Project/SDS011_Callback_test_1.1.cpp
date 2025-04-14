#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <poll.h>
#include <errno.h>

#define SERIAL_PORT "/dev/ttyUSB0"

using namespace std;

int open_serial(const char* device) {
    // 打开串口并设置为非阻塞模式
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }

    // 配置串口属性
    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag &= ~PARENB;  // 无奇偶校验
    options.c_cflag &= ~CSTOPB;  // 1个停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8个数据位

    options.c_cflag |= CREAD | CLOCAL; // 使能读取，忽略调制解调器控制线路
    options.c_iflag &= ~(IXON | IXOFF | IXANY);  // 禁用流控
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 设置为原始模式
    options.c_oflag &= ~OPOST; // 原始输出

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

int main() {
    // 打开串口设备
    int fd = open_serial(SERIAL_PORT);
    if (fd == -1)
        return 1;
    cout << "connect success, reading data now..." << endl;

    // 使用 poll 监听串口文件描述符的输入事件
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;

    unsigned char buffer[10];
    while (true) {
        // 阻塞等待事件到来，不使用额外延时函数
        int ret = poll(&pfd, 1, -1);  // 无限等待
        if (ret < 0) {
            cerr << "poll error: " << strerror(errno) << endl;
            break;
        }
        if (pfd.revents & POLLIN) {
            // 当有数据到达时读取数据
            int n = read(fd, buffer, sizeof(buffer));
            if (n > 0) {
                // SDS011 传感器数据格式：起始字节0xAA, 0xC0
                if (buffer[0] == 0xAA && buffer[1] == 0xC0) {
                    int pm25 = buffer[2] | (buffer[3] << 8);
                    int pm10 = buffer[4] | (buffer[5] << 8);
                    cout << "PM2.5: " << pm25 / 10.0 << " ug/m3, PM10: " << pm10 / 10.0 << " ug/m3" << endl;
                }
            }
        }
    }

    close(fd);
    return 0;
}