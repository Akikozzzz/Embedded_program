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
    // �򿪴��ڲ�����Ϊ������ģʽ
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }

    // ���ô�������
    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag &= ~PARENB;  // ����żУ��
    options.c_cflag &= ~CSTOPB;  // 1��ֹͣλ
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8������λ

    options.c_cflag |= CREAD | CLOCAL; // ʹ�ܶ�ȡ�����Ե��ƽ����������·
    options.c_iflag &= ~(IXON | IXOFF | IXANY);  // ��������
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // ����Ϊԭʼģʽ
    options.c_oflag &= ~OPOST; // ԭʼ���

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

int main() {
    // �򿪴����豸
    int fd = open_serial(SERIAL_PORT);
    if (fd == -1)
        return 1;
    cout << "connect success, reading data now..." << endl;

    // ʹ�� poll ���������ļ��������������¼�
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;

    unsigned char buffer[10];
    while (true) {
        // �����ȴ��¼���������ʹ�ö�����ʱ����
        int ret = poll(&pfd, 1, -1);  // ���޵ȴ�
        if (ret < 0) {
            cerr << "poll error: " << strerror(errno) << endl;
            break;
        }
        if (pfd.revents & POLLIN) {
            // �������ݵ���ʱ��ȡ����
            int n = read(fd, buffer, sizeof(buffer));
            if (n > 0) {
                // SDS011 ���������ݸ�ʽ����ʼ�ֽ�0xAA, 0xC0
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