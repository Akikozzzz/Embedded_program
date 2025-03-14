#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

#define SERIAL_PORT "/dev/ttyUSB0"  

using namespace std;

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

void read_pm25_data(int fd) {
    unsigned char buffer[10];
    while (true) {
        int n = read(fd, buffer, sizeof(buffer));
        if (n > 0) {
            if (buffer[0] == 0xAA && buffer[1] == 0xC0) {
                int pm25 = buffer[2] | (buffer[3] << 8);
                int pm10 = buffer[4] | (buffer[5] << 8);
                cout << "PM2.5: " << pm25 / 10.0 << " ug/m3, PM10: " << pm10 / 10.0 << " ug/m3" << endl;
            }
        }
        usleep(1000000); 
    }
}

int main() {
    int fd = open_serial(SERIAL_PORT);
    if (fd == -1) return 1;
    cout << "connect success, reading data now..." << endl;
    
    read_pm25_data(fd);
    
    close(fd);
    return 0;
}
