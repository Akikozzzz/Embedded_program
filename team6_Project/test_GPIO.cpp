#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cmath>
#include <mariadb/mysql.h>
#include <cstring>
#include <gpiod.h>
#include <linux/i2c-dev.h>

#define SERIAL_PORT "/dev/ttyUSB0"   // SDS011 sensor
#define SCD41_ADDR 0x62              // SCD41 I2C address
#define I2C_DEVICE "/dev/i2c-1"

// GPIO 定义
#define GPIO_CHIP "gpiochip0"
#define GPIO_PM25 17
#define GPIO_CO2  27
#define GPIO_DB   22

// GPIO 初始化
gpiod_line* init_gpio(gpiod_chip* chip, int gpio_num) {
    gpiod_line* line = gpiod_chip_get_line(chip, gpio_num);
    if (!line || gpiod_line_request_output(line, "gpio_control", 0) < 0) {
        std::cerr << "Failed to initialize GPIO " << gpio_num << std::endl;
        return nullptr;
    }
    return line;
}

// 控制 GPIO
void set_gpio(gpiod_line* line, int value) {
    gpiod_line_set_value(line, value);
}

// Open serial port for SDS011
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

// Read PM2.5 from SDS011
bool read_pm25_data(int fd, float &pm25) {
    unsigned char buffer[10];
    int n = read(fd, buffer, sizeof(buffer));
    if (n > 0 && buffer[0] == 0xAA && buffer[1] == 0xC0) {
        pm25 = (buffer[2] | (buffer[3] << 8)) / 10.0;
        return true;
    }
    return false;
}

// Send I2C command to SCD41
void send_command(int file, uint16_t cmd) {
    uint8_t buffer[2];
    buffer[0] = cmd >> 8;
    buffer[1] = cmd & 0xFF;
    write(file, buffer, 2);
}

// Read CO2, temperature, and humidity from SCD41
bool read_scd41_data(int file, int &co2, float &temperature, float &humidity) {
    uint8_t buffer[9];
    send_command(file, 0xEC05);  
    usleep(50000);               

    if (read(file, buffer, 9) != 9) {
        std::cerr << "failed" << std::endl;
        return false;
    }

    co2 = (buffer[0] << 8) | buffer[1];
    uint16_t raw_temp = (buffer[3] << 8) | buffer[4];
    temperature = -45 + 175 * ((float)raw_temp / 65535.0);
    uint16_t raw_humidity = (buffer[6] << 8) | buffer[7];
    humidity = 100 * ((float)raw_humidity / 65535.0);
    return true;
}

// Connect to MariaDB and insert data
void insert_to_db(float pm25, float co2, float temperature, float humidity, gpiod_line* gpio_db) {
    MYSQL *conn;
    conn = mysql_init(NULL);
    if (!mysql_real_connect(conn, "localhost", "root", "your_password", "EnvironmentMonitor", 0, NULL, 0)) {
        std::cerr << "Database connection failed: " << mysql_error(conn) << std::endl;
        return;
    }

    char query[512];
    snprintf(query, sizeof(query), 
             "INSERT INTO sensor_data (pm25, co2, temperature, humidity) VALUES (%.2f, %.2f, %.2f, %.2f);",
             pm25, co2, temperature, humidity);

    if (mysql_query(conn, query)) {
        std::cerr << "Failed to insert data: " << mysql_error(conn) << std::endl;
    } else {
        set_gpio(gpio_db, 1);  // 点亮 GPIO 表示数据已插入
        sleep(1);
        set_gpio(gpio_db, 0);  // 熄灭指示灯
        std::cout << "Data successfully stored in database" << std::endl;
    }

    mysql_close(conn);
}

int main() {
    // 初始化 GPIO
    gpiod_chip* chip = gpiod_chip_open_by_name(GPIO_CHIP);
    gpiod_line* gpio_pm25 = init_gpio(chip, GPIO_PM25);
    gpiod_line* gpio_co2 = init_gpio(chip, GPIO_CO2);
    gpiod_line* gpio_db = init_gpio(chip, GPIO_DB);

    // Open SDS011 sensor serial port
    int fd_serial = open_serial(SERIAL_PORT);
    if (fd_serial == -1) return 1;

    // Open SCD41 sensor I2C connection
    int fd_i2c = open(I2C_DEVICE, O_RDWR);
    if (fd_i2c < 0) {
        std::cerr << "Unable to open I2C device" << std::endl;
        return 1;
    }

    if (ioctl(fd_i2c, I2C_SLAVE, SCD41_ADDR) < 0) {
        std::cerr << "Unable to connect to SCD41" << std::endl;
        return 1;
    }

    send_command(fd_i2c, 0x21B1);
    std::cout << "Sensors initialized, waiting for 10 seconds..." << std::endl;
    sleep(10);

    while (true) {
        float pm25 = -1;
        int co2 = -1;
        float temperature = -1, humidity = -1;

        set_gpio(gpio_pm25, 1);
        if (read_pm25_data(fd_serial, pm25)) {
            std::cout << "PM2.5: " << pm25 << " ug/m3" << std::endl;
        }
        set_gpio(gpio_pm25, 0);

        set_gpio(gpio_co2, 1);
        if (read_scd41_data(fd_i2c, co2, temperature, humidity)) {
            std::cout << "CO2: " << co2 << " ppm, Temperature: " << temperature << "C, Humidity: " << humidity << "%" << std::endl;
        }
        set_gpio(gpio_co2, 0);

        insert_to_db(pm25, co2, temperature, humidity, gpio_db);

        sleep(10);
    }

    gpiod_chip_close(chip);
    close(fd_serial);
    close(fd_i2c);
    return 0;
}
