#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <mariadb/mysql.h>
#include <cstring>

#define SERIAL_PORT "/dev/ttyUSB0"  // SDS011 sensor
#define SCD41_ADDR 0x62            // SCD41 I2C address
#define I2C_DEVICE "/dev/i2c-1"

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
    humidity = 100 * ((float)raw_humidity / 65535.0) - 40;
    return true;
}

// Connect to MariaDB and insert data
void insert_to_db(float pm25, float co2, float temperature, float humidity) {
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
        std::cout << "Data successfully stored in database" << std::endl;
    }

    mysql_close(conn);
}

int main() {
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

        // Read PM2.5 data
        if (read_pm25_data(fd_serial, pm25)) {
            std::cout << "PM2.5: " << pm25 << " ug/m3" << std::endl;
        } else {
            std::cerr << "SDS011 reading failed" << std::endl;
        }

        // Read CO2, temperature, and humidity
        if (read_scd41_data(fd_i2c, co2, temperature, humidity)) {
            std::cout << "CO2: " << co2 << " ppm, Temperature: " << temperature << "C, Humidity: " << humidity << "%" << std::endl;
        }

        // Insert data into database
        insert_to_db(pm25, co2, temperature, humidity);

        // Wait for 10 seconds before the next reading
        sleep(10);
    }

    close(fd_serial);
    close(fd_i2c);
    return 0;
}
