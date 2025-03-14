#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>

#define SCD41_ADDR 0x62  
#define I2C_DEVICE "/dev/i2c-1"

void send_command(int file, uint16_t cmd) {
    uint8_t buffer[2];
    buffer[0] = cmd >> 8;
    buffer[1] = cmd & 0xFF;
    write(file, buffer, 2);
}

// Read CO2, temperature, humidity, and raw humidity
void read_scd41_data(int file, int &co2, float &temperature, float &humidity, uint16_t &raw_humidity) {
    uint8_t buffer[9];
    send_command(file, 0xEC05);
    usleep(50000);  // Wait for data

    if (read(file, buffer, 9) != 9) {
        std::cerr << "Failed to read SCD41 sensor" << std::endl;
        return;
    }

    for (int i = 0; i < 9; i++) {
        printf("%02X ", buffer[i]);
    }
    std::cout << std::endl;

    co2 = (buffer[0] << 8) | buffer[1];

    uint16_t raw_temp = (buffer[3] << 8) | buffer[4];
    temperature = -45 + 175 * ((float)raw_temp / 65535.0);

    raw_humidity = (buffer[6] << 8) | buffer[7];  // Store raw humidity
    humidity = 100 * ((float)raw_humidity / 65535.0);
    printf("%f",humidity);
    
}

int main() {
    int file = open(I2C_DEVICE, O_RDWR);
    if (file < 0) {
        std::cerr << "Cannot open I2C device" << std::endl;
        return 1;
    }

    if (ioctl(file, I2C_SLAVE, SCD41_ADDR) < 0) {
        std::cerr << "Cannot connect to SCD41" << std::endl;
        return 1;
    }

    send_command(file, 0x21B1);
    std::cout << "SCD41 sensor initialized" << std::endl;
    sleep(5);
    send_command(file, 0xF4F3);
    sleep(5);

    while (true) {
        int co2;
        float temperature, humidity;
        uint16_t raw_humidity;  // Make sure this is declared

        read_scd41_data(file, co2, temperature, humidity, raw_humidity);

        std::cout << "CO2: " << co2 << " ppm, Temperature: " << temperature << "C, "
                  << "Humidity: " << humidity << "%, Raw Humidity: " << raw_humidity << std::endl;

        sleep(2);
    }

    close(file);
    return 0;
}


