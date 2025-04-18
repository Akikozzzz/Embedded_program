#include <iostream>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define ADS1115_ADDRESS 0x48
#define CONVERSION_REG  0x00
#define CONFIG_REG      0x01

double readADC(int file, int channel) {
    uint8_t config[3] = {CONFIG_REG, 0xC3, 0x83}; 
    if (channel == 1) config[1] = 0xD3; 
    else if (channel == 2) config[1] = 0xE3; 
    else if (channel == 3) config[1] = 0xF3; 

    if (write(file, config, 3) != 3) {
        std::cerr << "Failed to configure ADS1115\n";
        return -1;
    }

    usleep(8000);
    uint8_t reg = CONVERSION_REG;
    write(file, &reg, 1);
    usleep(8000);

    uint8_t data[2];
    if (read(file, data, 2) != 2) {
        std::cerr << "Failed to read ADC!\n";
        return -1;
    }

    int16_t raw_adc = (data[0] << 8) | data[1];
    double voltage = raw_adc * 4.096 / 32767.0; 
    return voltage;
}

double calculateTDS(double voltage, double temperatureC = 25.0) {
    double compensationCoefficient = 1.0 + 0.02 * (temperatureC - 25.0); 
    double compensatedVoltage = voltage / compensationCoefficient;

    
    double tds = (compensatedVoltage / 2.3) * 1000.0;
    return tds;
}

int main() {
    const char* i2c_dev = "/dev/i2c-2";
    int file = open(i2c_dev, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open I2C device\n";
        return 1;
    }

    if (ioctl(file, I2C_SLAVE, ADS1115_ADDRESS) < 0) {
        std::cerr << "Failed to connect to ADS1115\n";
        return 1;
    }

    while (true) {
        double voltage = readADC(file, 0);  
        if (voltage < 0) continue;

        double temperature = 25.0;  
        double tds = calculateTDS(voltage, temperature);

        std::cout << "TDS Voltage: " << voltage << " V"
                  << ", TDS: " << tds << " ppm" << std::endl;

        usleep(1000000); 
    }

    close(file);
    return 0;
}
