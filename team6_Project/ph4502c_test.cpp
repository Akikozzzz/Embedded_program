#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define ADS1115_ADDRESS 0x48  // ADS1115 I2C address
#define CONVERSION_REG 0x00   // ADC conversion register
#define CONFIG_REG 0x01       // Configuration register

// Function to read ADC value
double readADC(int file, int channel) {
    uint8_t config[3] = {CONFIG_REG, 0xC3, 0x83};  // Default A0
    if (channel == 1) config[1] = 0xD3;  // Select A1 if needed

    if (write(file, config, 3) != 3) {
        std::cerr << "Failed to configure ADS1115" << std::endl;
        return -1;
    }

    usleep(8000);  // Wait for ADC conversion
    uint8_t reg = CONVERSION_REG;
    if (write(file, &reg, 1) != 1) {
        std::cerr << "Failed to set conversion register" << std::endl;
        return -1;
    }

    usleep(8000);  // Wait for ADC conversion
    uint8_t data[2];
    if (read(file, data, 2) != 2) {
        std::cerr << "Failed to read ADC!" << std::endl;
        return -1;
    }

    // Convert ADC data to voltage
    int16_t raw_adc = (data[0] << 8) | data[1];
    double voltage = raw_adc * 4.096 / 32767.0;
    return voltage;
}

// Function to convert voltage to pH
double voltageToPH(double voltage) {
    double pH7_voltage = 2.5;  // Default 2.5V at pH 7.0
    double slope = -0.18;  // Voltage change per pH unit

    double pH = 7.0 + (voltage - pH7_voltage) / slope;
    return pH;
}

int main() {
    int file;
    const char *i2c_dev = "/dev/i2c-2";  // Use correct I2C bus

    if ((file = open(i2c_dev, O_RDWR)) < 0) {
        std::cerr << "Failed to open I2C device" << std::endl;
        return -1;
    }

    if (ioctl(file, I2C_SLAVE, ADS1115_ADDRESS) < 0) {
        std::cerr << "Failed to communicate with ADS1115" << std::endl;
        return -1;
    }

    while (true) {
        // Read pH sensor voltage (A0)
        double ph_voltage = readADC(file, 0);
        if (ph_voltage < 0) continue;

        // Convert voltage to pH value
        double pH_value = voltageToPH(ph_voltage);

        // Print results
        std::cout << "pH Sensor Voltage: " << ph_voltage << "V, pH Value: " << pH_value << std::endl;

        usleep(1000000);  // Read every 1 second
    }

    close(file);
    return 0;
}
