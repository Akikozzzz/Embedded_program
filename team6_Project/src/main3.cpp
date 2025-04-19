#include <atomic>
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <poll.h>
#include <fcntl.h>
#include <sys/timerfd.h>
#include <ctime>
#include <fstream>
#include <cerrno>

#include "OLEDDriver.h"
#include "SCD41Driver.h"
#include "SDS011Driver.h"
#include "PHSensorReader.h"

OLEDDriver oled;

#define CO2_I2C_DEVICE "/dev/i2c-1"
#define SCD41_ADDR 0x62
#define PH_I2C_DEVICE "/dev/i2c-2"

std::atomic<bool> g_run_flag{true};
float co2_threshold = 1000.0f;
float pm25_threshold = 35.0f;

void signalHandler(int) {
    g_run_flag = false;
}

void logAlert(const std::string& message) {
    std::ofstream logFile("alert.log", std::ios::app);
    if (logFile.is_open()) {
        std::time_t now = std::time(nullptr);
        char timeStr[64];
        std::strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
        logFile << "[" << timeStr << "] " << message << std::endl;
    }
}

void displayToOLED(int co2, float temperature, float humidity, float pm25, double ph) {
    oled.clear();
    bool co2_alert = co2 > co2_threshold;
    bool pm25_alert = pm25 > pm25_threshold;
    char line[32];

    if (co2_alert || pm25_alert) {
        oled.drawText("!! ALERT !!", 0, 0);
        if (co2_alert) {
            snprintf(line, sizeof(line), "CO2 > %.0f", co2_threshold);
            oled.drawText(line, 0, 10);
        }
        if (pm25_alert) {
            snprintf(line, sizeof(line), "PM2.5 > %.1f", pm25_threshold);
            oled.drawText(line, 0, 20);
        }
        oled.drawText("OPEN WINDOW", 0, 30);
    } else {
        snprintf(line, sizeof(line), "TEMP: %.1fC", temperature);
        oled.drawText(line, 0, 0);
        snprintf(line, sizeof(line), "HUM : %.1f%%", humidity);
        oled.drawText(line, 0, 10);
        snprintf(line, sizeof(line), "CO2 : %dppm", co2);
        oled.drawText(line, 0, 20);
        snprintf(line, sizeof(line), "PM2.5: %.1fug", pm25);
        oled.drawText(line, 0, 30);
        snprintf(line, sizeof(line), "PH: %.2f", ph);
        oled.drawText(line, 0, 40);
    }
}

void combinedCallback(int co2, float temperature, float humidity, float pm25, float pm10, double ph, double ph_voltage) {
    std::cout << "=== Sensor Readings ===" << std::endl;
    std::cout << "CO2: " << co2 << " ppm\nTemp: " << temperature << " C\nHumidity: " << humidity
              << " %\nPM2.5: " << pm25 << " ug/m3\nPM10: " << pm10 << " ug/m3\npH: " << ph
              << " (" << ph_voltage << " V)" << std::endl;

    if (co2 > co2_threshold)
        logAlert("CO2 exceeded threshold: " + std::to_string(co2));
    if (pm25 > pm25_threshold)
        logAlert("PM2.5 exceeded threshold: " + std::to_string(pm25));
    if (ph < 5.5 || ph > 8.5)
        logAlert("Abnormal pH detected: " + std::to_string(ph));

    displayToOLED(co2, temperature, humidity, pm25, ph);
}

int main() {
    oled.init();
    signal(SIGINT, signalHandler);

    std::cout << "Enter CO2 alarm threshold (ppm): ";
    std::cin >> co2_threshold;
    std::cout << "Enter PM2.5 alarm threshold (ug/m3): ";
    std::cin >> pm25_threshold;

    SCD41Driver scd41(CO2_I2C_DEVICE, SCD41_ADDR);
    SDS011Driver sds011;
    PHSensorReader phsensor(PH_I2C_DEVICE);

    struct SensorData {
        int co2 = 0;
        float temperature = 0.0f;
        float humidity = 0.0f;
        float pm25 = 0.0f;
        float pm10 = 0.0f;
        double ph = 0.0;
        double ph_voltage = 0.0;
    } sensorData;

    scd41.register_callback([&](int co2, float temp, float hum) {
        sensorData.co2 = co2;
        sensorData.temperature = temp;
        sensorData.humidity = hum;
    });

    sds011.register_callback([&](float pm25, float pm10) {
        sensorData.pm25 = pm25;
        sensorData.pm10 = pm10;
    });

    if (!phsensor.init()) {
        std::cerr << "Failed to initialize pH sensor.\n";
        return 1;
    }
    phsensor.registerCallback([&](double voltage, double pH) {
        sensorData.ph_voltage = voltage;
        sensorData.ph = pH;
    });

    scd41.start();
    sds011.start();

    int one_sec_timer = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    struct itimerspec timer_spec = {{10, 0}, {10, 0}};
    timerfd_settime(one_sec_timer, 0, &timer_spec, nullptr);

    struct pollfd pfds[4];
    pfds[0].fd = scd41.getTimerFd(); pfds[0].events = POLLIN;
    pfds[1].fd = sds011.getSerialFd(); pfds[1].events = POLLIN;
    pfds[2].fd = one_sec_timer; pfds[2].events = POLLIN;
    pfds[3].fd = phsensor.getTimerFd(); pfds[3].events = POLLIN;

    while (g_run_flag) {
        int ret = poll(pfds, 4, 10000);
        if (ret < 0 && errno != EINTR) {
            std::cerr << "Poll error: " << strerror(errno) << std::endl;
            break;
        }

        if (ret > 0) {
            if (pfds[0].revents & POLLIN) scd41.handleTimerEvent();
            if (pfds[1].revents & POLLIN) sds011.process();
            if (pfds[2].revents & POLLIN) {
                uint64_t expirations;
                read(one_sec_timer, &expirations, sizeof(expirations));
                combinedCallback(
                    sensorData.co2,
                    sensorData.temperature,
                    sensorData.humidity,
                    sensorData.pm25,
                    sensorData.pm10,
                    sensorData.ph,
                    sensorData.ph_voltage
                );
            }
            if (pfds[3].revents & POLLIN) phsensor.handleTimerEvent();
        }
    }

    close(one_sec_timer);
    std::cout << "Sensor reading stopped." << std::endl;
    return 0;
}
