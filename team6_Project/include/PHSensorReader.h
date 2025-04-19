#ifndef PH_SENSOR_READER_H
#define PH_SENSOR_READER_H

#include <iostream>
#include <functional>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/timerfd.h>
#include <linux/i2c-dev.h>
#include <poll.h>
#include <cstring>
#include <errno.h>

#define ADS1115_ADDRESS 0x48
#define CONVERSION_REG 0x00
#define CONFIG_REG 0x01

class PHSensorReader {
public:
    using Callback = std::function<void(double, double)>;

    PHSensorReader(const std::string& device, int channel = 0);

    ~PHSensorReader();

    bool init();

    void registerCallback(Callback cb);

    void handleTimerEvent();

    int getTimerFd() const;

private:
    bool tryReadADC(double& voltage_out);
    double voltageToPH(double voltage);

    std::string device_path_;
    int channel_;
    int fd_;
    int timer_fd_;
    Callback callback_;
};

#endif // PH_SENSOR_READER_H
