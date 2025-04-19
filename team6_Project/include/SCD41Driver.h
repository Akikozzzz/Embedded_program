#ifndef SCD41_DRIVER_H
#define SCD41_DRIVER_H

#include <functional>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/timerfd.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cerrno>
#include <iostream>

class SCD41Driver {
public:
    using SensorCallback = std::function<void(int, float, float)>;

    SCD41Driver(const char* device, uint8_t address);
    ~SCD41Driver();

    void register_callback(SensorCallback callback);
    void start();
    void stop();
    int getTimerFd() const;
    void handleTimerEvent();

private:
    void send_command(uint16_t cmd);
    bool check_data_ready();
    void read_and_invoke_callback();

    int fd_;
    int timer_fd_;
    SensorCallback callback_;
};

#endif // SCD41_DRIVER_H
