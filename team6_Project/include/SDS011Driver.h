#ifndef SDS011_DRIVER_H
#define SDS011_DRIVER_H

#include <functional>
#include <termios.h>

class SDS011Driver {
public:
    using SensorCallback = std::function<void(float pm25, float pm10)>;

    SDS011Driver(const char* serial_path = "/dev/ttyUSB0");
    ~SDS011Driver();

    void register_callback(SensorCallback callback);
    void start();
    void stop();
    void process();
    int getSerialFd() const;

private:
    int serial_fd_;
    SensorCallback callback_;
    void configure_serial();
};

#endif // SDS011_DRIVER_H
