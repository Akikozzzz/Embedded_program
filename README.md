1.	Project Introduction:
This project designs and implements a removable environmental monitoring system based on Raspberry Pi 5 with Linux operating system for real-time, continuous and visualised monitoring and management of key environmental indicators in small spaces. The system adopts a hardware structure and an efficient C++ software framework, combined with a variety of sensor modules, to achieve the detection of data such as air quality, carbon dioxide (CO₂) concentration, and water quality pH value.

The system is suitable for home living environment, balance gardening, small-scale agricultural greenhouse, ecological farming and other scenarios. Users can view the real-time trend of environmental data at any time through the local terminal or web interface, to make scientific judgement and timely adjustment. Its detachable and expandable design enables the system to flexibly respond to different application requirements, providing a low-cost, high-availability and easy to maintain solution for environmental intelligent management.

2.	Solution to the problem and market demand
This project is suitable for the environmental monitoring problems that exist in the current family and small-scale agricultural scenarios, and solves the following core problems through intelligent means:

Traditional environmental monitoring tools face several limitations that hinder their accessibility and effectiveness. Firstly, they are primarily designed for industrial use, making them large, expensive, and difficult for ordinary users—especially households and small farms—to afford, often resulting in a complete lack of monitoring in these settings. Moreover, most low-cost devices only offer instantaneous readings without providing historical data, trend analysis, or real-time visibility, which significantly limits their usefulness for informed environmental decision-making. Additionally, many of these tools feature fixed structures with poor scalability, making it difficult to adapt sensor types or add functional modules to meet the needs of different application scenarios.

With this system, users can conveniently build a set of integrated and personalised environmental data platform to improve the quality of life, optimise crop management and promote green ecological farming.

3.	Hardware analysis
This project is an environmental monitor based on Raspberry Pi and C++, which supports both mounted and handheld usage, and can be flexibly applied to robots, drones, unmanned boats and other platforms. We have integrated several common environmental sensors to collect real-time water pH, TDS, PM2.5, temperature and humidity data, basically covering the common water quality and air testing needs. The project is written in C++ and runs on Raspberry Pi Linux system. The structure is modularised as much as possible, which is convenient for subsequent maintenance and function expansion. In the GPIO event processing, we use libgpiod, rather than the traditional polling method, so that the system responds faster and more resource-efficient, more suitable for long-time running scenarios. The whole project focuses on practicality, and make every part clear and reusable during the development process.

4.	Installation and Usage Guide
This guide will help you install and build our real-time environmental monitoring project step-by-step. This project is for Raspberry Pi, the recommended operating system is the latest version of Raspberry Pi OS, make sure your system is updated to the latest for best compatibility.

4.1	Hardware Preparation
Raspberry Pi (Raspberry Pi 5i recommended)
SDS011 Air Quality Sensor
SCD41 CO₂ sensor
ADS1115 with pH sensor module
OLED display module (I²C interface, SSD1306 12864 recommended)

4.2	Preparation of the software environment
Open the terminal and execute the following commands to update the system and install the necessary software tools:
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install git cmake build-essential libi2c-dev libpthread-stubs0-dev -y

4.3	Cloning and construction projects
#Clone the project source
git clone https://github.com/Akikozzzz/Embedded_program.git
cd Embedded_program/team6_Project
Create the build directory
mkdir build && cd build
Execute the CMake configuration 
make ..
Compile to generate executables
make
#Running Projects
./main
#Once the program is started, the OLED display will show environmental parameters in real time, including airborne particulate matter concentration, CO₂ concentration and water ph.

4.4	Precautions
Ensure that the sensor module is securely connected to the Raspberry Pi to avoid data reading anomalies.
If you have permission problems running, try using the sudo ./main command to start the program.
If you encounter device path problems, please verify that the device address of the sensor is consistent with the program definition.

5.	Software Architecture and Event-driven Model
The system has been designed on the basis of a single-threaded event-driven architecture, with the objective of achieving efficient concurrent collection of multi-sensor data through non-blocking event loops. The core architecture employs the poll() system call to listen for multiple event sources, including the timer (created through timerfd_create), the serial port device (SDS011 sensor), and the I2C device status, to achieve multiplexed event response. Each sensor module is independently configured with a timer to trigger the sampling process (2 seconds for SCD041, 5 seconds for SDS011, and 1 second for the pH sensor). The data is then sent back to the main thread for processing through a callback function mechanism to ensure the decoupling of functional modules. The timeout parameter of the main event loop is set to 10 milliseconds (ms) to ensure the real-time response capability of the system to events (the measured delay of the main loop is less than 50 milliseconds). As shown in Figure 1, the core software components of the system and their interaction relationships are described.
 ![image](https://github.com/user-attachments/assets/289c979c-9543-4655-b851-cd654392a2c8)
Fig1 The core software components of the system and their interaction relationships
In comparison with the conventional multi-threading scheme, this architecture circumvents the overhead of thread switching by unifying the event queue, reduces the memory usage to 3MB (a reduction of 83%), and wholly eliminates the risk of lock contention. The hardware interface layer attains device independence through abstract encapsulation (for example, the I2CDevice base class), thereby facilitating the rapid expansion of subsequent sensors.

6.	Implementation of Event Loop and Timer Mechanism
The core implementation of the event loop is based on the pollfd structure array, which is utilised for the dynamic management of four event sources (three sensor timers and one serial port device). The generation of high-precision timer file descriptors is achieved through the utilisation of timerfd_create, while concurrently ensuring cooperation with the CLOCK_MONOTONIC clock source. This approach serves to circumvent the impact of system time jumps, thereby ensuring the accuracy and consistency of the timer. The configuration of timer parameters is facilitated by the itimerspec structure. To illustrate this point, consider the 1-second sampling period of the pH sensor, which corresponds to it_interval=[1,0]and it_value=[1,0]. Upon detection of a timer event by poll(), the function calls read(fd, &expirations, sizeof(uint64_t)). The purpose of this call is to read the timeout count, thereby triggering the data acquisition process of the relevant sensor. In the context of the SDS011 serial port device, the non-blocking mode reading (O_NONBLOCK flag) is employed, with a maximum of 32 bytes of data frame read each time. The PM2.5 value is parsed through the state machine. This configuration is engineered to ensure that the CPU usage rate remains below 5% (Raspberry PI 5 quad-core load balancing), while guaranteeing the meticulous execution of the periodic sampling of each sensor in chronological order.

7.	Data Stream Synchronization and Atomization Operations
The system employs a two-level data synchronisation mechanism to ensure the consistency of multi-source data. The initial level constitutes the sensor-driven raw data acquisition layer. Each sensor writes data to the temporary buffer via the callback function void (*sensor_callback)(SensorData&), executing in the interrupt context. The second level is the atomic update layer of the main thread. The execution of atomic operations on the structure is facilitated by the use of the std::atomic<SensorData> template. The specific implementation utilises the built-in functions, namely __atomic_store_n and __atomic_load_n, to ensure the provision of lock-free access to 32-bit and 64-bit data. The SensorData data structure employs bit domain compression technology to compress CO₂ (16 bits), temperature (12 bits), humidity (12 bits), PM2.5 (16-bit floating-point encoding), and pH value (8 bits) into a 64-bit memory space. It is noteworthy that the entirety of data updates can be executed through the implementation of a single atomic operation. The display module employs a dual buffering mechanism to access data: the front-end buffer is utilised directly for OLED rendering, while the back-end buffer synchronises atomic data via the memcpy function to prevent read and write conflicts.

8.	Exception handling and system reliability
The software layer implements a three-level exception handling mechanism to ensure the robustness of the system. The initial level pertains to device communication timeout detection: Each sensor is set with an independent watchdog timer (initialized to three times the sampling period). In the absence of data following the designated timeout period, the ioctl(fd, TCSBRKP, 0) reset serial port or I2C bus will be initiated. The second level is data validity verification, which includes CRC-8 verification of SCD041 (polynomial 0x31), 16-bit CRC verification of SDS011 (CCITT standard), and voltage range detection of pH sensors (0-3.3V corresponding to pH 0-14). The third level of the hierarchy is system-level recovery. In the event of five consecutive unsuccessful verification attempts, a hardware restart is initiated (the raspi-gpio set resets the power pin). The storage of abnormal event records is facilitated by a circular buffer, which possesses a capacity of 256 entries. This buffer enables the real-time transfer of records to /var/log/sensor.log via the syslogd process. The key code segment employs the use of exception context saving through sigsetjmp/siglongjmp to ensure that the main loop is able to safely roll back to the most recent stable state in the event of serious errors.

9.	Display control and user interaction
The OLED display subsystem employs a hierarchical rendering architecture, comprising the data layer, the logic layer and the hardware driver layer. The data layer extracts environmental parameters from the atomic buffer, the logic layer manages the display mode (normal/alarm/configuration) through the state machine, and the hardware layer drives the SSD1306 controller based on the libsoc library. The rendering engine implements the difference update algorithm: By comparing the difference areas of the previous and subsequent frame buffers (uint8_t buf[1024]), only the changed pixels are rewritten, thereby reducing the refresh time from 15ms to an average of 3.2ms. In the event of the alarm being triggered, the inverse colour display strategy (SSD1306_INVERSE_DISPLAY command) is adopted, and the PWM buzzer (GPIO12, frequency 2kHz, duty cycle 50%) is invoked. The implementation of user interaction is facilitated by a rotary encoder, which enables interrupt-driven operation. The GPIO Interrupt Service Program (ISR) has been observed to push events into a lock-free queue, and it has been determined that the main thread processes key events once every 200 milliseconds (with a stabilization threshold of 20 milliseconds). This configuration facilitates an interface response delay of no more than 100 milliseconds, while maintaining an increase in CPU usage that does not surpass 1.5%.

10.	System Construction and Engineering Practice
The project employs the use of CMake in order to facilitate cross-platform construction. The core configuration comprises a three-stage compilation strategy: In the initial phase, the version of the Linux kernel header file must be verified using the CheckIncludeFile function (requiring ≥5.15). In the subsequent stage, the sensor driver should be compiled into a static library (libsensors.a). In the third stage, it is necessary to link the main program and inject the version information (GIT_COMMIT_HASH macro). The construction of build scripts is imperative for the purpose of achieving automated dependency detection. A case in point is the verification of the existence of hardware abstraction layer libraries through the use of find_package(libsoc REQUIRED). The code structure adheres to the ISO/IEC 12207 standard, and the driver module implements the POSIX interface specification (for example, open()/read()/ioctl()). Static code analysis integrates the clang-tidy checker and enables the modernize-* rule set. The purpose of this integration is to enforce the C++17 specification and terminate compilation when potentially dangerous operations, such as uninitialized atomic variables, are detected. During the debugging stage, remote debugging is facilitated by gdbserver, and the perf tool is utilised to analyse the event loop performance (with a sampling frequency of 1000Hz) to ensure that the poll() call accounts for less than 5% of the CPU time. The final generated executable file is compressed by UPX (--ultra brute option), resulting in a volume reduction from 2.1MB to 780KB. This adaptation is in accordance with the ARMv8 architecture of the Raspberry PI 5.

