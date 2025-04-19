cmake_minimum_required(VERSION 3.10)
project(EnvironmentMonitor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(include)

set(SOURCES
    src/main3.cpp
    src/OLEDDriver.cpp
    src/PHSensorReader.cpp
    src/SCD41Driver.cpp
    src/SDS011Driver.cpp
)

add_executable(main ${SOURCES})

target_link_libraries(main pthread)
