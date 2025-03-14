cmake_minimum_required(VERSION 3.10)
project(SCD041Test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Threads Library
find_package(Threads REQUIRED)

add_executable(scd041_test SCD041_TEST_CALLBACK.cpp)

# Link to the Thread Library
target_link_libraries(scd041_test pthread)