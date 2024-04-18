cmake_minimum_required(VERSION 3.6)
project(seatrac_simple_pinger_example VERSION 0.1)

find_package(seatrac_driver REQUIRED)

add_executable(calibration_example
    src/calibration_example.cpp
)
target_link_libraries(calibration_example PRIVATE seatrac_driver)
