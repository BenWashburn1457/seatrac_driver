cmake_minimum_required(VERSION 3.6)
project(seatrac_simple_pinger_example VERSION 0.1)

find_package(seatrac_driver REQUIRED)

add_executable(data_sender_example
    src/data_sender_example.cpp
)
target_link_libraries(data_sender_example PRIVATE seatrac_driver)
