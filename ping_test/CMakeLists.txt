cmake_minimum_required(VERSION 3.6)
project(seatrac_simple_pinger_example VERSION 0.1)

find_package(seatrac_driver REQUIRED)

add_executable(usbl_test
    src/usbl_test.cpp
)
target_link_libraries(usbl_test PRIVATE seatrac_driver)
