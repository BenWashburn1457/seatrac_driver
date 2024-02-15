cmake_minimum_required(VERSION 3.6)
project(seatrac_simple_pinger_example VERSION 0.1)

if(NOT TARGET seatrac_driver)
    find_package(seatrac_driver QUIET)
    if(NOT TARGET seatrac_driver)
        include(FetchContent)
        FetchContent_Declare(seatrac_driver
            GIT_REPOSITORY https://gitlab.ensta-bretagne.fr/narvorpi/seatrac_driver.git
            GIT_TAG        master
        )
        FetchContent_MakeAvailable(seatrac_driver)
    endif()
endif()

add_executable(simple_pinger
    src/simple_pinger.cpp
)
target_link_libraries(simple_pinger PRIVATE seatrac_driver)
