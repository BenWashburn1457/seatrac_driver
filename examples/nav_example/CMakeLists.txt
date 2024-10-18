cmake_minimum_required(VERSION 3.6)
project(seatrac_nav_example VERSION 0.1)

find_package(seatrac_driver QUIET)
if(NOT TARGET seatrac_driver)
    include(FetchContent)
    FetchContent_Declare(seatrac_driver
        GIT_REPOSITORY https://bitbucket.org/frostlab/seatrac_driver.git
        GIT_TAG        main
    )
    FetchContent_MakeAvailable(seatrac_driver)
endif()

add_executable(nav_example src/nav_example.cpp)
target_link_libraries(nav_example PRIVATE seatrac_driver)
