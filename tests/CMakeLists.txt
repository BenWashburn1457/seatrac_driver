
list(APPEND test_files
    src/serial_test.cpp
    src/checksum_test.cpp
    src/driver_test.cpp
)

foreach(filename ${test_files})
    get_filename_component(name ${filename} NAME_WE)
    set(target_name ${name}_${PROJECT_NAME})

    add_executable(${target_name} ${filename})
    target_link_libraries(${target_name} PRIVATE seatrac_driver)
endforeach()



