cmake_minimum_required(VERSION 3.10)


add_library(driver_serial STATIC
    serial_driver.cpp
)

add_library(Drivers::SerialDriver ALIAS driver_serial)

# 对外暴露头文件目录
target_include_directories(driver_serial
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(driver_serial
    PUBLIC
        Drivers::Logger
)

target_compile_features(driver_serial PUBLIC cxx_std_17)