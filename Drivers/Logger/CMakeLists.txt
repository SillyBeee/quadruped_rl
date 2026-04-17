cmake_minimum_required(VERSION 3.10)


add_library(driver_logger STATIC
    logger.cpp
)

add_library(Drivers::Logger ALIAS driver_logger)

# 对外暴露头文件目录
target_include_directories(driver_logger
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# logger.hpp 里直接包含了 spdlog 头，所以建议 PUBLIC 传递依赖
find_package(spdlog REQUIRED)
target_link_libraries(driver_logger
    PUBLIC
        spdlog::spdlog
)

target_compile_features(driver_logger PUBLIC cxx_std_17)