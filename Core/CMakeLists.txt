cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_executable(core_app
    main.cpp
)

target_compile_features(core_app PRIVATE cxx_std_17)

find_package(fastcdr 2 REQUIRED)
find_package(fastdds 3 REQUIRED)

target_link_libraries(core_app
    PRIVATE
        Drivers::Logger
        Drivers::SerialDriver
        DMW::Core
        fastdds
        fastcdr
)