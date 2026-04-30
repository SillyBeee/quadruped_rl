cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(GENERATED_DDS_DIR "${CMAKE_BINARY_DIR}/generated/idl")
file(GLOB GENERATED_DDS_SRCS CONFIGURE_DEPENDS "${GENERATED_DDS_DIR}/*.cxx")

add_executable(core_app
    main.cpp
    ${GENERATED_DDS_SRCS}
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