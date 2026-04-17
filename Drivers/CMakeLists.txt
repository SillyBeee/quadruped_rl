cmake_minimum_required(VERSION 3.10)

add_subdirectory(Logger)
add_subdirectory(Serial_Driver)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/DM_LinkX_Driver)
    add_subdirectory(DM_LinkX_Driver)
endif()