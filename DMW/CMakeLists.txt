cmake_minimum_required(VERSION 3.10)

# find_package(fastdds shared) 
if(NOT fastcdr_FOUND)
    find_package(fastcdr 2 REQUIRED)
endif()
if(NOT fastdds_FOUND)
    find_package(fastdds 3 REQUIRED)
endif()
# find_package(fastdds static) 


# 添加子目录
# add_subdirectory(???)


# 使用fastddsgen处理IDL文件
find_program(FASTDDSGEN_EXECUTABLE fastddsgen)
if(NOT FASTDDSGEN_EXECUTABLE)
    message(FATAL_ERROR "fastddsgen not found in PATH")
endif()

set(IDL_DIR ${CMAKE_SOURCE_DIR}/idl)
set(IDL_FILE ${IDL_DIR}/*.idl)
set(FASTDDS_GEN_DIR ${CMAKE_BINARY_DIR}/generated/idl)

add_custom_command(
    OUTPUT ${FASTDDS_GEN_DIR}/.dds_generated.stamp
    COMMAND ${CMAKE_COMMAND} -E make_directory ${FASTDDS_GEN_DIR}
    COMMAND ${FASTDDSGEN_EXECUTABLE} -d ${FASTDDS_GEN_DIR} -flat-output-dir -replace -typeros2 ${IDL_FILE}
    COMMAND ${CMAKE_COMMAND} -E touch ${FASTDDS_GEN_DIR}/.dds_generated.stamp
    DEPENDS ${IDL_FILE}
    COMMENT "Generate Fast DDS types from ${IDL_FILE}"
)

add_custom_target(generate_dds_types DEPENDS ${FASTDDS_GEN_DIR}/.dds_generated.stamp)

# Export generated directory to parent scope so top-level CMake can use it
set(GENERATED_DDS_DIR ${FASTDDS_GEN_DIR} PARENT_SCOPE)


