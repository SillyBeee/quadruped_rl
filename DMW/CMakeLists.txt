cmake_minimum_required(VERSION 3.10)

# Dependencies
if(NOT fastcdr_FOUND)
    find_package(fastcdr 2 REQUIRED)
endif()
if(NOT fastdds_FOUND)
    find_package(fastdds 3 REQUIRED)
endif()

# Sources and public headers
file(GLOB DMW_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# IDL generation
find_program(FASTDDSGEN_EXECUTABLE fastddsgen)
if(NOT FASTDDSGEN_EXECUTABLE)
    message(FATAL_ERROR "fastddsgen not found in PATH")
endif()

set(IDL_DIR "${CMAKE_SOURCE_DIR}/idl")
file(GLOB IDL_FILES CONFIGURE_DEPENDS "${IDL_DIR}/*.idl")
set(FASTDDS_GEN_DIR "${CMAKE_BINARY_DIR}/generated/idl")

add_custom_command(
    OUTPUT "${FASTDDS_GEN_DIR}/.dds_generated.stamp"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${FASTDDS_GEN_DIR}"
    COMMAND ${FASTDDSGEN_EXECUTABLE} -d "${FASTDDS_GEN_DIR}" -flat-output-dir -replace -typeros2 ${IDL_FILES}
    COMMAND ${CMAKE_COMMAND} -E touch "${FASTDDS_GEN_DIR}/.dds_generated.stamp"
    DEPENDS ${IDL_FILES}
    COMMENT "Generate Fast DDS types from ${IDL_FILES}"
    VERBATIM
)

add_custom_target(generate_dds_types DEPENDS "${FASTDDS_GEN_DIR}/.dds_generated.stamp")

# DMW core library
add_library(dmw_core STATIC ${DMW_SOURCES})
add_library(DMW::Core ALIAS dmw_core)

target_include_directories(dmw_core
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${FASTDDS_GEN_DIR}"
)

target_link_libraries(dmw_core
    PUBLIC
        fastdds
        fastcdr
)

add_dependencies(dmw_core generate_dds_types)
target_compile_features(dmw_core PUBLIC cxx_std_17)

# Export generated directory to parent scope so top-level CMake can use it
set(GENERATED_DDS_DIR "${FASTDDS_GEN_DIR}" PARENT_SCOPE)


