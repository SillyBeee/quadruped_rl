cmake_minimum_required(VERSION 3.10)
project(quadruped_rl VERSION 0.0.1)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -luuid")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install")
endif()




add_subdirectory(DMW)  #DDS Middle Ware
add_subdirectory(Drivers)
add_subdirectory(Core)



# install(FILES ${CMAKE_SOURCE_DIR}/CMake/auroraConfig.cmake 
#                 DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake)
                


install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/scripts
    FILES_MATCHING
    PATTERN "*"
)

include(InstallRequiredSystemLibraries)
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_PROCESSOR})
set(CPACK_PACKAGE_DESCRIPTION "A motion control system for quadruped robots.")
set(CPACK_DEBIAN_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION})
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Unheard")
set(CPACK_PACKAGE_CONTACT "mahc@fftai.com")
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "fourier_hardware(>=1.1.2), fourier_dds(>=1.1.0)")
include(CPack)    