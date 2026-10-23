cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(lara)



find_path(DYNAMIXEL_SDK_INCLUDE_DIR
    NAMES dynamixel_sdk/dynamixel_sdk.h
    PATHS /usr/local/include /usr/include
)

find_library(DYNAMIXEL_SDK_LIBRARY
    NAMES dxl_sbc_cpp
    PATHS /usr/local/lib /usr/lib
)

if (NOT DYNAMIXEL_SDK_INCLUDE_DIR OR NOT DYNAMIXEL_SDK_LIBRARY)
    message(FATAL_ERROR "Dynamixel SDK not found")
endif()

add_library(dynamixel_sdk UNKNOWN IMPORTED)

set_target_properties(dynamixel_sdk PROPERTIES
    IMPORTED_LOCATION ${DYNAMIXEL_SDK_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${DYNAMIXEL_SDK_INCLUDE_DIR}
)

# Add the library subdirectory
add_subdirectory(../robot ${CMAKE_BINARY_DIR}/robot_build)

# Get all .cpp test files in this directory
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS *.cpp)

foreach(test_src ${TEST_SOURCES})
    # Get the file name without the directory or extension
    get_filename_component(test_name ${test_src} NAME_WE)

    # Create an executable per test file
    add_executable(${test_name} ${test_src})

    # Link with the robotlib
    target_link_libraries(${test_name} PRIVATE robotlib)
endforeach()

