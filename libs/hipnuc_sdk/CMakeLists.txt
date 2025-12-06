cmake_minimum_required(VERSION 3.10)
project(hipnuc_sdk C)

file(GLOB_RECURSE HIPNUC_SDK_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

add_library(hipnuc_sdk STATIC ${HIPNUC_SDK_SOURCES})

target_include_directories(hipnuc_sdk PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/hipnuc"
)
target_include_directories(hipnuc_sdk PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)