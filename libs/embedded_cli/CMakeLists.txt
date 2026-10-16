cmake_minimum_required(VERSION 3.20)
project(embedded_cli)

add_library(embedded_cli INTERFACE)
target_include_directories(embedded_cli INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
