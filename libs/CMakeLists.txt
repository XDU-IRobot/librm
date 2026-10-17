cmake_minimum_required(VERSION 3.13)

if(NOT LIBRM_PLATFORM STREQUAL "STM32")
    # boost::asio
    find_package(Boost REQUIRED COMPONENTS system)
    # pthread
    find_package(Threads REQUIRED)
endif()

# kalman - https://github.com/mherb/kalman -  https://github.com/SF4cl/kalman.git
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/kalman)

# eigen - https://eigen.tuxfamily.org
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/eigen)

# etl - https://github.com/ETLCPP/etl
unset(BUILD_TESTS) # suppress CMP0077 warning
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/etl)

# hipnuc sdk
add_subdirectory(hipnuc_sdk)

# Ardupilot filter library
add_subdirectory(ardupilot_filter_lib)

# rpc_core - https://github.com/shuai132/rpc_core
add_subdirectory(rpc_core)

# eternal - https://github.com/mapbox/eternal
add_library(eternal INTERFACE)
target_include_directories(eternal INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/eternal/include
)

# embedded_cli - https://github.com/funbiscuit/embedded-cli
add_subdirectory(embedded_cli)

# pack up all third party libraries to a single target
add_library(third_party INTERFACE)
target_compile_definitions(third_party INTERFACE RPC_CORE_LOG_DISABLE_ALL)
if(LIBRM_PLATFORM STREQUAL "LINUX")
    target_link_libraries(third_party INTERFACE
        Boost::system
        Threads::Threads
    )
endif()

if(LIBRM_PLATFORM_LINUX_TYPE STREQUAL "RASPI")
    find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
    target_link_libraries(third_party INTERFACE ${WIRINGPI_LIBRARIES})
endif()

target_link_libraries(third_party INTERFACE
    Kalman
    eigen
    etl::etl
    hipnuc_sdk
    ardupilot_filter
    rpc_core
    eternal
    embedded_cli
)

set(BUILD_TESTS OFF)
set(BUILD_EXHAUSTIVE_TESTS OFF)
set(BUILD_EXAMPLES OFF)
