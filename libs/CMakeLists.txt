cmake_minimum_required(VERSION 3.13)

if (NOT LIBRM_PLATFORM STREQUAL "STM32")
    # boost::asio
    find_package(Boost REQUIRED COMPONENTS system)
    # pthread
    find_package(Threads REQUIRED)
endif ()

# kalman - https://github.com/mherb/kalman -  https://github.com/SF4cl/kalman.git
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/kalman)

# eigen - https://eigen.tuxfamily.org
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/eigen)

# etl - https://github.com/ETLCPP/etl
unset(BUILD_TESTS)      # suppress CMP0077 warning
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/etl)

# hipnuc sdk
add_subdirectory(hipnuc_sdk)

# pack up all third party libraries to a single target
add_library(third_party INTERFACE)
if (LIBRM_PLATFORM STREQUAL "LINUX")
    target_link_libraries(third_party INTERFACE Boost::system Threads::Threads)
endif ()

if (LIBRM_PLATFORM_LINUX_TYPE STREQUAL "RASPI")
    find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
    target_link_libraries(third_party INTERFACE ${WIRINGPI_LIBRARIES})
endif ()

target_link_libraries(third_party INTERFACE Kalman eigen etl::etl hipnuc_sdk)

set(BUILD_TESTS OFF)
set(BUILD_EXHAUSTIVE_TESTS OFF)
set(BUILD_EXAMPLES OFF)
