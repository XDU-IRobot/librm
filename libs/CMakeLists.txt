cmake_minimum_required(VERSION 3.13)

if (POLICY CMP0167)
    # cmake 3.30起移除了FindBoost模块，这里直接用boost的config文件
    cmake_policy(SET CMP0167 NEW)
endif ()

# 第三方库的测试和示例一律不构建
# 这些库有些用 option() 定义开关，CMP0077 OLD下普通变量会被option() 丢弃，必须写进cache才能覆盖。
set(BUILD_TESTS OFF CACHE BOOL "Build third-party unit tests" FORCE)
set(BUILD_EXHAUSTIVE_TESTS OFF CACHE BOOL "Build third-party exhaustive tests" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build third-party examples" FORCE)
set(WITH_TESTS OFF CACHE BOOL "Build eternal tests" FORCE)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Build benchmark tests" FORCE)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "Build benchmark gtest tests" FORCE)

# etl 会 include(GNUInstallDirs)，交叉编译时探测不到目标架构会报找不到 LIBDIR 的警告，
# 预先写进 cache 让它跳过默认值计算
if (NOT DEFINED CMAKE_INSTALL_LIBDIR)
    set(CMAKE_INSTALL_LIBDIR "lib" CACHE PATH "Installation directory for libraries")
endif ()

if(NOT LIBRM_PLATFORM STREQUAL "STM32")
    # boost::asio
    find_package(Boost REQUIRED)
    if (TARGET Boost::headers)
        set(BOOST_HEADERS_TARGET Boost::headers)
    else ()
        set(BOOST_HEADERS_TARGET Boost::boost)
    endif ()
    # pthread
    find_package(Threads REQUIRED)
    # libgpiod（GPIO）
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBGPIOD REQUIRED IMPORTED_TARGET libgpiod)
endif()

# kalman - https://github.com/mherb/kalman -  https://github.com/SF4cl/kalman.git
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/kalman)

# eigen - https://eigen.tuxfamily.org
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/eigen)

# etl - https://github.com/ETLCPP/etl
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/etl)

# hipnuc sdk
add_subdirectory(hipnuc_sdk)

# Ardupilot filter library
add_subdirectory(ardupilot_filter_lib)

# rpc_core - https://github.com/shuai132/rpc_core
# rpc_core 声明的 CMake 版本过旧，会在配置时产生 deprecation 警告，这里只在引入它时临时抑制
if (COMMAND cmake_diagnostic)
    # CMake 4.4 起，内建 deprecation 警告由诊断系统控制，需要先把 CMP0218 设为 NEW
    cmake_policy(SET CMP0218 NEW)
    cmake_diagnostic(PUSH)
    cmake_diagnostic(SET CMD_DEPRECATED IGNORE)
    add_subdirectory(rpc_core)
    cmake_diagnostic(POP)
else ()
    set(CMAKE_WARN_DEPRECATED OFF)
    add_subdirectory(rpc_core)
    set(CMAKE_WARN_DEPRECATED ON)
endif ()

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

# ETL等第三方模板代码在GCC下会报 "parameter passing for argument of type ... changed in GCC 7.1"
# 这类psabi提示，只是ABI说明、与功能无关，这里统一关掉
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(third_party INTERFACE -Wno-psabi)
endif ()
if(LIBRM_PLATFORM STREQUAL "LINUX")
    target_link_libraries(third_party INTERFACE
        ${BOOST_HEADERS_TARGET}
        Threads::Threads
        PkgConfig::LIBGPIOD
    )
    if (LIBGPIOD_VERSION VERSION_GREATER_EQUAL "2.0")
        target_compile_definitions(third_party INTERFACE LIBRM_LIBGPIOD_API_V2)
    endif ()
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
