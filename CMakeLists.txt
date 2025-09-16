#
# Copyright (c) 2024 XDU-IRobot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.13)
project(rm C CXX)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/check_cpp_std.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/detect_platform.cmake)

# main target
add_subdirectory(src)
# add third party libraries
add_subdirectory(libs)
# link third party libraries
target_link_libraries(${PROJECT_NAME} PUBLIC third_party)

if (${LIBRM_PLATFORM} STREQUAL "STM32")
    target_link_libraries(${PROJECT_NAME} PRIVATE STM32_Drivers)
    if (${LIBRM_FREERTOS_AVAILABLE})
        target_compile_definitions(${PROJECT_NAME} PUBLIC -DLIBRM_FREERTOS_AVAILABLE)
        target_link_libraries(${PROJECT_NAME} PRIVATE FreeRTOS)
    endif ()
endif ()

target_compile_definitions(${PROJECT_NAME} PUBLIC -DLIBRM_PLATFORM_${LIBRM_PLATFORM})
if (DEFINED LIBRM_PLATFORM_LINUX_TYPE)
    target_compile_definitions(${PROJECT_NAME} PUBLIC -DLIBRM_PLATFORM_LINUX_${LIBRM_PLATFORM_LINUX_TYPE})
endif ()
