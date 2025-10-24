#
# Copyright (c) 2025 XDU-IRobot
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

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/core/*.cc"
        "${CMAKE_CURRENT_LIST_DIR}/device/*.cc"
        "${CMAKE_CURRENT_LIST_DIR}/modules/*.cc")

if ("${LIBRM_PLATFORM}" STREQUAL "STM32")
    file(GLOB_RECURSE PLATFORM_SOURCES "${CMAKE_CURRENT_LIST_DIR}/hal/stm32/*.cc")
elseif (${LIBRM_PLATFORM} STREQUAL "LINUX")
    if (${LIBRM_PLATFORM_LINUX_TYPE} STREQUAL "JETSON")
        file(GLOB_RECURSE PLATFORM_SOURCES "${CMAKE_CURRENT_LIST_DIR}/hal/linux/*.cc"
                "${CMAKE_CURRENT_LIST_DIR}/hal/jetson/*.cc")
    elseif (${LIBRM_PLATFORM_LINUX_TYPE} STREQUAL "RASPI")
        file(GLOB_RECURSE PLATFORM_SOURCES "${CMAKE_CURRENT_LIST_DIR}/hal/linux/*.cc"
                "${CMAKE_CURRENT_LIST_DIR}/hal/raspi/*.cc")
    else ()
        file(GLOB_RECURSE PLATFORM_SOURCES "${CMAKE_CURRENT_LIST_DIR}/hal/linux/*.cc")
    endif ()
endif ()

add_library(${PROJECT_NAME} STATIC ${SOURCES} ${PLATFORM_SOURCES})
