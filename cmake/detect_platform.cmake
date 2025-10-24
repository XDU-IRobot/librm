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

set(LIBRM_AVAIABLE_PLATFORMS "STM32" "LINUX")
set(LIBRM_AVAILABLE_LINUX_TYPES "GENERAL" "JETSON" "RASPI")

# 尝试通过查找和平台有关的库
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)      # raspberry pi/orange pi
find_package(JetsonGPIO QUIET)                       # jetson

get_directory_property(DEFS COMPILE_DEFINITIONS)

if (DEFINED LIBRM_PLATFORM)                                         # 如果用户定义了LIBRM_PLATFORM变量
    if (NOT "${LIBRM_PLATFORM}" IN_LIST LIBRM_AVAIABLE_PLATFORMS)   # 但是用户定义的平台不在可用平台列表里
        message(WARNING "[librm]: Invalid platform: ${LIBRM_PLATFORM}\n"
                "[librm]: Available platforms: ${LIBRM_AVAIABLE_PLATFORMS}")
    endif ()
else ()                                                             # 就尝试自动检测平台
    if (TARGET STM32_Drivers)                                       # 存在cubemx生成的STM32_Drivers target
        set(LIBRM_PLATFORM "STM32")                                 # 那么就认为是STM32平台
    elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")                   # 如果是在Linux上编译
        set(LIBRM_PLATFORM "LINUX")                                 # 那就认为是Linux平台
    endif ()
endif ()

# 如果进行到这里LIBRM_PLATFORM还没有被定义，那么说明我们没有检测到平台，报错中断构建过程，并且提示用户手动设置LIBRM_PLATFORM变量
# 需要注意的是，如果已经到了上面的逻辑检测不到平台的地步，那就说明现在的环境或者工程多半有问题，即使手动设置了平台，也可能会有其他问题
# if LIBRM_PLATFORM is still not set, that means we failed to detect the platform
# in this case interrupt the build process and tell the user to set the LIBRM_PLATFORM variable manually
# NOTE THAT if we have reached this point where the detection above failed,
# it is likely that there are other issues with the environment or the project
if (NOT DEFINED LIBRM_PLATFORM)
    message(FATAL_ERROR "[librm]: Failed to detect the platform, please specify manually by setting LIBRM_PLATFORM variable.\r\n"
            "[librm]: Available platforms: ${LIBRM_AVAIABLE_PLATFORMS}")
endif ()

message(STATUS "[librm]: Platform: ${LIBRM_PLATFORM}")

# 如果顺利判断出平台是STM32，就进一步检测有没有FreeRTOS
set(LIBRM_FREERTOS_AVAILABLE false)
if (${LIBRM_PLATFORM} STREQUAL "STM32")
    if (TARGET FreeRTOS)
        message(STATUS "[librm]: Found FreeRTOS")
        set(LIBRM_FREERTOS_AVAILABLE true)
    endif ()
endif ()

# 如果顺利判断出平台是Linux，就根据找到的package进一步判断是哪一种嵌入式Linux
if (${LIBRM_PLATFORM} STREQUAL "LINUX")
    if (NOT ${WIRINGPI_LIBRARIES} STREQUAL "WIRINGPI_LIBRARIES-NOTFOUND")
        message(STATUS "[librm]: Found WiringPi")
        set(LIBRM_PLATFORM_LINUX_TYPE "RASPI")
    elseif (JetsonGPIO_FOUND)
        message(STATUS "[librm]: Found WiringPi")
        set(LIBRM_PLATFORM_LINUX_TYPE "JETSON")
    else ()
        set(LIBRM_PLATFORM_LINUX_TYPE "GENERAL")             # 如果什么都没找到就fallback到general linux，不提供平台驱动
    endif ()
    message(STATUS "[librm]: Linux type: ${LIBRM_PLATFORM_LINUX_TYPE}")
endif ()
