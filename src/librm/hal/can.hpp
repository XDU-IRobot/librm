/*
  Copyright (c) 2025 XDU-IRobot

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/**
 * @file  librm/hal/can.hpp
 * @brief 根据平台宏定义决定Can的具体实现，并且在can_interface.h里提供一个接口类CanInterface实现多态
 */

#ifndef LIBRM_HAL_CAN_HPP
#define LIBRM_HAL_CAN_HPP

#include "librm/hal/can_interface.hpp"

#if defined(LIBRM_PLATFORM_STM32)
#include "librm/hal/stm32/bxcan.hpp"
#include "librm/hal/stm32/fdcan.hpp"
#elif defined(LIBRM_PLATFORM_LINUX)
#include "librm/hal/linux/socketcan.hpp"
#endif

namespace rm::hal {
#if defined(LIBRM_PLATFORM_STM32)
#if defined(HAL_CAN_MODULE_ENABLED)
using Can = stm32::BxCan;
#elif defined(HAL_FDCAN_MODULE_ENABLED)
using Can = stm32::FdCan;  // TODO: 实现FdCan类
#endif
#elif defined(LIBRM_PLATFORM_LINUX)
using Can = linux_::SocketCan;
#endif
}  // namespace rm::hal

#endif  // LIBRM_HAL_CAN_HPP
