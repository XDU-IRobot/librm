/*
  Copyright (c) 2026 XDU-IRobot

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
#include "librm/hal/throttled_can.hpp"

#if defined(LIBRM_PLATFORM_STM32)
#include "librm/hal/stm32/bxcan.hpp"
#include "librm/hal/stm32/fdcan.hpp"
#include "stm32/mcp2515.hpp"
#elif defined(LIBRM_PLATFORM_LINUX)
#include "librm/hal/linux/socketcan.hpp"
#endif

namespace rm::hal {
#if defined(LIBRM_PLATFORM_STM32)
#if defined(HAL_CAN_MODULE_ENABLED)
using Can = stm32::BxCan;
template <size_t MaxQueueSize = 32, modules::SchedulingPolicy Policy = modules::SchedulingPolicy::kPriorityFifo>
using ThrottledCan = detail::ThrottledCan<stm32::BxCan, 8, MaxQueueSize, Policy>;
#elif defined(HAL_FDCAN_MODULE_ENABLED)
using Can = stm32::FdCan;
template <size_t MaxQueueSize = 32, modules::SchedulingPolicy Policy = modules::SchedulingPolicy::kPriorityFifo>
using ThrottledCan = detail::ThrottledCan<stm32::FdCan, 64, MaxQueueSize, Policy>;
#endif

#if defined(HAL_SPI_MODULE_ENABLED) && defined(HAL_GPIO_MODULE_ENABLED)
template <size_t MaxQueueSize = 32, modules::SchedulingPolicy Policy = modules::SchedulingPolicy::kPriorityFifo>
using ThrottledMcp2515 = detail::ThrottledCan<stm32::Mcp2515, 8, MaxQueueSize, Policy>;
#endif

#elif defined(LIBRM_PLATFORM_LINUX)
using Can = linux_::SocketCan;
template <size_t MaxQueueSize = 32, modules::SchedulingPolicy Policy = modules::SchedulingPolicy::kPriorityFifo>
using ThrottledCan = detail::ThrottledCan<linux_::SocketCan, 8, MaxQueueSize, Policy>;
#endif
}  // namespace rm::hal

#endif  // LIBRM_HAL_CAN_HPP
