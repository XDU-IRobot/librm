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
 * @file    librm/hal/serial.hpp
 * @brief   根据平台宏决定具体实现
 */

#ifndef LIBRM_HAL_UART_HPP
#define LIBRM_HAL_UART_HPP

#include "librm/hal/serial_interface.hpp"

#if defined(LIBRM_PLATFORM_STM32)
#include "librm/hal/stm32/uart.hpp"
#elif defined(LIBRM_PLATFORM_LINUX)
#include "librm/hal/linux/serial.hpp"
#endif

namespace rm::hal {

#if defined(LIBRM_PLATFORM_STM32) && defined(HAL_UART_MODULE_ENABLED)
/**
 * @brief STM32 串口类型别名
 *
 * 模板参数 RxBufSize 决定双缓冲每块的大小（字节），编译时确定，无堆分配。
 * 典型用法：
 *   rm::hal::Serial<50> sbus_serial{huart5, UartMode::kNormal, UartMode::kInterrupt};
 */
template <rm::usize RxBufSize>
using Serial = stm32::Uart<RxBufSize>;

#elif defined(LIBRM_PLATFORM_LINUX)
using Serial = linux_::Serial;
#endif

}  // namespace rm::hal

#endif  // LIBRM_HAL_UART_HPP
