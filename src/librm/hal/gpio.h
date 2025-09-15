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
 * @file  librm/hal/can.h
 * @brief 根据平台宏定义决定具体实现，并且在gpio_interface.h里提供一个接口类PinInterface实现多态
 */

#ifndef LIBRM_HAL_GPIO_H
#define LIBRM_HAL_GPIO_H

#include "librm/hal/gpio_interface.h"

#if defined(LIBRM_PLATFORM_STM32)
#include "librm/hal/stm32/gpio.h"
#elif defined(LIBRM_PLATFORM_LINUX)
#if defined(LIBRM_PLATFORM_LINUX_RASPI)
#include "librm/hal/raspi/gpio.hpp"
#endif
#endif

namespace rm::hal {
#if defined(LIBRM_PLATFORM_STM32) && defined(HAL_GPIO_MODULE_ENABLED)
using Pin = stm32::Pin;
#elif defined(LIBRM_PLATFORM_LINUX)
#if defined(LIBRM_PLATFORM_LINUX_RASPI)
template <int mode>
using Pin = raspi::Pin<mode>;
#elif defined(LIBRM_PLATFORM_LINUX_JETSON)
// TODO: JetsonGPIO GPIO wrapper
#endif
#endif
}  // namespace rm::hal

#endif  // LIBRM_HAL_GPIO_H
