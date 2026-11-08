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
 * @file    librm/hal/linux/gpio.hpp
 * @brief   根据 libgpiod 版本选择具体的 GPIO 实现
 */

#ifndef LIBRM_HAL_LINUX_GPIO_HPP
#define LIBRM_HAL_LINUX_GPIO_HPP

#if defined(LIBRM_PLATFORM_LINUX)

#if defined(LIBRM_LIBGPIOD_API_V2)
#include "librm/hal/linux/gpio_v2.hpp"
#else
#include "librm/hal/linux/gpio_v1.hpp"
#endif

#endif  // LIBRM_PLATFORM_LINUX

#endif  // LIBRM_HAL_LINUX_GPIO_HPP
