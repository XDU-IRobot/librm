/*
  Copyright (c) 2024 XDU-IRobot

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
 * @file  librm/hal/stm32/hal.h
 * @brief 根据不同STM32型号选择不同的HAL库
 */

#ifndef LIBRM_HAL_STM32_HAL_H
#define LIBRM_HAL_STM32_HAL_H

#if defined(STM32F407xx) && __has_include("stm32f407xx.h") && __has_include("stm32f4xx_hal.h")
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#elif defined(STM32H723xx) && __has_include("stm32h723xx.h") && __has_include("stm32h7xx_hal.h")
#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#elif defined(STM32H743xx) && __has_include("stm32h743xx.h") && __has_include("stm32h7xx_hal.h")
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#elif defined(STM32L412xx) && __has_include("stm32l412xx.h") && __has_include("stm32l4xx_hal.h")
#include "stm32l412xx.h"
#include "stm32l4xx_hal.h"
#elif defined(STM32F427xx) && __has_include("stm32f427xx.h") && __has_include("stm32f4xx_hal.h")
#include "stm32f427xx.h"
#include "stm32f4xx_hal.h"
#endif

#endif  // LIBRM_HAL_STM32_HAL_H
