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
 * @file  librm/hal/stm32/helper_macro.hpp
 * @brief STM32 HAL库辅助宏
 */

#ifndef LIBRM_HAL_STM32_HELPER_MACRO_HPP
#define LIBRM_HAL_STM32_HELPER_MACRO_HPP

#include "librm/core/exception.hpp"
#include "librm/hal/stm32/hal.hpp"

#define LIBRM_STM32_HAL_ASSERT(hal_api_call)  \
  do {                                        \
    if ((hal_api_call) != HAL_OK) {           \
      rm::Throw(rm::hal_error(hal_api_call)); \
    }                                         \
  } while (0)

#endif