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

#ifdef LIBRM_BYPASS_HAL_ASSERT  // 因为librm当前的错误处理系统还不够完善，所以暂时允许用户绕过HAL断言，后续会移除这个宏
#define LIBRM_STM32_HAL_ASSERT(hal_api_call) (hal_api_call)
#else
#define LIBRM_STM32_HAL_ASSERT(hal_api_call)         \
  do {                                               \
    const HAL_StatusTypeDef status = (hal_api_call); \
    if ((status) != HAL_OK) {                        \
      rm::Throw(rm::hal_error(status));              \
    }                                                \
  } while (0)
#endif

#endif