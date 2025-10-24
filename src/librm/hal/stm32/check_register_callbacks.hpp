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
 * @file  librm/hal/stm32/check_register_callbacks.h
 * @brief 由于对STM32的外设进行封装时，需要利用Register callback向HAL库注册回调函数，
 *        所以需要检查是否在CubeMX里启用了Register callback功能
 * @brief 本文件用于检查各个启用了的外设是否在CubeMX里启用了Register callback功能
 */

#ifndef LIBRM_HAL_STM32_CHECK_REGISTER_CALLBACK_HPP
#define LIBRM_HAL_STM32_CHECK_REGISTER_CALLBACK_HPP

// UART
#if defined(HAL_UART_MODULE_ENABLED)
#if defined(USE_HAL_UART_REGISTER_CALLBACKS)
#if (USE_HAL_UART_REGISTER_CALLBACKS != 1u)
#error "UART register callback must be enabled!"
#endif
#else
#error "Whether UART register callback is enabled or not is unknown!"
#endif
#endif

// bxCAN
#if defined(HAL_CAN_MODULE_ENABLED)
#if defined(USE_HAL_CAN_REGISTER_CALLBACKS)
#if (USE_HAL_CAN_REGISTER_CALLBACKS != 1u)
#error "CAN register callback must be enabled!"
#endif
#else
#error "Whether CAN register callback is enabled or not is unknown!"
#endif
#endif

// fdCAN
#if defined(HAL_FDCAN_MODULE_ENABLED)
#if defined(USE_HAL_FDCAN_REGISTER_CALLBACKS)
#if (USE_HAL_FDCAN_REGISTER_CALLBACKS != 1u)
#error "FDCAN register callback must be enabled!"
#endif
#else
#error "Whether FDCAN register callback is enabled or not is unknown!"
#endif
#endif

#endif  // LIBRM_HAL_STM32_CHECK_REGISTER_CALLBACK_HPP
