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
 * @file    librm/hal/stm32/gpio.cc
 * @brief   stm32的GPIO类
 */

#include "gpio.hpp"

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_GPIO_MODULE_ENABLED)

#include "librm/core/typedefs.hpp"
#include "librm/hal/gpio_interface.hpp"

namespace rm::hal::stm32 {

/**
 * @param port  GPIO端口
 * @param pin   引脚编号
 */
Pin::Pin(GPIO_TypeDef &port, const u16 pin) : port_(&port), pin_(pin) {}

/**
 * @param state 引脚状态
 */
void Pin::Write(const bool state) { HAL_GPIO_WritePin(this->port_, this->pin_, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }

/**
 * @return  引脚状态
 */
bool Pin::Read() const { return HAL_GPIO_ReadPin(this->port_, this->pin_) == GPIO_PIN_SET; }

}  // namespace rm::hal::stm32
#endif