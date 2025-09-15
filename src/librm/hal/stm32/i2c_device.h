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
 * @file  librm/hal/stm32/i2c_device.h
 * @brief i2c设备抽象
 * @todo  实现不够通用，有改进空间
 */

#ifndef LIBRM_HAL_STM32_I2C_DEVICE_H
#define LIBRM_HAL_STM32_I2C_DEVICE_H

#include "librm/hal/stm32/hal.h"
#if defined(HAL_I2C_MODULE_ENABLED)

#include "librm/core/typedefs.h"

namespace rm::hal::stm32 {

/**
 * @brief I2C设备
 */
class [[deprecated]] I2cDevice {
 public:
  I2cDevice(I2C_HandleTypeDef &hi2c, u8 addr);
  I2cDevice() = delete;
  ~I2cDevice() = default;

  void Write(u8 reg, u8 *data, uint16_t size);
  void Read(u8 reg, uint16_t size);

  [[nodiscard]] const u8 *buffer() const;

 protected:
  I2C_HandleTypeDef *hi2c_;
  u8 addr_;
  u8 buffer_[8]{0};
};

}  // namespace rm::hal::stm32

#endif

#endif  // LIBRM_HAL_STM32_I2C_DEVICE_H
