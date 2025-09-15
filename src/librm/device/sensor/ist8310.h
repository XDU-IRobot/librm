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
 * @file    librm/device/sensor/ist8310.h
 * @brief   IST8310磁力计类
 */

#ifndef LIBRM_DEVICE_SENSOR_IST8310_H
#define LIBRM_DEVICE_SENSOR_IST8310_H

#include "librm/hal/stm32/hal.h"
#if defined(HAL_I2C_MODULE_ENABLED)

#include "librm/core/typedefs.h"
#include "librm/hal/stm32/i2c_device.h"

namespace rm::device {

enum class IST8310Status : u8 {
  NO_ERROR = 0x00,
  NO_SENSOR = 0x40,
  SENSOR_ERROR = 0x80,
};

/**
 * @brief IST8310磁力计
 */
class [[deprecated]] IST8310 : public rm::hal::stm32::I2cDevice {
 public:
  IST8310(I2C_HandleTypeDef &hi2c, GPIO_TypeDef *rst_port, u16 rst_pin);
  void Reset();
  void Update();
  [[nodiscard]] IST8310Status status() const;
  [[nodiscard]] f32 mag_x() const;
  [[nodiscard]] f32 mag_y() const;
  [[nodiscard]] f32 mag_z() const;

 private:
  GPIO_TypeDef *rst_port_;
  u16 rst_pin_;
  IST8310Status status_;
  f32 mag_[3]{0};
};

}  // namespace rm::device

#endif

#endif  // LIBRM_DEVICE_SENSOR_IST8310_H