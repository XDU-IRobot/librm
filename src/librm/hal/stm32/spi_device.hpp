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
 * @file  librm/hal/stm32/spi_device.h
 * @brief SPI设备抽象
 * @todo  实现不够通用，有改进空间
 */

#ifndef LIBRM_HAL_STM32_SPI_DEVICE_HPP
#define LIBRM_HAL_STM32_SPI_DEVICE_HPP

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_SPI_MODULE_ENABLED) && defined(HAL_GPIO_MODULE_ENABLED)

#include "librm/core/typedefs.hpp"

namespace rm::hal::stm32 {
/**
 * @brief SPI设备基类
 */
class [[deprecated]] SpiDevice {
 public:
  SpiDevice(SPI_HandleTypeDef &hspi, GPIO_TypeDef *cs_gpio_port, u16 cs_pin);
  SpiDevice() = delete;
  ~SpiDevice() = default;

  void ReadByte(u8 reg);
  void ReadBytes(u8 reg, u8 len);
  void WriteByte(u8 reg, u8 data);

  [[nodiscard]] u8 single_byte_buffer() const;
  [[nodiscard]] const u8 *buffer() const;

 protected:
  void ReadWriteByte(u8 tx_data);
  void ReadWriteBytes(u8 reg, u8 len);

  SPI_HandleTypeDef *hspi_;
  GPIO_TypeDef *cs_gpio_port_;
  u16 cs_pin_;
  u8 single_byte_buffer_{};
  u8 buffer_[8]{0};
};
}  // namespace rm::hal::stm32

#endif

#endif  // LIBRM_HAL_STM32_SPI_DEVICE_H