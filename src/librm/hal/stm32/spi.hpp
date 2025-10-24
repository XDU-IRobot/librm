/*
  Copyright (c) 2025 XDU-IRobot

  Permission is hereby granted, free of u8ge, to any person obtaining a copy
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
 * @file  librm/hal/stm32/spi.h
 * @brief SPI类库
 */

#ifndef LIBRM_HAL_STM32_SPI_HPP
#define LIBRM_HAL_STM32_SPI_HPP

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_SPI_MODULE_ENABLED)

#include "librm/hal/spi_interface.hpp"
#include "librm/core/typedefs.hpp"

namespace rm::hal::stm32 {

class Spi final : public hal::SpiInterface {
 public:
  Spi(SPI_HandleTypeDef &hspi, usize timeout_ms);
  Spi() = delete;
  ~Spi() override = default;

  void Begin() override;
  void Write(const u8 *data, usize size) override;
  void Read(u8 *dst, usize size) override;
  void ReadWrite(const u8 *tx_data, u8 *rx_buffer, usize size) override;

 private:
  SPI_HandleTypeDef *hspi_{nullptr};
  usize transmission_timeout_{};
};

}  // namespace rm::hal::stm32

#endif

#endif  // LIBRM_HAL_STM32_SPI_HPP
