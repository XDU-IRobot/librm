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
 * @file  librm/hal/stm32/uart.h
 * @brief UART类库
 */

#ifndef LIBRM_HAL_STM32_UART_H
#define LIBRM_HAL_STM32_UART_H

#include "librm/hal/stm32/hal.h"
#if defined(HAL_UART_MODULE_ENABLED)

#include "librm/hal/serial_interface.h"
#include "librm/core/typedefs.h"

#include <unordered_map>

namespace rm::hal::stm32 {

enum class UartMode {
  kNormal,
  kInterrupt,
#if defined(HAL_DMA_MODULE_ENABLED)
  kDma,
#endif
};

/**
 * @brief UART类
 */
class Uart : public SerialInterface {
 public:
  Uart(UART_HandleTypeDef &huart, usize rx_buffer_size, UartMode tx_mode = UartMode::kNormal,
       UartMode rx_mode = UartMode::kNormal);

  void Begin() override;
  void Write(const u8 *data, usize size) override;
  void AttachRxCallback(SerialRxCallbackFunction &callback) override;
  [[nodiscard]] const std::vector<u8> &rx_buffer() const override;

 private:
  void HalRxCpltCallback(u16 rx_len);
  void HalErrorCallback();

  std::vector<SerialRxCallbackFunction *> rx_callbacks_;
  UART_HandleTypeDef *huart_;
  UartMode tx_mode_;
  UartMode rx_mode_;
  std::vector<u8> rx_buf_[2];
  bool buffer_selector_{false};
};

}  // namespace rm::hal::stm32

#endif
#endif  // LIBRM_HAL_STM32_UART_H
