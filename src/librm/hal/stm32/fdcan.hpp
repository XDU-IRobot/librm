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
 * @file  librm/hal/stm32/fdcan.h
 * @brief fdCAN类库
 */

#ifndef LIBRM_HAL_STM32_FDCAN_HPP
#define LIBRM_HAL_STM32_FDCAN_HPP

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_FDCAN_MODULE_ENABLED)

#include "librm/core/traits.hpp"
#include "librm/hal/can_interface.hpp"
#include "librm/device/can_device.hpp"

namespace rm::hal::stm32 {

class FdCan : public CanInterface, public detail::NonCopyable {
 public:
  explicit FdCan(FDCAN_HandleTypeDef &hfdcan);

  FdCan() = default;
  ~FdCan() override = default;

  void SetFilter(u16 id, u16 mask) override;
  void Write(u16 id, const u8 *data, usize size) override;
  void Begin() override;
  void Stop() override;

 private:
  void Fifo0MsgPendingCallback();

  u32 tx_mailbox_{0};
  CanFrame rx_buffer_{};
  FDCAN_HandleTypeDef *hfdcan_{nullptr};
  const bool fd_mode_{false};
  FDCAN_TxHeaderTypeDef hal_tx_header_ = {
      0,
      FDCAN_STANDARD_ID,
      FDCAN_DATA_FRAME,
      FDCAN_DLC_BYTES_0,
      FDCAN_ESI_PASSIVE,
      FDCAN_BRS_OFF,
      FDCAN_CLASSIC_CAN,
      FDCAN_STORE_TX_EVENTS,
      0,
  };
};

}  // namespace rm::hal::stm32

#endif

#endif  // LIBRM_HAL_STM32_FDCAN_H