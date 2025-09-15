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
 * @todo  部分完成
 */

#ifndef LIBRM_HAL_STM32_FDCAN_H
#define LIBRM_HAL_STM32_FDCAN_H

#include "librm/hal/stm32/hal.h"
#if defined(HAL_FDCAN_MODULE_ENABLED)

#include <unordered_map>
#include <deque>
#include <memory>

#include "librm/hal/can_interface.h"
#include "librm/device/can_device.hpp"

namespace rm::hal::stm32 {

class FdCan : public CanInterface {
 public:
  explicit FdCan(FDCAN_HandleTypeDef &hfdcan);

  FdCan() = default;
  ~FdCan() override = default;

  // 禁止拷贝构造
  FdCan(const FdCan &) = delete;
  FdCan &operator=(const FdCan &) = delete;

  void SetFilter(u16 id, u16 mask) override;
  void Write(u16 id, const u8 *data, usize size) override;
  void Write() override;
  void Enqueue(u16 id, const u8 *data, usize size, CanTxPriority priority) override;
  void Begin() override;
  void Stop() override;

 private:
  void Fifo0MsgPendingCallback();

  u32 tx_mailbox_{0};
  CanMsg rx_buffer_{};
  std::unordered_map<CanTxPriority, std::deque<std::shared_ptr<CanMsg>>> tx_queue_{
      {CanTxPriority::kHigh, {}},
      {CanTxPriority::kNormal, {}},
      {CanTxPriority::kLow, {}},
  };
  FDCAN_HandleTypeDef *hfdcan_{nullptr};
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

  /**
   * @brief 消息队列最大长度
   * @note  在插入或发送消息的时候，如果检测到消息队列长度超过这个值，就会清空消息队列（待发送的消息都会被丢弃）
   * @note  触发清空队列的动作意味着插入消息的速度大于发送消息的速度，应该减少发送消息的数量
   */
  static constexpr usize kQueueMaxSize = 100;
};

}  // namespace rm::hal::stm32

#endif

#endif  // LIBRM_HAL_STM32_FDCAN_H