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
 * @file  librm/hal/stm32/bxcan.h
 * @brief bxCAN类库
 */

#ifndef LIBRM_HAL_STM32_BXCAN_H
#define LIBRM_HAL_STM32_BXCAN_H

#include "librm/hal/stm32/hal.h"
#if defined(HAL_CAN_MODULE_ENABLED)

#include <unordered_map>
#include <deque>
#include <memory>

#include "librm/hal/can_interface.h"
#include "librm/device/can_device.hpp"

namespace rm::hal::stm32 {

/**
 * @brief bxCAN类库
 */
class BxCan final : public CanInterface {
 public:
  explicit BxCan(CAN_HandleTypeDef &hcan);
  BxCan() = default;
  ~BxCan() override = default;

  // 禁止拷贝构造
  BxCan(const BxCan &) = delete;
  BxCan &operator=(const BxCan &) = delete;

  void SetFilter(u16 id, u16 mask) override;
  void Write(u16 id, const u8 *data, usize size) override;
  void Write() override;
  void Enqueue(u16 id, const u8 *data, usize size, CanTxPriority priority) override;
  void Begin() override;
  void Stop() override;

 private:
  void RegisterDevice(device::CanDevice &device, u32 rx_stdid) override;
  void Fifo0MsgPendingCallback();

  u32 tx_mailbox_{0};
  CanMsg rx_buffer_{};
  std::unordered_map<CanTxPriority, std::deque<std::shared_ptr<CanMsg>>> tx_queue_{
      {CanTxPriority::kHigh, {}},
      {CanTxPriority::kNormal, {}},
      {CanTxPriority::kLow, {}},
  };  // <priority, queue>
  CAN_HandleTypeDef *hcan_{nullptr};
  CAN_TxHeaderTypeDef hal_tx_header_ = {
      .StdId = 0,
      .ExtId = 0,
      .IDE = CAN_ID_STD,
      .RTR = CAN_RTR_DATA,
      .DLC = 0,
      .TransmitGlobalTime = DISABLE,
  };
  std::unordered_map<u16, device::CanDevice *> device_list_{};  // <rx_stdid, device>

  /**
   * @brief 消息队列最大长度
   * @note  在插入或发送消息的时候，如果检测到消息队列长度超过这个值，就会清空消息队列（待发送的消息都会被丢弃）
   * @note  触发清空队列的动作意味着插入消息的速度大于发送消息的速度，应该减少发送消息的数量
   */
  static constexpr usize kQueueMaxSize = 100;
};

}  // namespace rm::hal::stm32

#endif

#endif  // LIBRM_HAL_STM32_BXCAN_H
