/*
  Copyright (c) 2026 XDU-IRobot

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
 * @file  librm/hal/stm32/throttled_bxcan.hpp
 * @brief 基于限流优先级队列的bxCAN发送封装
 */

#ifndef LIBRM_HAL_STM32_THROTTLED_BXCAN_HPP
#define LIBRM_HAL_STM32_THROTTLED_BXCAN_HPP

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_CAN_MODULE_ENABLED)

#include "librm/hal/stm32/bxcan.hpp"
#include "librm/modules/throttled_prio_queue.hpp"

namespace rm::hal::stm32 {

/**
 * @brief  基于限流优先级队列的bxCAN发送封装
 * @details 继承自 BxCan，override Write() 将帧入队而非立即发送。
 *          需要在主循环或定时器中尽量高频地调用 Process() 以按限频策略出队并实际发送。
 * @tparam MaxQueueSize 发送队列最大深度
 */
template <size_t MaxQueueSize = 128>
class ThrottledBxCan final : public BxCan {
 public:
  using clock = std::chrono::steady_clock;
  using time_point = clock::time_point;
  using duration = clock::duration;

  /**
   * @param hcan                     HAL库的CAN_HandleTypeDef
   * @param tx_frequency_hz          发送频率上限（Hz），用于限频
   * @param default_deadline_offset  默认截止时间偏移量，Write() override使用
   */
  explicit ThrottledBxCan(CAN_HandleTypeDef &hcan, double tx_frequency_hz,
                          duration default_deadline_offset = std::chrono::milliseconds(50))
      : BxCan(hcan), queue_(tx_frequency_hz), default_deadline_offset_(default_deadline_offset) {}

  ~ThrottledBxCan() override = default;

  /**
   * @brief 将数据帧入队（使用默认优先级和默认截止时间）
   * @note  不会立即发送，需要定期调用 Process() 以实际发送
   * @param id    标准帧ID
   * @param data  数据指针
   * @param size  数据长度
   */
  void Write(u16 id, const u8 *data, usize size) override {
    Write(id, data, size, kDefaultPriority, clock::now() + default_deadline_offset_);
  }

  /**
   * @brief 将数据帧入队（指定优先级和截止时间点）
   * @param id        标准帧ID
   * @param data      数据指针
   * @param size      数据长度
   * @param priority  软件优先级（数值越大，优先级越高）
   * @param deadline  绝对截止时间点，超过此时间未发送则丢弃
   * @return true 入队成功, false 队列已满
   */
  bool Write(u16 id, const u8 *data, usize size, u8 priority, time_point deadline) {
    TxRequest req;
    req.id = id;
    req.size = (size <= 8) ? size : 8;
    std::copy_n(data, req.size, req.data.begin());
    return queue_.Push(req, priority, deadline);
  }

  /**
   * @brief 处理发送队列，取出一帧并通过底层BxCan实际发送（自动获取当前时间）
   * @note  需要在主循环或定时器任务中定期调用
   * @return true 成功发送了一帧, false 队列为空/未到发送间隔/消息已过期
   */
  bool Process() {
    auto result = queue_.Process();
    if (result.has_value()) {
      const auto &req = result.value();
      BxCan::Write(req.id, req.data.data(), req.size);
      return true;
    }
    return false;
  }

  /**
   * @brief 处理发送队列，取出一帧并通过底层BxCan实际发送（使用指定时间点）
   * @param now 当前时间点
   * @return true 成功发送了一帧, false 队列为空/未到发送间隔/消息已过期
   */
  bool Process(time_point now) {
    auto result = queue_.Process(now);
    if (result.has_value()) {
      const auto &req = result.value();
      BxCan::Write(req.id, req.data.data(), req.size);
      return true;
    }
    return false;
  }

  /**
   * @brief 停止CAN外设并清空发送队列
   */
  void Stop() override {
    BxCan::Stop();
    queue_.Clear();
  }

  const auto &queue() const { return queue_; }

 private:
  struct TxRequest {
    u16 id;
    std::array<u8, 8> data;
    usize size;
  };

  static constexpr u8 kDefaultPriority = 128;

  modules::ThrottledPrioQueue<TxRequest, MaxQueueSize> queue_;
  duration default_deadline_offset_;
};

}  // namespace rm::hal::stm32

#endif

#endif  // LIBRM_HAL_STM32_THROTTLED_BXCAN_HPP
