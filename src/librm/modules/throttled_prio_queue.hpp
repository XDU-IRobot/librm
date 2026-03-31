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
 * @file  librm/modules/throttled_prio_queue.hpp
 * @brief 轨迹限制器
 */

#ifndef LIBRM_MODULES_THROTTLED_PRIO_QUEUE_HPP
#define LIBRM_MODULES_THROTTLED_PRIO_QUEUE_HPP

#include <optional>

#include <etl/priority_queue.h>

#include "librm/core/typedefs.hpp"

namespace rm::modules {

/**
 * @brief 限流优先级调度队列，用于处理CAN总线定频发送以及类似逻辑
 * @tparam T 负载数据类型 (如CanFrame)
 * @tparam MaxQueueSize 队列最大深度
 */
template <typename T, size_t MaxQueueSize>
class ThrottledPrioQueue {
 public:
  struct QueueItem {
    T payload;
    u8 priority;      ///< 软件优先级 (数值越大，优先级越高)
    u32 deadline_ms;  ///< 绝对截止时间戳，超过此时间未发送则丢弃

    // etl::priority_queue 默认是大顶堆，通过重载 < 决定谁在堆顶
    bool operator<(const QueueItem& other) const {
      if (priority == other.priority) {
        // 优先级相同时，deadline 越早 (数值越小)，越需要优先处理
        // 因为是大顶堆，返回 true 会让 other 浮到堆顶
        return deadline_ms > other.deadline_ms;
      }
      return priority < other.priority;  // 优先级高的在堆顶
    }
  };

  /**
   * @param tx_interval_ms 处理/发送的最小间隔时间（毫秒），用于限频
   */
  explicit ThrottledPrioQueue(u32 tx_interval_ms) : interval_ms_(tx_interval_ms) {}

  /**
   * @brief 数据入队
   * @param payload 业务数据
   * @param priority 优先级
   * @param deadline_ms 绝对截止时间戳 (如：当前时间 + 50ms)
   * @return true 入队成功, false 队列已满
   */
  bool Push(const T& payload, u8 priority, u32 deadline_ms) {
    if (queue_.full()) {
      return false;
    }
    queue_.push({payload, priority, deadline_ms});
    return true;
  }

  /**
   * @brief 尝试从队列中提取一个待处理的消息
   * @param current_time_ms 当前系统时间戳
   * @return std::optional<T> 如果满足限流条件且有有效消息，返回消息内容；否则返回 std::nullopt
   */
  std::optional<T> Process(u32 current_time_ms) {
    // 1. 清理已超时的过期消息
    while (!queue_.empty()) {
      if (current_time_ms >= queue_.top().deadline_ms) {
        queue_.pop();
      } else {
        break;
      }
    }

    // 2. 检查队列是否为空或是否未到限频间隔
    if (queue_.empty() || (current_time_ms - last_process_time_ < interval_ms_)) {
      return std::nullopt;
    }

    // 3. 提取消息并更新状态
    T payload = queue_.top().payload;
    queue_.pop();
    last_process_time_ = current_time_ms;

    return payload;
  }

  // 实用接口
  size_t size() const { return queue_.size(); }
  bool empty() const { return queue_.empty(); }
  void Clear() {
    while (!queue_.empty()) {
      queue_.pop();
    }
  }

 private:
  etl::priority_queue<QueueItem, MaxQueueSize> queue_;
  u32 interval_ms_;           ///< 定频发送周期
  u32 last_process_time_{0};  ///< 上一次成功处理的时间戳
};
}  // namespace rm::modules

#endif  // LIBRM_MODULES_THROTTLED_PRIO_QUEUE_HPP