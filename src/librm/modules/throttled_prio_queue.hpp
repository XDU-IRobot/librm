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
 * @brief 限流优先级调度队列
 */

#ifndef LIBRM_MODULES_THROTTLED_PRIO_QUEUE_HPP
#define LIBRM_MODULES_THROTTLED_PRIO_QUEUE_HPP

#include <chrono>
#include <optional>
#include <type_traits>

#include <etl/priority_queue.h>

#include "librm/core/typedefs.hpp"

namespace rm::modules {

/**
 * @brief 调度策略
 */
enum class SchedulingPolicy {
  kEdf,   ///< Earliest Deadline First：优先级数值越大越先出队；同优先级时 deadline 越早越先出队
  kFifo,  ///< 严格 FIFO：完全按入队顺序出队（priority 参数被忽略）；deadline 过期仍然丢弃
  kPriorityFifo,  ///< 优先级 + FIFO：优先级高的先出队；同优先级时严格按入队顺序出队
};

/**
 * @brief 限流优先级调度队列，用于处理CAN总线定频发送以及类似逻辑
 * @tparam T               负载数据类型 (如CanFrame)
 * @tparam MaxQueueSize    队列最大深度
 * @tparam Policy          调度策略，默认 kFifo
 */
template <typename T, usize MaxQueueSize, SchedulingPolicy Policy = SchedulingPolicy::kPriorityFifo>
class ThrottledPrioQueue {
 public:
  using clock = std::chrono::steady_clock;
  using time_point = clock::time_point;
  using duration = clock::duration;

  struct QueueItem {
    T payload;
    u8 priority;           ///< EDF 模式下的软件优先级（数值越大越高）；FIFO 模式下忽略
    time_point deadline;   ///< 绝对截止时间点，超过此时间未发送则丢弃
    usize enqueue_seq{0};  ///< 入队序号，FIFO 模式下用于保证时序

    bool operator<(const QueueItem& other) const {
      // 使用有符号差值比较序号，正确处理 enqueue_seq 上溢回绕
      // 只要队列中同时存在的最大序号跨度 < usize 范围的一半，此方法即可正确工作
      using signed_seq = std::make_signed_t<usize>;
      const auto seq_after = [](usize a, usize b) -> bool { return static_cast<signed_seq>(a - b) > 0; };

      if constexpr (Policy == SchedulingPolicy::kFifo) {
        // FIFO：序号越小（越早入队）越应先出队
        // 大顶堆中返回 true 表示 other 优先，故序号较晚（seq_after）的返回 true
        return seq_after(enqueue_seq, other.enqueue_seq);
      } else if constexpr (Policy == SchedulingPolicy::kPriorityFifo) {
        // 优先级 + FIFO：优先级高的先出；同优先级时序号小的（早入队的）先出
        if (priority == other.priority) {
          return seq_after(enqueue_seq, other.enqueue_seq);
        }
        return priority < other.priority;
      } else {
        // EDF：优先级数值大的先出；同优先级时 deadline 早的先出
        if (priority == other.priority) {
          return deadline > other.deadline;
        }
        return priority < other.priority;
      }
    }
  };

  /**
   * @param tx_frequency_hz 处理/发送的频率上限（Hz），用于限频
   */
  explicit ThrottledPrioQueue(double tx_frequency_hz)
      : interval_(std::chrono::duration_cast<duration>(std::chrono::duration<double>(1.0 / tx_frequency_hz))) {}

  /**
   * @brief 数据入队
   * @param payload  业务数据
   * @param priority 优先级（kFifo 模式下忽略此参数）
   * @param deadline 绝对截止时间点 (如：clock::now() + 50ms)
   * @return true 入队成功, false 队列已满
   */
  bool Push(const T& payload, u8 priority, time_point deadline) {
    if (queue_.full()) {
      return false;
    }
    queue_.push({payload, priority, deadline, enqueue_seq_++});
    return true;
  }

  /**
   * @brief 尝试从队列中提取一个待处理的消息（自动获取当前时间）
   * @return std::optional<T> 如果满足限流条件且有有效消息，返回消息内容；否则返回 std::nullopt
   */
  std::optional<T> Process() { return Process(clock::now()); }

  /**
   * @brief 尝试从队列中提取一个待处理的消息
   * @param now 当前时间点
   * @return std::optional<T> 如果满足限流条件且有有效消息，返回消息内容；否则返回 std::nullopt
   */
  std::optional<T> Process(time_point now) {
    // 1. 清理已超时的过期消息，精确计数
    while (!queue_.empty()) {
      if (now >= queue_.top().deadline) {
        queue_.pop();
        ++expired_count_;
      } else {
        break;
      }
    }

    // 2. 检查队列是否为空或是否未到限频间隔
    if (queue_.empty() || (now - last_process_time_ < interval_)) {
      return std::nullopt;
    }

    // 3. 提取消息并更新状态
    T payload = queue_.top().payload;
    queue_.pop();
    last_process_time_ = now;

    return payload;
  }

  /// @brief 获取当前队列中的消息数
  usize size() const { return queue_.size(); }
  /// @brief 队列是否为空
  bool empty() const { return queue_.empty(); }
  /// @brief 累计过期丢弃帧数（单调递增）
  usize expired_count() const { return expired_count_; }
  void Clear() {
    while (!queue_.empty()) {
      queue_.pop();
    }
  }

 private:
  etl::priority_queue<QueueItem, MaxQueueSize> queue_;
  duration interval_;               ///< 定频发送周期
  time_point last_process_time_{};  ///< 上一次成功处理的时间点
  usize expired_count_{0};          ///< 累计过期丢弃帧数（单调递增）
  usize enqueue_seq_{0};            ///< 入队序号计数器（FIFO 模式使用）
};
}  // namespace rm::modules

#endif  // LIBRM_MODULES_THROTTLED_PRIO_QUEUE_HPP
