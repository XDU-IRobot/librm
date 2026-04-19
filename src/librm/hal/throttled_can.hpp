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
 * @file  librm/hal/throttled_can.hpp
 * @brief 基于限流优先级队列的通用CAN发送封装（平台无关）
 * @details 通过模板参数 Base 接受任意 CanInterface 子类（BxCan / FdCan / Mcp2515 / SocketCan 等），
 *          将throttle逻辑（入队、限频出队、流量统计）统一实现，避免代码重复。
 */

#ifndef LIBRM_HAL_THROTTLED_CAN_HPP
#define LIBRM_HAL_THROTTLED_CAN_HPP

#include <chrono>
#include <array>
#include <algorithm>

#include <etl/pseudo_moving_average.h>

#include "librm/core/typedefs.hpp"
#include "librm/modules/throttled_prio_queue.hpp"

namespace rm::hal::detail {

/**
 * @brief  基于限流优先级队列的通用CAN发送封装
 * @details 继承自 Base（任意 CanInterface 子类），override Write() 将帧入队而非立即发送。
 *          需要在主循环或定时器中尽量高频地调用 Process() 以按限频策略出队并实际发送。
 * @tparam Base         底层CAN实现类（BxCan / FdCan / Mcp2515 / SocketCan 等）
 * @tparam MaxDataSize  单帧最大数据长度（经典CAN=8, CAN FD=64）
 * @tparam MaxQueueSize 发送队列最大深度
 * @tparam Policy       调度策略，默认 kPriorityFifo（严格先入先出）；kEdf 为优先级+最早截止时间优先
 */
template <typename Base, usize MaxDataSize = 8, usize MaxQueueSize = 128,
          modules::SchedulingPolicy Policy = modules::SchedulingPolicy::kPriorityFifo>
class ThrottledCan : public Base {
 public:
  using clock = std::chrono::steady_clock;
  using time_point = clock::time_point;
  using duration = clock::duration;

  /**
   * @brief 流量统计快照（每秒刷新一次）
   */
  struct TxStats {
    f32 tx_fps{0.0f};            ///< 实际发送帧率（帧/秒）
    f32 enqueue_fps{0.0f};       ///< 入队帧率（帧/秒，含成功+被丢弃的）
    f32 drop_full_fps{0.0f};     ///< 因队列满被丢弃的帧率（帧/秒）
    f32 drop_expired_fps{0.0f};  ///< 因超过 deadline 被丢弃的帧率（帧/秒）
    f32 drop_total_fps{0.0f};    ///< 总丢弃帧率（帧/秒）
    usize peak_queue_depth{0};   ///< 统计周期内队列深度峰值
    f32 avg_queue_depth{0.0f};   ///< 统计周期内队列深度均值
  };

  /**
   * @brief 构造函数，将底层CAN构造参数透传给Base
   * @param tx_frequency_hz          发送频率上限（Hz），用于限频
   * @param base_args                转发给Base构造函数的参数
   */
  template <typename... BaseArgs>
  explicit ThrottledCan(double tx_frequency_hz, BaseArgs &&...base_args)
      : Base(std::forward<BaseArgs>(base_args)...),
        queue_(tx_frequency_hz),
        default_deadline_offset_(std::chrono::milliseconds(50)) {}

  ~ThrottledCan() override = default;

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
    req.size = (size <= MaxDataSize) ? size : MaxDataSize;
    std::copy_n(data, req.size, req.data.begin());
    ++stat_window_data_.enqueue_count;
    if (!queue_.Push(req, priority, deadline)) {
      ++stat_window_data_.drop_full_count;
      return false;
    }
    return true;
  }

  /**
   * @brief 处理发送队列，取出一帧并通过底层CAN实际发送（自动获取当前时间）
   * @note  需要在主循环或定时器任务中定期调用
   * @return true 成功发送了一帧, false 队列为空/未到发送间隔/消息已过期
   */
  bool Process() { return Process(clock::now()); }

  /**
   * @brief 处理发送队列，取出一帧并通过底层CAN实际发送（使用指定时间点）
   * @param now 当前时间点
   * @return true 成功发送了一帧, false 队列为空/未到发送间隔/消息已过期
   */
  bool Process(time_point now) {
    // 采样队列深度
    const usize depth = queue_.size();
    depth_avg_.add(static_cast<f32>(depth));
    if (depth > stat_window_data_.peak_depth) {
      stat_window_data_.peak_depth = depth;
    }

    auto result = queue_.Process(now);
    const bool sent = result.has_value();
    if (sent) {
      const auto &req = result.value();
      Base::Write(req.id, req.data.data(), req.size);
      ++stat_window_data_.tx_count;
    }

    // 检查是否到了 1 秒刷新窗口
    if (now - stats_window_start_ >= std::chrono::seconds(1)) {
      const f32 elapsed_s = std::chrono::duration<f32>(now - stats_window_start_).count();

      const usize expired_delta = queue_.expired_count() - stat_window_data_.expired_baseline;

      // clang-format off
      TxStats snap;
      snap.tx_fps           = static_cast<f32>(stat_window_data_.tx_count)        / elapsed_s;
      snap.enqueue_fps      = static_cast<f32>(stat_window_data_.enqueue_count)   / elapsed_s;
      snap.drop_full_fps    = static_cast<f32>(stat_window_data_.drop_full_count) / elapsed_s;
      snap.drop_expired_fps = static_cast<f32>(expired_delta)                     / elapsed_s;
      snap.drop_total_fps   = snap.drop_full_fps + snap.drop_expired_fps;
      snap.peak_queue_depth = stat_window_data_.peak_depth;
      snap.avg_queue_depth  = depth_avg_.value();
      stats_snapshot_ = snap;

      // 重置窗口
      stats_window_start_                  = now;
      stat_window_data_.tx_count           = 0;
      stat_window_data_.enqueue_count      = 0;
      stat_window_data_.drop_full_count    = 0;
      stat_window_data_.expired_baseline   = queue_.expired_count();
      stat_window_data_.peak_depth         = 0;
      depth_avg_.clear(0.0f);
      // clang-format on
    }

    return sent;
  }

  /**
   * @brief 停止CAN外设并清空发送队列
   */
  void Stop() override {
    Base::Stop();
    queue_.Clear();
  }

  const auto &queue() const { return queue_; }

  /// @brief 获取最近一个统计周期（1 秒）的流量快照（Process() 驱动刷新）
  const TxStats &stats() const { return stats_snapshot_; }

 private:
  struct TxRequest {
    u16 id;
    std::array<u8, MaxDataSize> data;
    usize size;
  };

  static constexpr u8 kDefaultPriority = 128;

  modules::ThrottledPrioQueue<TxRequest, MaxQueueSize, Policy> queue_;
  duration default_deadline_offset_;

  // 流量统计
  TxStats stats_snapshot_{};                             ///< 对外快照，每 1 秒更新一次
  time_point stats_window_start_{clock::now()};          ///< 当前统计窗口起点
  etl::pseudo_moving_average<f32, 16> depth_avg_{0.0f};  ///< 队列深度滑动平均（窗口 16 个采样点）

  // 窗口内原始累计计数
  struct {
    usize tx_count{0};          ///< 成功发送帧数
    usize enqueue_count{0};     ///< 入队尝试次数（成功+失败）
    usize drop_full_count{0};   ///< 因队满丢弃帧数
    usize expired_baseline{0};  ///< 窗口起点时 queue_.expired_count() 的基准值
    usize peak_depth{0};        ///< 窗口内队列深度峰值
  } stat_window_data_;
};

}  // namespace rm::hal::detail

#endif  // LIBRM_HAL_THROTTLED_CAN_HPP
