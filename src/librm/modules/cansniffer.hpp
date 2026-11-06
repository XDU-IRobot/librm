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
 * @file  librm/modules/cansniffer.hpp
 * @brief CAN总线报文嗅探器
 */

#ifndef LIBRM_MODULES_CANSNIFFER_HPP
#define LIBRM_MODULES_CANSNIFFER_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <optional>

#include <etl/vector.h>

#include "librm/core/typedefs.hpp"
#include "librm/hal/can_interface.hpp"

namespace rm::modules {

/**
 * @brief 单个CAN ID的统计结果
 */
struct CanIdStats {
  u16 id{0};            ///< 标准帧ID
  u32 frame_count{0};   ///< 累计收到的帧数
  f32 average_hz{0.0f}; ///< 平均频率，窗口内 (n-1)/(t_last-t_first)；样本不足时为0
  f32 stddev_hz{0.0f};  ///< 窗口内周期抖动的标准差，按 sigma_rate=sigma_period/mean_period^2 换算
  u8 dlc{0};            ///< 最近一帧的数据长度（字节）
  bool is_fd_frame{false};  ///< 最近一帧是否为FD帧
};

/**
 * @brief CAN总线报文嗅探器
 * @details 挂在CAN总线上统计各ID的活跃情况和出现频率，功能类似can-utils的cansniffer，
 *          但不打印终端表格，而是提供查询接口供应用读取。
 * @tparam MaxIds           最多同时统计的ID数量
 * @tparam WindowSize       每个ID保留的时间戳样本数，越大统计越平稳
 * @tparam MaxPendingFrames 中断与任务之间缓冲的帧数
 * @note  用法：
 * @note  1. 构造时传入CAN总线对象，自动注册为总线观察者；
 * @note  2. 在任务上下文中周期调用Update()，把中断里收到的报文并入统计；
 * @note  3. 通过active_ids()查询最近活跃的ID，通过stats()查询频率统计。
 * @note  总线回调只把报文压入无锁环形队列，统计全部在Update()里完成，
 *        因此Update()的调用频率决定了队列是否会溢出（见dropped_frames()）。
 * @note  时间戳在总线回调里取，CAN控制器一次中断批量取出多帧时，同批帧的时间差只是读寄存器的耗时，
 *        不是总线上的真实间隔，这会让窗口内的平均频率略微偏高、标准差偏大。
 * @note  CAN外设的硬件过滤器需要放行想观察的报文，否则统计不到。
 */
template <usize MaxIds = 32, usize WindowSize = 16, usize MaxPendingFrames = 32>
class CanSniffer : public hal::CanFrameObserver {
  static_assert(MaxIds > 0, "MaxIds must be greater than 0");
  static_assert(WindowSize > 0, "WindowSize must be greater than 0");
  static_assert(MaxPendingFrames > 0, "MaxPendingFrames must be greater than 0");

 public:
  using clock = std::chrono::steady_clock;
  using duration = clock::duration;
  using StatsList = etl::vector<CanIdStats, MaxIds>;
  using IdList = etl::vector<u16, MaxIds>;

  /**
   * @param can CAN外设对象
   */
  explicit CanSniffer(hal::CanInterface &can) : can_(&can) { can_->add_observer(*this); }

  ~CanSniffer() override { can_->remove_observer(*this); }

  CanSniffer(const CanSniffer &) = delete;
  CanSniffer &operator=(const CanSniffer &) = delete;

  /**
   * @brief 处理总线回调缓冲的报文，更新统计数据
   * @note  需要在任务上下文中周期调用，不要在中断里调用
   */
  void Update() {
    while (true) {
      const u32 tail = tail_.load(std::memory_order_relaxed);
      if (tail == head_.load(std::memory_order_acquire)) {
        break;
      }
      const PendingFrame frame = pending_[tail];
      tail_.store((tail + 1) % kPendingCapacity, std::memory_order_release);
      Accumulate(frame);
    }
  }

  /**
   * @brief 获取所有已统计ID的数据
   * @return 按ID升序排列的统计结果
   */
  [[nodiscard]] StatsList stats() const {
    StatsList result;
    for (const auto &entry : entries_) {
      result.push_back(MakeStats(entry));
    }
    return result;
  }

  /**
   * @brief 获取单个ID的统计数据
   * @param id 标准帧ID
   * @return 该ID尚未被统计到时返回std::nullopt
   */
  [[nodiscard]] std::optional<CanIdStats> stats(u16 id) const {
    for (const auto &entry : entries_) {
      if (entry.id == id) {
        return MakeStats(entry);
      }
    }
    return std::nullopt;
  }

  /**
   * @brief 获取最近一段时间内活跃的ID
   * @param timeout 判定活跃的时间窗口
   * @return 按ID升序排列的活跃ID列表
   */
  [[nodiscard]] IdList active_ids(duration timeout = std::chrono::seconds(1)) const {
    IdList ids;
    const u32 now = CurrentTimeUs();
    const u32 timeout_us = TimeoutToUs(timeout);
    for (const auto &entry : entries_) {
      if (entry.frame_count > 0 && (now - entry.last_timestamp_us) <= timeout_us) {
        ids.push_back(entry.id);
      }
    }
    return ids;
  }

  /**
   * @brief 获取未能计入统计的帧数
   * @note  包含两种情况：中断缓冲队列已满，或ID数量超过MaxIds
   */
  [[nodiscard]] u32 dropped_frames() const { return dropped_frames_.load(std::memory_order_relaxed); }

  /**
   * @brief 总线观察者入口，由CAN总线在接收上下文调用，不要手动调用
   * @param msg 收到的报文
   */
  void notification(const hal::CanFrame &msg) override {
    const u32 head = head_.load(std::memory_order_relaxed);
    const u32 next = (head + 1) % kPendingCapacity;
    if (next == tail_.load(std::memory_order_acquire)) {
      dropped_frames_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    PendingFrame &frame = pending_[head];
    frame.timestamp_us = CurrentTimeUs();
    frame.id = msg.std_id;
    frame.dlc = static_cast<u8>(msg.dlc);
    frame.is_fd_frame = msg.is_fd_frame;
    head_.store(next, std::memory_order_release);
  }

 private:
  static constexpr usize kPendingCapacity = MaxPendingFrames + 1;

  struct PendingFrame {
    u32 timestamp_us{0};
    u16 id{0};
    u8 dlc{0};
    bool is_fd_frame{false};
  };

  struct Entry {
    u16 id{0};
    u8 dlc{0};
    bool is_fd_frame{false};
    u32 frame_count{0};
    u32 last_timestamp_us{0};
    usize sample_count{0};
    usize next_index{0};
    std::array<u32, WindowSize> timestamps_us{};
  };

  /**
   * @brief 把一帧的到达时间并入统计
   * @param frame 待处理的报文
   * @note  时间戳来自gettimeofday合成的steady_clock，在中断里读到毫秒和微秒余量不一致时可能回退1ms，
   *        这种样本直接丢弃并保留原基准，否则u32相减会得到极大的周期，把平均值和标准差全带偏。
   */
  void Accumulate(const PendingFrame &frame) {
    Entry *entry = FindOrCreate(frame.id);
    if (entry == nullptr) {
      dropped_frames_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    if (entry->frame_count == 0) {
      PushTimestamp(*entry, frame.timestamp_us);
      entry->last_timestamp_us = frame.timestamp_us;
    } else if (static_cast<i32>(frame.timestamp_us - entry->last_timestamp_us) > 0) {
      PushTimestamp(*entry, frame.timestamp_us);
      entry->last_timestamp_us = frame.timestamp_us;
    }

    entry->dlc = frame.dlc;
    entry->is_fd_frame = frame.is_fd_frame;
    ++entry->frame_count;
  }

  /**
   * @brief 查找ID对应的统计项，不存在时按ID升序插入
   * @return 统计项指针；ID数量已满时返回nullptr
   */
  Entry *FindOrCreate(u16 id) {
    usize pos = 0;
    while (pos < entries_.size() && entries_[pos].id < id) {
      ++pos;
    }
    if (pos < entries_.size() && entries_[pos].id == id) {
      return &entries_[pos];
    }
    if (entries_.full()) {
      return nullptr;
    }

    entries_.push_back(Entry{});
    for (usize i = entries_.size() - 1; i > pos; --i) {
      entries_[i] = entries_[i - 1];
    }
    entries_[pos].id = id;
    return &entries_[pos];
  }

  /**
   * @brief 取出窗口内第index个时间戳，index为0时是最早的样本
   */
  static u32 TimestampAt(const Entry &entry, usize index) {
    if (entry.sample_count < WindowSize) {
      return entry.timestamps_us[index];
    }
    return entry.timestamps_us[(entry.next_index + index) % WindowSize];
  }

  /**
   * @brief 把一个时间戳压入窗口
   */
  static void PushTimestamp(Entry &entry, u32 timestamp_us) {
    entry.timestamps_us[entry.next_index] = timestamp_us;
    entry.next_index = (entry.next_index + 1) % WindowSize;
    if (entry.sample_count < WindowSize) {
      ++entry.sample_count;
    }
  }

  /**
   * @brief 由统计项计算对外的统计结果
   * @note  平均频率用窗口首尾时间戳计算，即 (n-1)/(t_last-t_first)，和ros2 topic hz的做法一致，
   *        避免调和平均被单个长周期拉偏；标准差按 sigma_rate = sigma_period / mean_period^2 传播。
   */
  static CanIdStats MakeStats(const Entry &entry) {
    CanIdStats result;
    result.id = entry.id;
    result.frame_count = entry.frame_count;
    result.dlc = entry.dlc;
    result.is_fd_frame = entry.is_fd_frame;
    if (entry.sample_count < 2) {
      return result;
    }

    const usize count = entry.sample_count;
    const u32 span_us = TimestampAt(entry, count - 1) - TimestampAt(entry, 0);
    if (span_us == 0) {
      return result;
    }

    const f32 mean_period_us = static_cast<f32>(span_us) / static_cast<f32>(count - 1);
    result.average_hz = static_cast<f32>(count - 1) * 1e6f / static_cast<f32>(span_us);

    f32 squared_deviation_sum = 0.0f;
    for (usize i = 1; i < count; ++i) {
      const f32 period_us = static_cast<f32>(TimestampAt(entry, i) - TimestampAt(entry, i - 1));
      const f32 deviation = period_us - mean_period_us;
      squared_deviation_sum += deviation * deviation;
    }
    const f32 stddev_period_us = std::sqrt(squared_deviation_sum / static_cast<f32>(count - 1));
    result.stddev_hz = stddev_period_us * 1e6f / (mean_period_us * mean_period_us);
    return result;
  }

  /**
   * @brief 获取当前时间，单位为微秒，截断到32位
   */
  static u32 CurrentTimeUs() {
    return static_cast<u32>(
        std::chrono::duration_cast<std::chrono::microseconds>(clock::now().time_since_epoch()).count());
  }

  /**
   * @brief 把时间窗口换算成微秒并限制在u32范围内
   */
  static u32 TimeoutToUs(duration timeout) {
    if (timeout <= duration::zero()) {
      return 0;
    }
    const auto timeout_us = std::chrono::duration_cast<std::chrono::microseconds>(timeout).count();
    return timeout_us > static_cast<i64>(0xFFFFFFFFu) ? 0xFFFFFFFFu : static_cast<u32>(timeout_us);
  }

  hal::CanInterface *can_;
  std::array<PendingFrame, kPendingCapacity> pending_{};
  std::atomic<u32> head_{0};            ///< 写指针，仅总线回调访问
  std::atomic<u32> tail_{0};            ///< 读指针，仅Update()访问
  std::atomic<u32> dropped_frames_{0};  ///< 未能计入统计的帧数
  etl::vector<Entry, MaxIds> entries_;  ///< 按ID升序排列的统计项
};

}  // namespace rm::modules

#endif  // LIBRM_MODULES_CANSNIFFER_HPP