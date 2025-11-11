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
 * @file  librm/modules/sequence_player.hpp
 * @brief 通用序列播放器，可以播放任意类型的数据序列
 */

#ifndef LIBRM_MODULES_SEQUENCE_PLAYER_HPP
#define LIBRM_MODULES_SEQUENCE_PLAYER_HPP

#include <chrono>
#include <tuple>

#include <etl/delegate.h>

#include "librm/core/typedefs.hpp"

namespace rm::modules {

/**
 * @brief 序列生成器的基类
 * @tparam OutputType 序列输出的数据类型
 *
 * @note 子类需要实现Update和Reset方法来定义一个序列生成器
 * @note Update方法接收当前时间戳，基于绝对时间进行状态更新
 */
template <typename OutputType>
class SequenceGenerator {
 public:
  virtual ~SequenceGenerator() = default;
  using Output = OutputType;
  using TimePoint = std::chrono::steady_clock::time_point;
  using Duration = std::chrono::steady_clock::duration;

  /**
   * @brief       更新序列状态并返回当前输出
   * @param now   当前时间点
   * @return      当前序列的输出值
   */
  virtual Output Update(TimePoint now) = 0;

  /**
   * @brief       重置序列状态到初始状态
   * @param now   重置时的时间点
   */
  virtual void Reset(TimePoint now) = 0;

 protected:
  /**
   * @brief       计算从起始时间到现在经过的毫秒数
   * @param start 起始时间点
   * @param now   当前时间点
   * @return      经过的毫秒数
   */
  static u32 ElapsedMs(TimePoint start, TimePoint now) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
  }
};

namespace detail {
template <typename T, typename Tuple>
struct tuple_element_index;

template <typename T, typename... Ts>
struct tuple_element_index<T, std::tuple<T, Ts...>> {
  static constexpr usize value = 0;
};

template <typename T, typename U, typename... Ts>
struct tuple_element_index<T, std::tuple<U, Ts...>> {
  static constexpr usize value = 1 + tuple_element_index<T, std::tuple<Ts...>>::value;
};
}  // namespace detail

/**
 * @brief 通用序列播放器，管理多个序列生成器并在它们之间切换
 * @tparam OutputType       序列输出的数据类型
 * @tparam SequenceTypes    序列生成器类型列表，所有类型必须继承自SequenceGenerator<OutputType>
 *
 * @example
 * ```cpp
 * // 定义序列生成器
 * class LinearRamp : public SequenceGenerator<float> {
 *   float value_ = 0.0f;
 * public:
 *   float Update(TimePoint now_unused) override { return value_ += 0.1f; }
 *   void Reset(TimePoint now_unused) override { value_ = 0.0f; }
 * };
 *
 * class SineWave : public SequenceGenerator<float> {
 *   float phase_ = 0.0f;
 * public:
 *   float Update(TimePoint now_unused) override {
 *     phase_ += 0.01f;
 *     return std::sin(phase_);
 *   }
 *   void Reset(TimePoint now_unused) override { phase_ = 0.0f; }
 * };
 *
 * // 创建播放器
 * SequencePlayer<float, LinearRamp, SineWave> player;
 *
 * // 切换序列
 * player.SetSequence<SineWave>();
 *
 * // 更新并获取输出
 * float output = player.Update();
 * ```
 */
template <typename OutputType, typename... SequenceTypes>
class SequencePlayer {
  static_assert(sizeof...(SequenceTypes) > 0, "At least one sequence type must be provided");
  static_assert((std::is_base_of<SequenceGenerator<OutputType>, SequenceTypes>::value && ...),
                "All SequenceTypes must derive from SequenceGenerator<OutputType>");

 public:
  using Output = OutputType;

  SequencePlayer() : sequences_{SequenceTypes()...} {
    // 初始化时设置当前更新函数指向第一个序列的Update方法
    SetSequenceByIndex<0>();
  }

  /**
   * @brief             切换当前的序列生成器
   * @tparam Sequence   要切换到的序列生成器类型
   */
  template <typename Sequence>
  void SetSequence() {
    constexpr usize sequence_id = detail::tuple_element_index<Sequence, std::tuple<SequenceTypes...>>::value;
    SetSequenceByIndex<sequence_id>();
  }

  /**
   * @brief  更新当前序列并获取输出
   * @return 当前序列的输出值
   */
  Output Update() {
    const auto now = std::chrono::steady_clock::now();
    // 直接调用当前序列的Update方法，传入当前时间
    current_output_ = current_update_fn_(now);
    return current_output_;
  }

  /**
   * @brief  获取当前输出值（不更新序列）
   * @return 当前序列的输出值
   */
  const Output &current_output() const { return current_output_; }

  /**
   * @brief  重置当前序列到初始状态
   */
  void Reset() {
    auto now = std::chrono::steady_clock::now();
    if (current_reset_fn_) {
      current_reset_fn_(now);
    }
  }

 private:
  /**
   * @brief                   通过索引设置序列（内部辅助方法）
   * @tparam sequence_id      序列在tuple中的索引
   */
  template <usize sequence_id>
  void SetSequenceByIndex() {
    auto now = std::chrono::steady_clock::now();
    // 重置序列状态
    std::get<sequence_id>(sequences_).Reset(now);
    // 使用etl::delegate绑定对应序列的Update方法
    current_update_fn_ = etl::delegate<Output(std::chrono::steady_clock::time_point)>::template create<
        SequencePlayer, &SequencePlayer::UpdateSequence<sequence_id>>(*this);
    // 绑定Reset方法
    current_reset_fn_ = etl::delegate<void(std::chrono::steady_clock::time_point)>::create<
        SequencePlayer, &SequencePlayer::ResetSequence<sequence_id>>(*this);
  }

  /**
   * @brief                   调用指定索引的序列的Update方法（内部辅助方法）
   * @tparam sequence_id      序列在tuple中的索引
   * @param now               当前时间点
   * @return                  序列输出值
   */
  template <usize sequence_id>
  Output UpdateSequence(std::chrono::steady_clock::time_point now) {
    return std::get<sequence_id>(sequences_).Update(now);
  }

  /**
   * @brief                   调用指定索引的序列的Reset方法（内部辅助方法）
   * @tparam sequence_id      序列在tuple中的索引
   * @param now               当前时间点
   */
  template <usize sequence_id>
  void ResetSequence(std::chrono::steady_clock::time_point now) {
    std::get<sequence_id>(sequences_).Reset(now);
  }

  Output current_output_{};
  std::tuple<SequenceTypes...> sequences_;
  // 使用etl::delegate，无动态内存分配，直接指向当前序列的Update和Reset方法
  etl::delegate<Output(std::chrono::steady_clock::time_point)> current_update_fn_;
  etl::delegate<void(std::chrono::steady_clock::time_point)> current_reset_fn_;
};

}  // namespace rm::modules

#endif  // LIBRM_MODULES_SEQUENCE_PLAYER_HPP
