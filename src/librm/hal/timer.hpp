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
 * @file  librm/hal/timer.hpp
 * @brief 根据平台宏定义决定具体实现
 */

#ifndef LIBRM_HAL_TIMER_HPP
#define LIBRM_HAL_TIMER_HPP

#include <chrono>

#include "librm/hal/timer_interface.hpp"

#if defined(LIBRM_PLATFORM_STM32)
#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_TIM_MODULE_ENABLED)
#include "librm/hal/stm32/timer.hpp"
#endif
#elif defined(LIBRM_PLATFORM_LINUX)
#include "librm/hal/linux/timerfd.hpp"
#endif

namespace rm::hal {

/**
 * @brief 轮询模拟定时器实现，当没有合适的硬件定时器时可作为替代方案，不推荐使用。
 */
class PollingTimer : public TimerInterface {
 public:
  /**
   * @brief 构造函数，初始化定时器状态。
   */
  PollingTimer()
      : running_(false),
        timeout_ms_(0),
        elapsed_ms_(0),
        callback_{},
        type_(TimerType::kPeriodic),
        timeout_set_(false) {}

  /**
   * @brief 析构函数。
   */
  ~PollingTimer() override = default;

  /**
   * @brief 启动定时器。
   * @return 是否启动成功，未设置过超时时间时返回false。
   */
  bool Start() override {
    if (!timeout_set_) {
      return false;  // 还没有成功设置过超时时间，启动失败
    }
    elapsed_ms_ = 0;
    running_ = true;
    return true;
  }

  /**
   * @brief 停止定时器。
   * @return 是否停止成功，定时器本来就没有在运行时返回false。
   */
  bool Stop() override {
    if (!running_) {
      return false;
    }
    running_ = false;
    return true;
  }

  /**
   * @brief 查询定时器是否正在运行。
   * @return 若定时器正在运行返回true，否则返回false。
   */
  bool running() const override { return running_; }

  /**
   * @brief 设置定时器超时时间。
   * @param timeout 超时时间，内部以毫秒计，不足1毫秒按1毫秒处理。
   * @return 是否设置成功，定时器正在运行、或超时时间不为正数时返回false。
   */
  bool SetTimeout(std::chrono::duration<f32> timeout) override {
    if (running_) {
      return false;
    }
    if (timeout <= std::chrono::duration<f32>::zero()) {
      return false;
    }
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
    if (ms < 1) {
      ms = 1;
    }
    timeout_ms_ = static_cast<u32>(ms);
    timeout_set_ = true;
    return true;
  }

  /**
   * @brief 查询定时器超时时间。
   * @return 当前超时时间；从未成功设置过超时时间时返回 0。
   * @note 返回的是量化到整毫秒后的实际周期，可能与 SetTimeout() 传入的值略有差异。
   */
  std::chrono::duration<f32> GetTimeout() const override {
    if (!timeout_set_) {
      return std::chrono::duration<f32>::zero();
    }
    return std::chrono::duration<f32>(static_cast<f32>(timeout_ms_) / 1000.0f);
  }

  /**
   * @brief 绑定定时器超时回调函数。
   * @param callback 超时后调用的回调函数。
   * @return 是否绑定成功，定时器正在运行时绑定失败并返回false。
   */
  bool AttachCallback(etl::delegate<void()> callback) override {
    if (running_) {
      return false;
    }
    callback_ = callback;
    return true;
  }

  /**
   * @brief 设置定时器类型。
   * @param type 定时器类型，循环或单次。
   * @return 是否设置成功，定时器正在运行时设置失败并返回false。
   *
   * @note 对象创建后默认为循环模式。
   */
  bool SetTimerType(TimerType type) override {
    if (running_) {
      return false;
    }
    type_ = type;
    return true;
  }

  /**
   * @brief 轮询定时器状态，需手动传入本次时间增量。
   * @param time_increment_ms 本次调用增加的毫秒数。
   *
   * 若累计时间超过超时时间，则自动调用回调；循环模式下重新开始计时，单次模式下停止定时器。
   */
  void Poll(uint32_t time_increment_ms) {
    if (!running_) return;
    elapsed_ms_ += time_increment_ms;
    if (elapsed_ms_ >= timeout_ms_) {
      if (type_ == TimerType::kOneShot) {
        running_ = false;
      } else {
        elapsed_ms_ = 0;
      }
      if (callback_) {
        callback_();
      }
    }
  }

 private:
  bool running_;                    ///< 定时器运行状态
  uint32_t timeout_ms_;             ///< 超时时间（毫秒）
  uint32_t elapsed_ms_;             ///< 已累计时间（毫秒）
  etl::delegate<void()> callback_;  ///< 超时回调
  TimerType type_;                  ///< 定时器类型（循环/单次）
  bool timeout_set_;                ///< 是否已经设置过超时时间
};

#if defined(LIBRM_PLATFORM_STM32)
#if defined(HAL_TIM_MODULE_ENABLED)
using Timer = stm32::Timer;
#else
#warning "No HAL timer module found, using polling timer as fallback."
using Timer = PollingTimer;
#endif
#elif defined(LIBRM_PLATFORM_LINUX)
using Timer = linux_::TimerFd;
#endif
}  // namespace rm::hal

#endif  // LIBRM_HAL_TIMER_HPP
