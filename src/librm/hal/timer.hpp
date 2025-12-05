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
 * @file  librm/hal/timer.hpp
 * @brief 根据平台宏定义决定具体实现
 */

#ifndef LIBRM_HAL_TIMER_HPP
#define LIBRM_HAL_TIMER_HPP

#include <chrono>

#include "librm/hal/timer_interface.hpp"

#if defined(LIBRM_PLATFORM_STM32)
#include "librm/hal/stm32/hal.hpp"
#elif defined(LIBRM_PLATFORM_LINUX)
#include "librm/hal/linux/timerfd.hpp"
#endif

namespace rm::hal {

/**
 * @brief 轮询式定时器实现（适用于无chrono环境，如STM32）。通过手动累加时间增量实现超时检测，需定期调用 Poll()。
 */
class PollingTimer : public TimerInterface {
 public:
  /**
   * @brief 构造函数，初始化定时器状态。
   */
  PollingTimer() : running_(false), timeout_ms_(0), elapsed_ms_(0), callback_{} {}

  /**
   * @brief 析构函数。
   */
  ~PollingTimer() override = default;

  /**
   * @brief 启动定时器。
   * @param timeout 超时时间，std::chrono::duration，内部以毫秒计。
   */
  void Start(std::chrono::duration<f32> timeout) override {
    timeout_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
    elapsed_ms_ = 0;
    running_ = true;
  }

  /**
   * @brief 停止定时器。
   */
  void Stop() override { running_ = false; }

  /**
   * @brief 查询定时器是否正在运行。
   * @return 若定时器正在运行返回true，否则返回false。
   */
  bool running() const override { return running_; }

  /**
   * @brief 绑定定时器超时回调函数。
   * @param callback 超时后调用的回调函数。
   */
  void AttachCallback(etl::delegate<void()> callback) override { callback_ = callback; }

  /**
   * @brief 轮询定时器状态，需手动传入本次时间增量。
   * @param time_increment_ms 本次调用增加的毫秒数。
   *
   * 若累计时间超过超时时间，则自动调用回调并停止定时器。
   */
  void Poll(uint32_t time_increment_ms) {
    if (!running_) return;
    elapsed_ms_ += time_increment_ms;
    if (elapsed_ms_ >= timeout_ms_) {
      running_ = false;
      if (callback_ && running_) {
        callback_();
      }
    }
  }

 private:
  bool running_;                    ///< 定时器运行状态
  uint32_t timeout_ms_;             ///< 超时时间（毫秒）
  uint32_t elapsed_ms_;             ///< 已累计时间（毫秒）
  etl::delegate<void()> callback_;  ///< 超时回调
};

#if defined(LIBRM_PLATFORM_STM32)
#if defined(HAL_TIM_MODULE_ENABLED)
// TODO
#else
#warning "No HAL timer module found, using polling timer as fallback."
using Timer = PollingTimer;
#endif
#elif defined(LIBRM_PLATFORM_LINUX)
using Timer = linux_::TimerFd;
#endif
}  // namespace rm::hal

#endif  // LIBRM_HAL_TIMER_HPP
