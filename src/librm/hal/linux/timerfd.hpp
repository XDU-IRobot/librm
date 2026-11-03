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
 * @file  librm/hal/linux/timerfd.hpp
 * @brief 基于内核定时器实现的Timer
 */

#ifndef LIBRM_HAL_LINUX_TIMERFD_HPP
#define LIBRM_HAL_LINUX_TIMERFD_HPP

#include <condition_variable>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>

#include "librm/hal/timer_interface.hpp"

namespace rm::hal::linux_ {

class TimerFd : public TimerInterface {
 public:
  TimerFd();
  ~TimerFd() override;

  bool Start() override;

  bool Stop() override;

  bool running() const override;

  bool SetTimeout(std::chrono::duration<f32> timeout) override;

  std::chrono::duration<f32> GetTimeout() const override;

  bool AttachCallback(etl::delegate<void()> callback) override;

  bool SetTimerType(TimerType type) override;

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  int tfd_;
  std::atomic<bool> running_;
  etl::delegate<void()> callback_;
  bool is_periodic_;
  std::chrono::duration<f32> timeout_;
  bool timeout_set_;
};

}  // namespace rm::hal::linux_

#endif  // LIBRM_HAL_LINUX_TIMERFD_HPP