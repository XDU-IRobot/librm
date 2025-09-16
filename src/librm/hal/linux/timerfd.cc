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
 * @file  librm/hal/linux/timerfd.cc
 * @brief 基于内核定时器实现的Timer
 */

#include "timerfd.hpp"

#include <unistd.h>
#include <sys/timerfd.h>
#include <cstring>

namespace rm::hal::linux_ {

TimerFd::TimerFd() : running_{false}, tfd_{-1}, is_periodic_{false} {}

TimerFd::~TimerFd() { Stop(); }

void TimerFd::Start(std::chrono::duration<f32> timeout) {
  Stop();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tfd_ = ::timerfd_create(CLOCK_MONOTONIC, 0);
    if (tfd_ == -1) {
      throw std::runtime_error("timerfd_create failed");
    }
    itimerspec new_value{};
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(timeout);
    auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout - sec);
    new_value.it_value.tv_sec = sec.count();
    new_value.it_value.tv_nsec = nsec.count();
    if (is_periodic_) {
      new_value.it_interval.tv_sec = sec.count();
      new_value.it_interval.tv_nsec = nsec.count();
    } else {
      new_value.it_interval.tv_sec = 0;
      new_value.it_interval.tv_nsec = 0;
    }
    if (::timerfd_settime(tfd_, 0, &new_value, nullptr) == -1) {
      ::close(tfd_);
      tfd_ = -1;
      throw std::runtime_error("timerfd_settime failed");
    }
    running_ = true;
    worker_ = std::thread([this]() {
      uint64_t expirations;
      while (true) {
        {
          std::unique_lock<std::mutex> lock(mutex_);
          if (!running_) break;
        }
        int ret = ::read(tfd_, &expirations, sizeof(expirations));
        if (ret == sizeof(expirations)) {
          etl::delegate<void()> cb;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            cb = callback_;
          }
          if (cb) cb();
          if (!is_periodic_) break;
        } else if (ret == -1 && errno == EINTR) {
          continue;
        } else {
          break;
        }
      }
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
      if (tfd_ != -1) {
        ::close(tfd_);
        tfd_ = -1;
      }
    });
  }
}

/**
 * @brief 设置定时器为周期/单次模式。
 * @param periodic true为周期定时器，false为oneshot。
 */
void TimerFd::SetPeriodic(bool periodic) {
  std::lock_guard<std::mutex> lock(mutex_);
  is_periodic_ = periodic;
}

void TimerFd::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) return;
    running_ = false;
    if (tfd_ != -1) {
      ::close(tfd_);
      tfd_ = -1;
    }
  }
  cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

bool TimerFd::running() const { return running_; }

void TimerFd::AttachCallback(etl::delegate<void()> callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  callback_ = callback;
}

}  // namespace rm::hal::linux_
