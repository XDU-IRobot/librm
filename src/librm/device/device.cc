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
 * @file  librm/device/device.cc
 * @brief 驱动框架的设备基类，主要功能为监视设备在线状态
 */

#include "device.hpp"

namespace rm::device {

void Device::SetHeartbeatTimeout(duration timeout) { heartbeat_timeout_ = timeout; }

[[nodiscard]] bool Device::IsAlive() {
  if (online_status_ == Status::kOnline) {
    const auto now = std::chrono::steady_clock::now();
    // 如果距离上次在线时间戳超过心跳超时时间，认为设备离线
    if (now - last_online_ > heartbeat_timeout_) {
      online_status_ = Status::kOffline;
    }
  }
  return online_status_ == Status::kOnline;
}

[[nodiscard]] Device::time_point Device::last_online() const { return last_online_; }

void Device::Heartbeat() {
  last_online_ = std::chrono::steady_clock::now();
  online_status_ = Status::kOnline;
}

}  // namespace rm::device
