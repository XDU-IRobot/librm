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

[[nodiscard]] bool Device::Ok() {
  const auto now = std::chrono::steady_clock::now();
  if (now - last_seen_ > heartbeat_timeout_) {
    // 如果距离上次设备上报状态时间超过心跳超时时间，则认为设备离线
    online_status_ = kOffline;
  }
  return online_status_ == kOk;
}

[[nodiscard]] Device::time_point Device::last_seen() const { return last_seen_; }

void Device::ReportStatus(Status status) {
  last_seen_ = std::chrono::steady_clock::now();
  online_status_ = status;
}

}  // namespace rm::device
