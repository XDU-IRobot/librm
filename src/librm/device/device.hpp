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
 * @file  librm/device/device.hpp
 * @brief 驱动框架的设备基类，主要功能为监视设备在线状态
 */

#ifndef LIBRM_DEVICE_DEVICE_HPP
#define LIBRM_DEVICE_DEVICE_HPP

#include <chrono>

#include "librm/core/typedefs.hpp"

namespace rm::device {

/**
 * @brief 驱动框架的设备基类，主要功能为监视设备在线状态
 */
class Device {
  friend class DeviceManager;

  using time_point = std::chrono::time_point<std::chrono::steady_clock>;
  using duration = std::chrono::steady_clock::duration;

 public:
  Device() = default;
  virtual ~Device() = default;

  /**
   * @brief 设置心跳超时时间
   * @param timeout 心跳超时时间
   * @note  如果超过这个时间没有收到心跳则认为设备离线，默认值为1秒
   */
  void SetHeartbeatTimeout(duration timeout);

  /**
   * @return 设备在线状态
   */
  [[nodiscard]] bool IsAlive();

  /**
   * @return 上一次调用Heartbeat()时设备在线的时间戳
   */
  [[nodiscard]] time_point last_online() const;

 protected:
  /**
   * @brief 更新设备状态
   * @note  子类应该在确定设备仍然在线（比如收到反馈报文）时调用此函数，更新设备在线时间戳
   */
  void Heartbeat();

 private:
  /**
   * @brief 设备状态
   */
  enum Status {
    kUnknown = -1,  ///< 未知
    kOffline,       ///< 离线
    kOnline,        ///< 在线
  };

  Status online_status_{Status::kUnknown};
  time_point last_online_{time_point::min()};  ///< 上一次调用Heartbeat()时设备在线的时间戳
  duration heartbeat_timeout_{std::chrono::seconds(1)};  ///< 心跳超时时间，超过这个时间没有收到心跳则认为设备离线
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_DEVICE_HPP
