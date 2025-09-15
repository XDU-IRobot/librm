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
 * @file  librm/device/device.h
 * @brief 设备基类和设备管理器，用于监视设备状态
 * @todo  考虑到判断设备在线需要一个参照时间，所以需要先做一个时间模块提供全局时间参照
 */

#ifndef LIBRM_DEVICE_DEVICE_H
#define LIBRM_DEVICE_DEVICE_H

#include <list>

#include "librm/core/typedefs.h"

namespace rm::device {

/**
 * @brief 设备状态
 */
enum class DeviceStatus {
  kOnline,   // 在线
  kOffline,  // 离线
  kUnknown,  // 未知(初始状态)
};

/**
 * @brief 设备基类，用于监视设备状态
 */
class Device {
  friend class DeviceManager;

 public:
  Device() = default;
  virtual ~Device() = default;

  /**
   * @brief 更新设备状态
   * @note  子类自行实现这个方法，设备管理器会定期调用；方法内部应该判断设备状态，然后返回设备是否在线
   */
  virtual bool IsDeviceOnline() = 0;
  [[nodiscard]] DeviceStatus GetDeviceStatus() const;

 protected:
  DeviceStatus status_{DeviceStatus::kUnknown};
  u64 last_online_timestamp_{0};  // 上一次调用IsDeviceOnline()时设备在线的时间戳
};

/**
 * @brief 设备管理器，集中更新所有设备的在线状态
 * @note  单例类，使用GetInstance()获取实例
 * @note  应当定期不断调用Update()方法，更新所有设备的在线状态
 */
class DeviceManager final {
 public:
  static DeviceManager &GetInstance();

  void RegisterDevice(Device &device);
  void UnregisterDevice(Device &device);
  void Update();

 private:
  DeviceManager() = default;
  ~DeviceManager() = default;

  std::list<Device *> devices_;
  static constexpr u64 kOfflineTimeout{1000};  // 离线超时时间(ms)
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_DEVICE_H
