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
 * @file  librm/device/supercap/supercap.hpp
 * @brief 超级电容
 */

#ifndef LIBRM_DEVICE_SUPERCAP_SUPERCAP_HPP
#define LIBRM_DEVICE_SUPERCAP_SUPERCAP_HPP

#include "librm/core/typedefs.hpp"
#include "librm/device/can_device.hpp"

namespace rm::device {

/**
 * @brief 超级电容的错误flags各位的定义
 */
enum class SuperCapError : u16 {
  kOverVoltage = 1u << 0,        // 电容过压
  kOverCurrent = 1u << 1,        // 电容过流
  kUnderVoltage = 1u << 2,       // 电容欠压
  kInputUnderVoltage = 1u << 3,  // 裁判系统欠压
  kNoData = 1u << 4,             // 未读到数据

  // 5-15 reserved
};

/**
 * @brief 超级电容
 */
class SuperCap final : public CanDevice {
 public:
  explicit SuperCap(hal::CanInterface &can);
  SuperCap(SuperCap &&other) noexcept = default;
  SuperCap() = delete;
  ~SuperCap() override = default;

  [[nodiscard]] f32 voltage() const;
  [[nodiscard]] f32 current() const;
  [[nodiscard]] bool error(SuperCapError error) const;

  void UpdateChassisBuffer(i16 power_buffer);
  void UpdateSettings(i16 power_limit, i16 output_limit, i16 input_limit, bool power_switch, bool enable_log);
  void RxCallback(const hal::CanMsg *msg) override;

 private:
  u8 tx_buf_[8]{0};

  f32 voltage_{};
  f32 current_{};
  u16 error_flags_{};
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_SUPERCAP_SUPERCAP_HPP
