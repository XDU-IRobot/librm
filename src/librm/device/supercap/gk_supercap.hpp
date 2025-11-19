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
 * @file  librm/device/supercap/gk_supercap.hpp
 * @brief 港科超级电容协议
 */

#ifndef LIBRM_DEVICE_SUPERCAP_GK_SUPERCAP_HPP
#define LIBRM_DEVICE_SUPERCAP_GK_SUPERCAP_HPP

#include "librm/device/can_device.hpp"

namespace rm::device {

/**
 * @brief 港科超级电容
 */
class GkSupercap final : public CanDevice {
 public:
  /**
   * @brief 港科超级电容的错误flags各位的定义
   */
  enum ErrorFlag : u8 {
    kUnderVoltage = 1u << 0,     ///< 低电压警告
    kOverVoltage = 1u << 1,      ///< 超压警告
    kBuckBoost = 1u << 2,        ///< BUCKBOOST回路错误警告(通常为MOS管损坏或者驱动器损坏)
    kShortCircuit = 1u << 3,     ///< 短路警告
    kHighTemperature = 1u << 4,  ///< 高温警告
    kNoPowerInput = 1u << 5,     ///< 无电源输入警告
    kCapacitor = 1u << 6,        ///< 电容错误警告(通常为未接电容)
  };

  /**
   * @brief 港科超级电容用于CAN通信的结构体，必须以一位对齐
   */
  struct __attribute__((__packed__)) TxData {
    u8 enable_dcdc : 1;                  ////< 是否开启电容
    u8 system_restart : 1;               ////< 重启电容
    u8 resv0 : 6;                        ////< 保留位
    u16 feedback_referee_power_limit;    ////< 底盘功率上限
    u16 feedback_referee_energy_buffer;  ////< 缓冲能量
    u8 resv1[3];                         ////< 保留位
  };

  /**
   * @brief 港科超级电容用于CAN通信的结构体，必须以一位对齐
   */
  struct __attribute__((__packed__)) RxData {
    u8 error_code;
    f32 chassis_power;
    u16 chassis_power_limit;
    u8 cap_energy;
  };

  static_assert(sizeof(TxData) <= 8, "TxData size must be less than or equal to 8 bytes");
  static_assert(sizeof(RxData) <= 8, "RxData size must be less than or equal to 8 bytes");

  explicit GkSupercap(hal::CanInterface &can);
  GkSupercap(GkSupercap &&other) noexcept = default;

  GkSupercap() = delete;
  ~GkSupercap() override = default;

  void Update(const TxData &tx_data_);
  void RxCallback(const hal::CanFrame *msg) override;

 private:
  RxData rx_data_{};
  u8 tx_buf_[8]{};
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_SUPERCAP_GK_SUPERCAP_HPP