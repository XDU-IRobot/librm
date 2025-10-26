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
 * @file  librm/device/supercap/supercap.h
 * @brief 超级电容
 */

#ifndef LIBRM_DEVICE_SUPERCAP_SUPERCAP_H
#define LIBRM_DEVICE_SUPERCAP_SUPERCAP_H

#include "librm/core/typedefs.h"
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
 * @brief 港科电容错误flags各位的定义
 */
#ifdef Gk_SuperCap

#define ERROR_UNDER_VOLTAGE 0b00000001
#define ERROR_OVER_VOLTAGE 0b00000010
#define ERROR_BUCK_BOOST 0b00000100
#define ERROR_SHORT_CIRCUIT 0b00001000
#define ERROR_HIGH_TEMPERATURE 0b00010000
#define ERROR_NO_POWER_INPUT 0b00100000
#define ERROR_CAPACITOR 0b01000000

#endif

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

#ifdef Gk_SuperCap
  void Gk_UpdateCan(u8 enableDCDC, u8 systemRestart, u8 resv0, u16 feedbackRefereePowerLimit,
                    u16 feedbackRefereeEnergyBuffer, u8 *resv1);
  void Gk_RxCallback(const hal::CanMsg *msg);
#endif

 private:
  u8 tx_buf_[8]{0};

  f32 voltage_{};
  f32 current_{};
  u16 error_flags_{};

#ifdef Gk_SuperCap
  struct RxData {
    u8 errorCode{};
    f32 chassisPower{};
    u16 chassisPowerLimit{};
    u8 capEnergy{};
  } __attribute__((packed));

  struct TxData {
    u8 enableDCDC : 1;
    u8 systemRestart : 1;
    u8 resv0 : 6;
    u8 feedbackRefereePowerLimit;
    u16 feedbackRefereeEnergyBuffer;
    u8 resv1[3];
  } __attribute__((packed));
#endif
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_SUPERCAP_SUPERCAP_H
