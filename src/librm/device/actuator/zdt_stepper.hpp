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
 * @file  librm/device/actuator/zdt_stepper.hpp
 * @brief 张大头闭环步进驱动（Emm_V5.0）
 */

#ifndef LIBRM_DEVICE_ACTUATOR_ZDT_STEPPER_HPP
#define LIBRM_DEVICE_ACTUATOR_ZDT_STEPPER_HPP

#include "librm/core/typedefs.h"
#include "librm/hal/serial_interface.h"

using namespace rm;

namespace rm::device {

class ZdtStepper {
 public:
  ZdtStepper(hal::SerialInterface &serial, u8 motor_id = 0);
  ~ZdtStepper() = default;

  /**
   * @brief    速度模式
   * @param    dir ：方向       ，0为CW，其余值为CCW
   * @param    vel ：速度       ，范围0 - 5000RPM
   * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
   * @param    sync_flag ：多机同步标志，false为不启用，true为启用
   * @retval   地址 + 功能码 + 命令状态 + 校验字节
   */
  void MotorVelCtrl(u8 dir, u16 vel, u8 acc, bool sync_flag);

  /**
   * @brief    位置模式
   * @param    dir ：方向        ，0为CW，其余值为CCW
   * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
   * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
   * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
   * @param    absolute ：相位/绝对标志，false为相对运动，true为绝对值运动
   * @param    sync ：多机同步标志 ，false为不启用，true为启用
   * @retval   地址 + 功能码 + 命令状态 + 校验字节
   */
  void MotorPosCtrl(u8 dir, u16 vel, u8 acc, u32 clk, bool absolute, bool sync);

  /**
   * @brief    多机同步运动
   * @param    addr  ：电机地址
   * @retval   地址 + 功能码 + 命令状态 + 校验字节
   */
  void MotorSyncCtrl();

  /**
   * @brief    读取电机位置
   * @param    addr  ：电机地址
   */
  void ReadPos();

  /**
   * @brief    读取电机速度
   * @param    addr  ：电机地址
   */
  void ReadVel();

  /**
   * @brief    获取电机状态
   * @param    addr  ：电机地址
   */
  [[nodiscard]] auto feedback() { return feedback_; }

 private:
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

 private:
  hal::SerialInterface *serial_;

  struct {
    f32 pos{0.0f};
    f32 vel{0.0f};
  } feedback_{};

  const u8 motor_id_;
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_ZDT_STEPPER_HPP