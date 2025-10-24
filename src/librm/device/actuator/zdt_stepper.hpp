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

#include "librm/core/typedefs.hpp"
#include "librm/hal/serial_interface.hpp"
#include "librm/device/device.hpp"

namespace rm::device {

class ZdtStepper : public Device {
 public:
  ZdtStepper(hal::SerialInterface &serial, u8 motor_id, bool reversed = false);
  ~ZdtStepper() = default;

  /**
   * @brief    速度模式
   * @param    vel ：速度       ，范围-5000 - 5000RPM
   * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
   * @param    sync ：多机同步标志，false为不启用，true为启用
   * @retval   地址 + 功能码 + 命令状态 + 校验字节
   */
  void MotorVelCtrl(u16 vel, u8 acc, bool sync);

  /**
   * @brief    位置模式
   * @param    vel ：速度(RPM)   ，范围-5000 - 5000RPM
   * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
   * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
   * @param    absolute ：相位/绝对标志，false为相对运动，true为绝对值运动
   * @param    sync ：多机同步标志 ，false为不启用，true为启用
   * @param    absolute ：相位/绝对标志，false为相对运动，true为绝对值运动
   * @param    sync ：多机同步标志 ，false为不启用，true为启用
   * @retval   地址 + 功能码 + 命令状态 + 校验字节
   */
  void MotorPosCtrl(u16 vel, u8 acc, u32 clk, bool absolute, bool sync);

  /**
   * @brief    触发和这个电机在同一条串口总线上的所有电机执行同步命令
   */
  void MotorSyncCtrl();
  void MotorSyncCtrl();

  /**
   * @brief    读取电机位置
   */
  void ReadPos();
  void ReadPos();

  /**
   * @brief    读取电机速度
   */
  void ReadVel();
  void ReadVel();

  /**
   * @brief    获取电机状态
   */
  [[nodiscard]] auto feedback() { return feedback_; }

 private:
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

 private:
  hal::SerialInterface *serial_;
  bool reversed_;

  struct {
    f32 pos{0.0f};
    f32 vel{0.0f};
  } feedback_{};

  const u8 motor_id_;
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_ZDT_STEPPER_HPP