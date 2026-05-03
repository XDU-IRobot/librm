/*
  Copyright (c) 2026 XDU-IRobot

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
 * @file  librm/device/actuator/dm_motor.hpp
 * @brief 达妙电机类库
 * @note  https://gitee.com/kit-miao/damiao
 */

#ifndef LIBRM_DEVICE_ACTUATOR_DM_MOTOR_HPP
#define LIBRM_DEVICE_ACTUATOR_DM_MOTOR_HPP

#include <utility>
#include <cstring>

#include "librm/device/can_device.hpp"
#include "librm/core/typedefs.hpp"
#include "librm/modules/utils.hpp"

namespace rm::device {

/**
 * @brief 达妙电机状态
 */
enum class DmMotorStatus {
  kDisable = 0x0,
  kEnable = 0x1,
  kOverVoltage = 0x8,
  kUnderVoltage = 0x9,
  kOverCurrent = 0xa,
  kMosOverTemp = 0xb,
  kCoilOverTemp = 0xc,
  kCommLost = 0xd,
  kOverload = 0xe,
};

/**
 * @brief 达妙电机的三种控制模式
 */
enum class DmMotorControlMode {
  kMit,            // MIT模式
  kSpeedPosition,  // 速度位置模式
  kSpeed           // 速度模式
};

/**
 * @brief 达妙电机的功能指令和其对应的最后一个字节的值
 */
enum class DmMotorInstructions {
  kEnable = 0xfc,           // 使能
  kDisable = 0xfd,          // 失能
  kSetZeroPosition = 0xfe,  // 设置零点
  kClearError = 0xfb,       // 清除错误
};

/**
 * @brief 在达妙电机的上位机软件里设置的一些会影响反馈信息的参数，需要填入这里传给类库
 */
template <DmMotorControlMode control_mode>
struct DmMotorSettings {};

/**
 * @brief 在达妙电机的上位机软件里设置的一些会影响反馈信息的参数，需要填入这里传给类库
 */
template <>
struct DmMotorSettings<DmMotorControlMode::kSpeed> {
  i16 master_id;  // 电机反馈报文的ID
  i16 slave_id;   // 电机控制报文的ID
  f32 v_max;      // 最大速度
  f32 t_max;      // 最大扭矩
};

/**
 * @brief 在达妙电机的上位机软件里设置的一些会影响反馈信息的参数，需要填入这里传给类库
 */
template <>
struct DmMotorSettings<DmMotorControlMode::kSpeedPosition> {
  i16 master_id;  // 电机反馈报文的ID
  i16 slave_id;   // 电机控制报文的ID
  f32 p_max;      // 最大位置
  f32 v_max;      // 最大速度
  f32 t_max;      // 最大扭矩
};

/**
 * @brief 在达妙电机的上位机软件里设置的一些会影响反馈信息的参数，需要填入这里传给类库
 */
template <>
struct DmMotorSettings<DmMotorControlMode::kMit> {
  i16 master_id;                 // 电机反馈报文的ID
  i16 slave_id;                  // 电机控制报文的ID
  f32 p_max;                     // 最大位置
  f32 v_max;                     // 最大速度
  f32 t_max;                     // 最大扭矩
  std::pair<f32, f32> kp_range;  // kp范围
  std::pair<f32, f32> kd_range;  // kd范围
};

/**
 * @brief  达妙电机类库
 * @tparam control_mode 电机的控制模式
 */
template <DmMotorControlMode control_mode>
class DmMotor final : public CanDevice {
 public:
  using Settings = DmMotorSettings<control_mode>;

  DmMotor() = delete;
  DmMotor(DmMotor &&other) noexcept = default;
  ~DmMotor() override = default;

  /**
   * @param can         CAN外设对象
   * @param settings    电机参数
   * @param reversed    是否反转
   */
  DmMotor(hal::CanInterface &can, Settings settings, bool reversed = false)
      : CanDevice(can, settings.master_id), settings_(settings), reversed_(reversed) {}

  /**
   * @brief  MIT模式下发送控制命令
   * @tparam mode                   控制模式
   * @param  position_rad           期望位置
   * @param  speed_rad_per_sec      期望速度
   * @param  torque_ff_nm           前馈力矩
   * @param  kp                     位置误差比例系数
   * @param  kd                     速度误差比例系数
   */
  template <DmMotorControlMode mode = control_mode,
            typename std::enable_if_t<mode == DmMotorControlMode::kMit, int> = 0>
  void SetMitCommand(f32 position_rad, f32 speed_rad_per_sec, f32 torque_ff_nm, f32 kp, f32 kd) {
    if (reversed_) {
      position_rad = -position_rad;
      speed_rad_per_sec = -speed_rad_per_sec;
      torque_ff_nm = -torque_ff_nm;
    }

    using modules::FloatToInt;
    const u16 pos_tmp = FloatToInt(position_rad, -settings_.p_max, settings_.p_max, 16);
    const u16 vel_tmp = FloatToInt(speed_rad_per_sec, -settings_.v_max, settings_.v_max, 12);
    const u16 kp_tmp = FloatToInt(kp, settings_.kp_range.first, settings_.kp_range.second, 12);
    const u16 kd_tmp = FloatToInt(kd, settings_.kd_range.first, settings_.kd_range.second, 12);
    const u16 tor_tmp = FloatToInt(torque_ff_nm, -settings_.t_max, settings_.t_max, 12);

    tx_buffer_[0] = (pos_tmp >> 8);
    tx_buffer_[1] = pos_tmp;
    tx_buffer_[2] = (vel_tmp >> 4);
    tx_buffer_[3] = ((vel_tmp & 0xf) << 4) | (kp_tmp >> 8);
    tx_buffer_[4] = kp_tmp;
    tx_buffer_[5] = (kd_tmp >> 4);
    tx_buffer_[6] = ((kd_tmp & 0xf) << 4) | (tor_tmp >> 8);
    tx_buffer_[7] = tor_tmp;
    can_->Write(settings_.slave_id, tx_buffer_, 8);
  }

  /**
   * @brief  速度位置模式下发送控制命令
   * @tparam mode                   控制模式
   * @param  position_rad           期望位置
   * @param  speed_rad_per_sec      期望速度
   */
  template <DmMotorControlMode mode = control_mode,
            typename std::enable_if_t<mode == DmMotorControlMode::kSpeedPosition, int> = 0>
  void SetPosition(f32 position_rad, f32 speed_rad_per_sec) {
    if (reversed_) {
      position_rad = 0 - position_rad;
    }
    memcpy(tx_buffer_, &position_rad, 4);
    memcpy(tx_buffer_ + 4, &speed_rad_per_sec, 4);
    can_->Write(settings_.slave_id, tx_buffer_, 8);
  }

  /**
   * @brief  速度模式下发送控制命令
   * @tparam mode               控制模式
   * @param  speed_rad_per_sec  期望速度
   */
  template <DmMotorControlMode mode = control_mode,
            typename std::enable_if_t<mode == DmMotorControlMode::kSpeed, int> = 0>
  void SetSpeed(f32 speed_rad_per_sec) {
    if (reversed_) {
      speed_rad_per_sec = -speed_rad_per_sec;
    }
    memcpy(tx_buffer_, &speed_rad_per_sec, 4);
    can_->Write(settings_.slave_id, tx_buffer_, 4);
  }

  /**
   * @brief 向电机发送功能指令，可以发送的指令见DmMotorInstructions枚举定义
   * @param instruction 要发送的指令
   */
  void SendInstruction(DmMotorInstructions instruction) {
    memset(tx_buffer_, 0xff, 8);
    tx_buffer_[7] = static_cast<u8>(instruction);
    can_->Write(settings_.slave_id, tx_buffer_, 8);
  }

  /** 取值函数 **/
  [[nodiscard]] u8 status() const { return status_; }
  [[nodiscard]] f32 pos() const { return position_; }
  [[nodiscard]] f32 vel() const { return speed_; }
  [[nodiscard]] f32 tau() const { return torque_; }
  [[nodiscard]] u8 mos_temperature() const { return mos_temperature_; }
  [[nodiscard]] u8 coil_temperature() const { return coil_temperature_; }
  /*************/

 private:
  /**
   * @brief CAN回调函数，解码收到的反馈报文
   * @param msg   收到的报文
   */
  void RxCallback(const hal::CanFrame *msg) override {
    ReportStatus(kOk);
    const int p_int = (msg->data[1] << 8) | msg->data[2];
    const int v_int = (msg->data[3] << 4) | (msg->data[4] >> 4);
    const int t_int = ((msg->data[4] & 0xf) << 8) | msg->data[5];
    status_ = (msg->data[0] >> 4) & 0x0f;
    using modules::IntToFloat;
    position_ = IntToFloat(p_int, -settings_.p_max, settings_.p_max, 16);
    speed_ = IntToFloat(v_int, -settings_.v_max, settings_.v_max, 12);
    torque_ = IntToFloat(t_int, -settings_.t_max, settings_.t_max, 12);
    mos_temperature_ = msg->data[6];
    coil_temperature_ = msg->data[7];
    if (reversed_) {
      position_ = -position_;
      speed_ = -speed_;
      torque_ = -torque_;
    }
  }

  DmMotorSettings<control_mode> settings_{};
  bool reversed_{};  // 是否反转
  u8 tx_buffer_[8]{0};
  /**   FEEDBACK DATA   **/
  u8 status_{};            // 电机状态
  f32 position_{};         // 电机位置，单位rad
  f32 speed_{};            // 电机转速，单位rad/s
  f32 torque_{};           // 电机实际扭矩，单位N*m
  u8 mos_temperature_{};   // 电机MOS管的平均温度
  u8 coil_temperature_{};  // 电机线圈的平均温度
  /***********************/
};

using DmMotorMit = DmMotor<DmMotorControlMode::kMit>;
using DmMotorSpeedPosition = DmMotor<DmMotorControlMode::kSpeedPosition>;
using DmMotorSpeed = DmMotor<DmMotorControlMode::kSpeed>;

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_DM_MOTOR_HPP
