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
 * @file  librm/device/actuator/dm_motor.hpp
 * @brief 达妙电机类库
 * @note  https://gitee.com/kit-miao/damiao
 */

#ifndef LIBRM_DEVICE_ACTUATOR_DM_MOTOR_HPP
#define LIBRM_DEVICE_ACTUATOR_DM_MOTOR_HPP

#include <utility>
#include <cstring>

#include "librm/device/can_device.hpp"
#include "librm/core/typedefs.h"
#include "librm/modules/algorithm/utils.hpp"

namespace rm::device {

/**
 * @brief 达妙电机状态
 */
enum class DmMotorStatus {
  kUnable = 0x0,
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
  DmMotor() = delete;
  ~DmMotor() override = default;

  /**
   * @param can         CAN外设对象
   * @param settings    电机参数
   * @param reversed    是否反转
   */
  DmMotor(hal::CanInterface &can, DmMotorSettings<control_mode> settings, bool reversed = false)
      : CanDevice(can, settings.master_id), settings_(settings), reversed_(reversed) {}

  // 禁止拷贝构造
  DmMotor(const DmMotor &) = delete;
  DmMotor &operator=(const DmMotor &) = delete;

  /**
   * @brief  MIT模式的控制函数
   * @tparam mode                   控制模式
   * @param  position_rad           期望位置
   * @param  max_speed_rad_per_sec  控制过程中的最大速度
   * @param  accel_torque_nm        加速扭矩
   * @param  kp                     比例系数
   * @param  kd                     微分系数
   */
  template <DmMotorControlMode mode = control_mode,
            typename std::enable_if_t<mode == DmMotorControlMode::kMit, int> = 0>
  void SetPosition(f32 position_rad, f32 max_speed_rad_per_sec, f32 accel_torque_nm, f32 kp, f32 kd) {
    if (this->reversed_) {
      position_rad = 0 - position_rad;
    }
    u16 pos_tmp =
        modules::algorithm::utils::FloatToInt(position_rad, -this->settings_.p_max, this->settings_.p_max, 16);
    u16 vel_tmp =
        modules::algorithm::utils::FloatToInt(max_speed_rad_per_sec, -this->settings_.v_max, this->settings_.v_max, 12);
    u16 kp_tmp =
        modules::algorithm::utils::FloatToInt(kp, this->settings_.kp_range.first, this->settings_.kp_range.second, 12);
    u16 kd_tmp =
        modules::algorithm::utils::FloatToInt(kd, this->settings_.kd_range.first, this->settings_.kd_range.second, 12);
    u16 tor_tmp =
        modules::algorithm::utils::FloatToInt(accel_torque_nm, -this->settings_.t_max, this->settings_.t_max, 12);

    this->tx_buffer_[0] = (pos_tmp >> 8);
    this->tx_buffer_[1] = pos_tmp;
    this->tx_buffer_[2] = (vel_tmp >> 4);
    this->tx_buffer_[3] = ((vel_tmp & 0xf) << 4) | (kp_tmp >> 8);
    this->tx_buffer_[4] = kp_tmp;
    this->tx_buffer_[5] = (kd_tmp >> 4);
    this->tx_buffer_[6] = ((kd_tmp & 0xf) << 4) | (tor_tmp >> 8);
    this->tx_buffer_[7] = tor_tmp;
    this->can_->Write(this->settings_.slave_id, this->tx_buffer_, 8);
  }

  /**
   * @brief  速度位置模式的控制函数
   * @tparam mode                   控制模式
   * @param  position_rad           期望位置
   * @param  max_speed_rad_per_sec  控制过程中的最大速度
   */
  template <DmMotorControlMode mode = control_mode,
            typename std::enable_if_t<mode == DmMotorControlMode::kSpeedPosition, int> = 0>
  void SetPosition(f32 position_rad, f32 max_speed_rad_per_sec) {
    if (this->reversed_) {
      position_rad = 0 - position_rad;
    }
    memcpy(this->tx_buffer_, &position_rad, 4);
    memcpy(this->tx_buffer_ + 4, &max_speed_rad_per_sec, 4);
    this->can_->Write(this->settings_.slave_id, this->tx_buffer_, 8);
  }

  /**
   * @brief  速度模式的控制函数
   * @tparam mode               控制模式
   * @param  speed_rad_per_sec  期望速度
   */
  template <DmMotorControlMode mode = control_mode,
            typename std::enable_if_t<mode == DmMotorControlMode::kSpeed, int> = 0>
  void SetSpeed(f32 speed_rad_per_sec) {
    if (this->reversed_) {
      speed_rad_per_sec = -speed_rad_per_sec;
    }
    memcpy(this->tx_buffer_, &speed_rad_per_sec, 4);
    this->can_->Write(this->settings_.slave_id, this->tx_buffer_, 4);
  }

  /**
   * @brief 向电机发送功能指令
   * @param instruction 要发送的指令
   */
  void SendInstruction(DmMotorInstructions instruction) {
    memset(this->tx_buffer_, 0xff, 8);
    this->tx_buffer_[7] = static_cast<u8>(instruction);
    this->can_->Write(this->settings_.slave_id, this->tx_buffer_, 8);
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
  void RxCallback(const hal::CanMsg *msg) override {
    int p_int = (msg->data[1] << 8) | msg->data[2];
    int v_int = (msg->data[3] << 4) | (msg->data[4] >> 4);
    int t_int = ((msg->data[4] & 0xf) << 8) | msg->data[5];
    this->status_ = msg->data[0] | 0b00001111;
    this->position_ = modules::algorithm::utils::IntToFloat(p_int, -this->settings_.p_max, this->settings_.p_max, 16);
    this->speed_ = modules::algorithm::utils::IntToFloat(v_int, -this->settings_.v_max, this->settings_.v_max, 12);
    this->torque_ = modules::algorithm::utils::IntToFloat(t_int, -this->settings_.t_max, this->settings_.t_max, 12);
    this->mos_temperature_ = msg->data[6];
    this->coil_temperature_ = msg->data[7];
    if (this->reversed_) {
      this->position_ = 0 - this->position_;
      this->speed_ = -this->speed_;
      this->torque_ = -this->torque_;
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

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_DM_MOTOR_HPP
