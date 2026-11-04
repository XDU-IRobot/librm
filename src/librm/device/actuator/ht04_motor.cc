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
 * @file  librm/device/actuator/ht04_motor.cc
 * @brief HT-04电机（MIT CAN协议）类库
 */

#include "ht04_motor.hpp"

#include <algorithm>
#include <cstring>

#include "librm/modules/utils.hpp"

namespace rm::device {

Ht04Motor::Ht04Motor(hal::CanInterface &can, Settings settings, bool reversed)
    : CanDevice(can), settings_(settings), reversed_(reversed) {}

void Ht04Motor::SetMitCommand(f32 position_rad, f32 speed_rad_per_sec, f32 torque_ff_nm, f32 kp, f32 kd) {
  if (reversed_) {
    position_rad = -position_rad;
    speed_rad_per_sec = -speed_rad_per_sec;
    torque_ff_nm = -torque_ff_nm;
  }

  // 前馈力矩（N·m）→ 电流（A），协议帧该字段实为电流命令
  const f32 current_ff = torque_ff_nm / settings_.torque_coef;

  // 对输入参数进行限幅
  position_rad = std::clamp(position_rad, -settings_.p_max, settings_.p_max);
  speed_rad_per_sec = std::clamp(speed_rad_per_sec, -settings_.v_max, settings_.v_max);
  const f32 current_ff_clamped = std::clamp(current_ff, -settings_.t_max, settings_.t_max);
  kp = std::clamp(kp, settings_.kp_range.first, settings_.kp_range.second);
  kd = std::clamp(kd, settings_.kd_range.first, settings_.kd_range.second);

  using modules::FloatToInt;
  const u16 pos_tmp = FloatToInt(position_rad, -settings_.p_max, settings_.p_max, 16);
  const u16 vel_tmp = FloatToInt(speed_rad_per_sec, -settings_.v_max, settings_.v_max, 12);
  const u16 kp_tmp = FloatToInt(kp, settings_.kp_range.first, settings_.kp_range.second, 12);
  const u16 kd_tmp = FloatToInt(kd, settings_.kd_range.first, settings_.kd_range.second, 12);
  const u16 tor_tmp = FloatToInt(current_ff_clamped, -settings_.t_max, settings_.t_max, 12);

  // MIT协议打包格式：
  // [Position 16bit][Velocity 12bit][Kp 12bit][Kd 12bit][Torque_FF 12bit]
  tx_buffer_[0] = static_cast<u8>(pos_tmp >> 8);
  tx_buffer_[1] = static_cast<u8>(pos_tmp & 0xff);
  tx_buffer_[2] = static_cast<u8>(vel_tmp >> 4);
  tx_buffer_[3] = static_cast<u8>(((vel_tmp & 0xf) << 4) | (kp_tmp >> 8));
  tx_buffer_[4] = static_cast<u8>(kp_tmp & 0xff);
  tx_buffer_[5] = static_cast<u8>(kd_tmp >> 4);
  tx_buffer_[6] = static_cast<u8>(((kd_tmp & 0xf) << 4) | (tor_tmp >> 8));
  tx_buffer_[7] = static_cast<u8>(tor_tmp & 0xff);

  can_->Write(settings_.slave_id, tx_buffer_, 8);
}

void Ht04Motor::SendInstruction(Instruction instruction) {
  memset(tx_buffer_, 0xff, 8);
  tx_buffer_[7] = static_cast<u8>(instruction);
  can_->Write(settings_.slave_id, tx_buffer_, 8);
}

void Ht04Motor::RxCallback(const hal::CanFrame *msg) {
  // 只处理自己这一路反馈帧，其余报文一律忽略
  if (msg->std_id != settings_.master_id) {
    return;
  }
  ReportStatus(kOk);

  // 反馈报文格式：
  // Byte0: Motor ID（= slave_id）
  // Byte1: Position[15:8]
  // Byte2: Position[7:0]
  // Byte3: Velocity[11:4]
  // Byte4: Velocity[3:0] | Torque[11:8]
  // Byte5: Torque[7:0]
  const int p_int = (msg->data[1] << 8) | msg->data[2];
  const int v_int = (msg->data[3] << 4) | (msg->data[4] >> 4);
  const int t_int = ((msg->data[4] & 0xf) << 8) | msg->data[5];

  using modules::IntToFloat;
  position_ = IntToFloat(p_int, -settings_.p_max, settings_.p_max, 16);

  // 解码速度后减去零偏
  velocity_ = IntToFloat(v_int, -settings_.v_max, settings_.v_max, 12) - settings_.speed_bias;

  // 反馈帧末12bit为驱动器估算电流（A），乘以转矩系数得力矩（N·m）
  torque_ = IntToFloat(t_int, -settings_.t_max, settings_.t_max, 12) * settings_.torque_coef;

  if (reversed_) {
    position_ = -position_;
    velocity_ = -velocity_;
    torque_ = -torque_;
  }
}

}  // namespace rm::device
