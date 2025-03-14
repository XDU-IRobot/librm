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
 * @file  librm/device/actuator/directdrive_motor.cc
 * @brief 本末电机类库
 * @todo  这个驱动是用p1010b_111写的，其他型号的本末电机没有测试过
 * @todo  rs485模式
 * @todo  还有一些杂七杂八的什么主动查询、设置反馈方式都没写，暂时用不上
 */

#include "directdrive_motor.hpp"

#include <algorithm>

#include "librm/core/time.hpp"

namespace rm::device {

std::unordered_map<hal::CanInterface *, DirectDriveMotor::TxBufferTable> DirectDriveMotor::tx_buffer_table_{};

/**
 * @brief 电机反馈处理回调函数
 */
void DirectDriveMotor::RxCallback(const hal::CanMsg *msg) {
  if (msg->rx_std_id == 0x50 + id_) {
    const u16 iq_temp = (msg->data[2] << 8) | msg->data[3];
    const u16 rpm_temp = (msg->data[0] << 8) | msg->data[1];
    feedback_.rpm = (*reinterpret_cast<const i16 *>(&rpm_temp)) / 10.f;
    feedback_.iq = (*reinterpret_cast<const i16 *>(&iq_temp)) / 100.f;
    feedback_.encoder = (msg->data[4] << 8) | msg->data[5];
    feedback_.master_voltage = static_cast<f32>((msg->data[6] << 8) | msg->data[7]) / 10.f;

    // TODO
  } else if (msg->rx_std_id == 0x60 + id_) {
  } else if (msg->rx_std_id == 0x70 + id_) {
  } else if (msg->rx_std_id == 0x80 + id_) {
  } else if (msg->rx_std_id == 0x90 + id_) {
  } else if (msg->rx_std_id == 0xa0 + id_) {
  } else if (msg->rx_std_id == 0xb0 + id_) {
  }
}

/**
 * @brief 向所有电机发送一次当前的控制指令，让它反馈一次数据
 */
void DirectDriveMotor::Heartbeat() {
  for (auto &[can, buffer] : tx_buffer_table_) {
    can->Write(TxCommandId::kDrive1234, buffer.command_data, 8);
    can->Write(TxCommandId::kDrive5678, &buffer.command_data[8], 8);
  }
}

/**
 * @brief 使能/失能电机
 * @param enable 使能/失能
 */
void DirectDriveMotor::Enable(bool enable) {
  tx_buffer_table_.at(can_).mode_control_data[id_ - 1] = enable ? 0x02 : 0x01;
  can_->Write(TxCommandId::kModeControl, tx_buffer_table_.at(can_).mode_control_data, 8);
}

/**
 * @brief 重置某条CAN总线上的所有电机
 * @param can CAN总线
 */
void DirectDriveMotor::ResetAllOn(hal::CanInterface &can) {
  const u8 tx_buf[8]{0x1u, 0, 0, 0, 0, 0, 0, 0};
  can.Write(TxCommandId::kSoftwareReset, tx_buf, 8);
}

/**
 * @brief 重置所有电机
 */
void DirectDriveMotor::ResetAll() {
  const u8 tx_buf[8]{0x1u, 0, 0, 0, 0, 0, 0, 0};
  for (auto &[can, _] : tx_buffer_table_) {
    can->Write(TxCommandId::kSoftwareReset, tx_buf, 8);
  }
}

/**
 * @brief 发送控制指令，和DJI的电机一样，调用Set函数后需要调用这个函数才能真正发送指令
 */
void DirectDriveMotor::SendCommand() {
  for (auto &[can, buffer] : tx_buffer_table_) {
    if (buffer.command_data_updated_flag_1234) {
      can->Write(TxCommandId::kDrive1234, buffer.command_data, 8);
      buffer.command_data_updated_flag_1234 = false;
    }
    if (buffer.command_data_updated_flag_5678) {
      can->Write(TxCommandId::kDrive5678, &buffer.command_data[8], 8);
      buffer.command_data_updated_flag_5678 = false;
    }
  }
}

/**
 * @brief 设置电机控制指令
 * @param control_value 控制值，单位根据电机模式而定(V, A, rpm, 圈)
 */
void DirectDriveMotor::Set(f32 control_value) {
  i16 control_value_int;
  if (current_mode_ == Mode::kUnknown) {
    using namespace std::chrono_literals;
    Enable(false);
    rm::core::time::Sleep(2ms);
    SetParameter(Parameters::Mode(Mode::kCurrent));
    rm::core::time::Sleep(2ms);
    Enable(true);
    current_mode_ = Mode::kCurrent;
  }
  switch (current_mode_) {
    case Mode::kVoltageOpenloop: {
      control_value = std::clamp(control_value, -24.f, 24.f);
      control_value_int = control_value * 100;
      break;
    }
    case Mode::kCurrent: {
      control_value = std::clamp(control_value, -75.f, 75.f);
      control_value_int = control_value * 100;
      break;
    }
    case Mode::kSpeed: {
      control_value = std::clamp(control_value, -160.f, 160.f);
      control_value_int = control_value * 10;
      break;
    }
    case Mode::kPosition: {
      control_value = std::clamp(control_value, -50.f, 50.f);
      control_value_int = control_value * 100;
      break;
    }
    default:
      return;
  }
  tx_buffer_table_.at(can_).command_data[(id_ - 1) * 2] = (control_value_int >> 8);
  tx_buffer_table_.at(can_).command_data[(id_ - 1) * 2 + 1] = control_value_int;
  if (id_ < 5) {
    tx_buffer_table_.at(can_).command_data_updated_flag_1234 = true;
  } else {
    tx_buffer_table_.at(can_).command_data_updated_flag_5678 = true;
  }
}

/**
 * @brief   切换到指定的控制模式，并设置控制值
 *
 * @warning 电机在切换模式时会失能一小段时间，务必注意
 *
 * @param   control_value 控制值
 * @param   mode          控制模式
 */
void DirectDriveMotor::Set(f32 control_value, Mode mode) {
  if (current_mode_ != mode) {
    using namespace std::chrono_literals;
    Enable(false);
    rm::core::time::Sleep(2ms);
    SetParameter(Parameters::Mode(mode));
    rm::core::time::Sleep(2ms);
    Enable(true);
  }
  current_mode_ = mode;
  Set(control_value);
}

}  // namespace rm::device