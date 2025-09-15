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

namespace rm::device {

std::unordered_map<hal::CanInterface *, DirectDriveMotor::TxBufferTable> DirectDriveMotor::tx_buffer_table_{};

void DirectDriveMotor::RxCallback(const hal::CanMsg *msg) {
  if (msg->rx_std_id == 0x50 + id_) {
    feedback_.rpm = static_cast<f32>((msg->data[0] << 8) | msg->data[1]) / 10.f;
    feedback_.iq = static_cast<f32>((msg->data[2] << 8) | msg->data[3]) / 100.f;
    feedback_.encoder = (msg->data[4] << 8) | msg->data[5];
    feedback_.master_voltage = static_cast<f32>((msg->data[6] << 8) | msg->data[7]) / 10.f;
  } else if (msg->rx_std_id == 0x60 + id_) {
  } else if (msg->rx_std_id == 0x70 + id_) {
  } else if (msg->rx_std_id == 0x80 + id_) {
  } else if (msg->rx_std_id == 0x90 + id_) {
  } else if (msg->rx_std_id == 0xa0 + id_) {
  } else if (msg->rx_std_id == 0xb0 + id_) {
  }
}

void DirectDriveMotor::Enable(bool enable) {
  tx_buffer_table_.at(can_).mode_control_data[id_] = enable ? 0x02 : 0x01;
  can_->Write(TxCommandId::kModeControl, tx_buffer_table_.at(can_).mode_control_data, 8);
}

void DirectDriveMotor::ResetAllOn(hal::CanInterface &can) {
  const u8 tx_buf[8]{0x1u, 0, 0, 0, 0, 0, 0, 0};
  can.Write(TxCommandId::kSoftwareReset, tx_buf, 8);
}

void DirectDriveMotor::ResetAll() {
  const u8 tx_buf[8]{0x1u, 0, 0, 0, 0, 0, 0, 0};
  for (auto &[can, _] : tx_buffer_table_) {
    can->Write(TxCommandId::kSoftwareReset, tx_buf, 8);
  }
}

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

void DirectDriveMotor::Set(f32 control_value) {
  i16 control_value_int;
  if (current_mode_ == Mode::kUnknown) {
    SetParameter(Parameters::Mode(Mode::kCurrent));
  }
  switch (current_mode_) {
    case Mode::kVoltageOpenloop: {
      control_value = modules::algorithm::utils::absConstrain(control_value, 24.f);
      control_value_int = control_value * 100;
      break;
    }
    case Mode::kCurrent: {
      control_value = modules::algorithm::utils::absConstrain(control_value, 75.f);
      control_value_int = control_value * 100;
      break;
    }
    case Mode::kSpeed: {
      control_value = modules::algorithm::utils::absConstrain(control_value, 160.f);
      control_value_int = control_value * 10;
      break;
    }
    case Mode::kPosition: {
      control_value = modules::algorithm::utils::absConstrain(control_value, 50.f);
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

void DirectDriveMotor::Set(f32 control_value, Mode mode) {
  if (current_mode_ != mode) {
    SetParameter(Parameters::Mode(mode));
  }
  current_mode_ = mode;
  Set(control_value);
}

}  // namespace rm::device