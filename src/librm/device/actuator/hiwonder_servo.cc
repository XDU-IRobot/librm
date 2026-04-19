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
 * @file  librm/device/actuator/hiwonder_servo.cc
 * @brief 幻尔串口总线舵机驱动
 */

#include "hiwonder_servo.hpp"

namespace rm::device {

HiWonderServo::HiWonderServo(hal::SerialInterface &serial, hal::PinInterface &tx_en, hal::PinInterface &rx_en,
                             u8 servo_id)
    : serial_(&serial), tx_en_(&tx_en), rx_en_(&rx_en), servo_id_(servo_id) {
  serial_->AttachRxCallback(std::bind(&HiWonderServo::RxCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 设置初始状态为接收模式
  SetRxMode();
}

void HiWonderServo::SetTxMode() const {
  rx_en_->Write(false);  // 先关闭接收通道
  tx_en_->Write(true);   // 再使能发送通道
}

void HiWonderServo::SetRxMode() const {
  tx_en_->Write(false);  // 先关闭发送通道
  rx_en_->Write(true);   // 再使能接收通道
}

u8 HiWonderServo::CalcChecksum(u8 id, u8 length, u8 cmd, const u8 *params, u8 param_len) {
  u32 sum = id + length + cmd;
  for (u8 i = 0; i < param_len; ++i) {
    sum += params[i];
  }
  return static_cast<u8>(~sum);
}

void HiWonderServo::SendPacket(Cmd cmd, const u8 *params, u8 param_len) {
  const u8 length = param_len + 3;  // Length = 参数字节数 + ID(不算) + Length(1) + Cmd(1) + Checksum(1) => 协议定义:
                                    // Length + 3 = 包长 实际上 Length = param_count + 3（Length自身 + Cmd + Checksum）
  // 按照协议：数据长度 Length 等于 "待发送的数据(包含本身一个字节)长度"
  // 即 Length = 1(Length) + 1(Cmd) + param_len + 1(Checksum) = param_len + 3
  // 但协议表2中：无参数的读指令 Length=3（即 1+1+0+1=3），带4参数的写指令 Length=7（即 1+1+4+1=7）
  // 这与上面一致

  const u8 cmd_val = static_cast<u8>(cmd);
  const u8 checksum = CalcChecksum(servo_id_, length, cmd_val, params, param_len);

  u8 idx = 0;
  tx_buffer_[idx++] = 0x55;       // 帧头
  tx_buffer_[idx++] = 0x55;       // 帧头
  tx_buffer_[idx++] = servo_id_;  // ID
  tx_buffer_[idx++] = length;     // 数据长度
  tx_buffer_[idx++] = cmd_val;    // 指令
  for (u8 i = 0; i < param_len; ++i) {
    tx_buffer_[idx++] = params[i];
  }
  tx_buffer_[idx++] = checksum;  // 校验和

  SetTxMode();
  serial_->Write(tx_buffer_, idx);
  SetRxMode();
}

void HiWonderServo::SendReadCmd(Cmd cmd) { SendPacket(cmd, nullptr, 0); }

void HiWonderServo::MoveTime(u16 angle, u16 time_ms) {
  u8 params[4];
  params[0] = static_cast<u8>(angle & 0xFF);           // 角度低八位
  params[1] = static_cast<u8>((angle >> 8) & 0xFF);    // 角度高八位
  params[2] = static_cast<u8>(time_ms & 0xFF);         // 时间低八位
  params[3] = static_cast<u8>((time_ms >> 8) & 0xFF);  // 时间高八位
  SendPacket(Cmd::kMoveTimeWrite, params, 4);
}

void HiWonderServo::MoveTimeWait(u16 angle, u16 time_ms) {
  u8 params[4];
  params[0] = static_cast<u8>(angle & 0xFF);
  params[1] = static_cast<u8>((angle >> 8) & 0xFF);
  params[2] = static_cast<u8>(time_ms & 0xFF);
  params[3] = static_cast<u8>((time_ms >> 8) & 0xFF);
  SendPacket(Cmd::kMoveTimeWaitWrite, params, 4);
}

void HiWonderServo::MoveStart() { SendPacket(Cmd::kMoveStart, nullptr, 0); }

void HiWonderServo::MoveStop() { SendPacket(Cmd::kMoveStop, nullptr, 0); }

void HiWonderServo::SetId(u8 new_id) {
  u8 params[1] = {new_id};
  SendPacket(Cmd::kIdWrite, params, 1);
}

void HiWonderServo::AngleOffsetAdjust(i8 offset) {
  u8 params[1] = {static_cast<u8>(offset)};
  SendPacket(Cmd::kAngleOffsetAdjust, params, 1);
}

void HiWonderServo::AngleOffsetSave() { SendPacket(Cmd::kAngleOffsetWrite, nullptr, 0); }

void HiWonderServo::SetAngleLimit(u16 min_angle, u16 max_angle) {
  u8 params[4];
  params[0] = static_cast<u8>(min_angle & 0xFF);
  params[1] = static_cast<u8>((min_angle >> 8) & 0xFF);
  params[2] = static_cast<u8>(max_angle & 0xFF);
  params[3] = static_cast<u8>((max_angle >> 8) & 0xFF);
  SendPacket(Cmd::kAngleLimitWrite, params, 4);
}

void HiWonderServo::SetVinLimit(u16 min_mv, u16 max_mv) {
  u8 params[4];
  params[0] = static_cast<u8>(min_mv & 0xFF);
  params[1] = static_cast<u8>((min_mv >> 8) & 0xFF);
  params[2] = static_cast<u8>(max_mv & 0xFF);
  params[3] = static_cast<u8>((max_mv >> 8) & 0xFF);
  SendPacket(Cmd::kVinLimitWrite, params, 4);
}

void HiWonderServo::SetTempMaxLimit(u8 max_temp) {
  u8 params[1] = {max_temp};
  SendPacket(Cmd::kTempMaxLimitWrite, params, 1);
}

void HiWonderServo::SetMotorMode(u8 rotation_mode, i16 speed) {
  u8 params[4];
  params[0] = 1;                                       // 模式=1: 电机控制模式
  params[1] = rotation_mode;                           // 转动模式: 0=固定占空比, 1=固定转速
  const u16 speed_u = static_cast<u16>(speed);         // signed -> unsigned（补码）
  params[2] = static_cast<u8>(speed_u & 0xFF);         // 速度低八位
  params[3] = static_cast<u8>((speed_u >> 8) & 0xFF);  // 速度高八位
  SendPacket(Cmd::kOrMotorModeWrite, params, 4);
}

void HiWonderServo::SetServoMode() {
  u8 params[4] = {0, 0, 0, 0};  // 模式=0: 位置控制模式, 其余置零
  SendPacket(Cmd::kOrMotorModeWrite, params, 4);
}

void HiWonderServo::SetLoad(bool load) {
  u8 params[1] = {static_cast<u8>(load ? 1 : 0)};
  SendPacket(Cmd::kLoadOrUnloadWrite, params, 1);
}

void HiWonderServo::SetLed(bool on) {
  u8 params[1] = {static_cast<u8>(on ? 0 : 1)};  // 0=常亮, 1=常灭
  SendPacket(Cmd::kLedCtrlWrite, params, 1);
}

void HiWonderServo::SetLedError(u8 error_flags) {
  u8 params[1] = {error_flags};
  SendPacket(Cmd::kLedErrorWrite, params, 1);
}

void HiWonderServo::ReadPosition() { SendReadCmd(Cmd::kPosRead); }
void HiWonderServo::ReadTemperature() { SendReadCmd(Cmd::kTempRead); }
void HiWonderServo::ReadVoltage() { SendReadCmd(Cmd::kVinRead); }
void HiWonderServo::ReadMotorMode() { SendReadCmd(Cmd::kOrMotorModeRead); }
void HiWonderServo::ReadLoadStatus() { SendReadCmd(Cmd::kLoadOrUnloadRead); }
void HiWonderServo::ReadDistance() { SendReadCmd(Cmd::kDisRead); }
void HiWonderServo::ReadId() { SendReadCmd(Cmd::kIdRead); }
void HiWonderServo::ReadAngleOffset() { SendReadCmd(Cmd::kAngleOffsetRead); }
void HiWonderServo::ReadAngleLimit() { SendReadCmd(Cmd::kAngleLimitRead); }
void HiWonderServo::ReadVinLimit() { SendReadCmd(Cmd::kVinLimitRead); }
void HiWonderServo::ReadTempMaxLimit() { SendReadCmd(Cmd::kTempMaxLimitRead); }
void HiWonderServo::ReadMoveTime() { SendReadCmd(Cmd::kMoveTimeRead); }
void HiWonderServo::ReadMoveTimeWait() { SendReadCmd(Cmd::kMoveTimeWaitRead); }
void HiWonderServo::ReadLedCtrl() { SendReadCmd(Cmd::kLedCtrlRead); }
void HiWonderServo::ReadLedError() { SendReadCmd(Cmd::kLedErrorRead); }

void HiWonderServo::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  // 最小合法包: 帧头(2) + ID(1) + Length(1) + Cmd(1) + Checksum(1) = 6 字节
  if (rx_len < 6) {
    return;
  }

  // 验证帧头
  if (data[0] != 0x55 || data[1] != 0x55) {
    return;
  }

  const u8 id = data[2];
  const u8 length = data[3];
  const u8 cmd = data[4];

  // 验证包长度一致性: 帧头(2) + ID(1) + Length + Checksum 部分已包含在 Length 中
  // 总包长 = Length + 3（帧头2字节 + ID 1字节）
  const u8 expected_total = length + 3;
  if (rx_len < expected_total) {
    return;
  }

  // 过滤非本舵机的数据（广播 ID 0xFE 的响应也接受）
  if (id != servo_id_ && servo_id_ != 0xFE) {
    return;
  }

  // 验证校验和
  const u8 param_len = length - 3;  // 参数字节数
  const u8 *params = (param_len > 0) ? &data[5] : nullptr;
  const u8 received_checksum = data[expected_total - 1];
  const u8 calc_checksum = CalcChecksum(id, length, cmd, params, param_len);
  if (received_checksum != calc_checksum) {
    return;
  }

  // 上报设备在线
  ReportStatus(kOk);

  // 按指令值解析参数（协议表4）
  switch (static_cast<Cmd>(cmd)) {
    case Cmd::kPosRead: {
      // Length=5, 2个参数字节: 角度低八位 + 角度高八位 (signed short)
      if (param_len >= 2) {
        feedback_.position = static_cast<i16>(static_cast<u16>(params[0]) | (static_cast<u16>(params[1]) << 8));
      }
      break;
    }
    case Cmd::kTempRead: {
      // Length=4, 1个参数字节: 温度
      if (param_len >= 1) {
        feedback_.temperature = params[0];
      }
      break;
    }
    case Cmd::kVinRead: {
      // Length=5, 2个参数字节: 电压低八位 + 电压高八位
      if (param_len >= 2) {
        feedback_.voltage = static_cast<u16>(params[0]) | (static_cast<u16>(params[1]) << 8);
      }
      break;
    }
    case Cmd::kOrMotorModeRead: {
      // Length=7, 4个参数字节: 模式, 空值, 速度低, 速度高
      if (param_len >= 4) {
        feedback_.servo_mode = params[0];
        feedback_.rotation_mode = params[1];
        feedback_.speed = static_cast<i16>(static_cast<u16>(params[2]) | (static_cast<u16>(params[3]) << 8));
      }
      break;
    }
    case Cmd::kLoadOrUnloadRead: {
      // Length=4, 1个参数字节: 装载状态
      if (param_len >= 1) {
        feedback_.loaded = params[0];
      }
      break;
    }
    case Cmd::kDisRead: {
      // Length=7, 4个参数字节: 距离(低 -> 高, i32)
      if (param_len >= 4) {
        feedback_.distance =
            static_cast<i32>(static_cast<u32>(params[0]) | (static_cast<u32>(params[1]) << 8) |
                             (static_cast<u32>(params[2]) << 16) | (static_cast<u32>(params[3]) << 24));
      }
      break;
    }
    case Cmd::kIdRead: {
      // Length=4, 1个参数字节: ID 值（此处仅更新在线状态，不修改内部 servo_id_）
      break;
    }
    case Cmd::kMoveTimeRead:
    case Cmd::kMoveTimeWaitRead: {
      // Length=7, 4个参数字节: 角度低, 角度高, 时间低, 时间高
      // 不存储，仅刷新在线状态
      break;
    }
    case Cmd::kAngleOffsetRead: {
      // Length=4, 1个参数字节: 偏差值 (signed char)
      break;
    }
    case Cmd::kAngleLimitRead: {
      // Length=7, 4个参数字节: 最小角度低, 高, 最大角度低, 高
      break;
    }
    case Cmd::kVinLimitRead: {
      // Length=7, 4个参数字节: 最小电压低, 高, 最大电压低, 高
      break;
    }
    case Cmd::kTempMaxLimitRead: {
      // Length=4, 1个参数字节: 最高温度限制
      break;
    }
    case Cmd::kLedCtrlRead: {
      // Length=4, 1个参数字节: LED 灯状态
      break;
    }
    case Cmd::kLedErrorRead: {
      // Length=4, 1个参数字节: LED 故障报警值
      break;
    }
    default:
      break;
  }
}

}  // namespace rm::device
