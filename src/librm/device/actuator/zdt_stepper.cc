
#include "zdt_stepper.hpp"

namespace rm::device {

std::unordered_map<
    rm::hal::SerialInterface *,
    std::unordered_map<
        rm::u8, std::function<void(const std::vector<rm::u8> &, rm::u16)>>>
    rx_callback_map_emotor;

ZdtStepper::ZdtStepper(hal::SerialInterface &serial, u8 motor_id) : serial_(&serial) {
  rx_callback_map_emotor[&serial][motor_id] = std::bind(
      &ZdtStepper::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  serial_->AttachRxCallback(rx_callback_map_emotor[&serial][motor_id]);

  motor_id_ = motor_id;
}

void ZdtStepper::MotorVelCtrl(u8 addr, u8 dir, u16 vel, u8 acc, bool snF) {
  static uint8_t cmd[8] = {0};

  // 装载命令
  cmd[0] = addr;                // 地址
  cmd[1] = 0xF6;                // 功能码
  cmd[2] = dir;                 // 方向
  cmd[3] = (uint8_t)(vel >> 8); // 速度(RPM)高8位字节
  cmd[4] = (uint8_t)(vel >> 0); // 速度(RPM)低8位字节
  cmd[5] = acc;                 // 加速度，注意：0是直接启动
  cmd[6] = snF;                 // 多机同步运动标志
  cmd[7] = 0x6B;                // 校验字节

  // 发送命令
  serial_->Write(cmd, 8);
}

void ZdtStepper::MotorPosCtrl(u8 addr, u8 dir, u16 vel, u8 acc, u32 clk, bool raF,
                         bool snF) {
  static uint8_t cmd[13] = {0};

  // 装载命令
  cmd[0] = addr;                 // 地址
  cmd[1] = 0xFD;                 // 功能码
  cmd[2] = dir;                  // 方向
  cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
  cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
  cmd[5] = acc;                  // 加速度，注意：0是直接启动
  cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
  cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
  cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
  cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
  cmd[10] = raF;  // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] = snF;  // 多机同步运动标志，false为不启用，true为启用
  cmd[12] = 0x6B; // 校验字节

  // 发送命令
  serial_->Write(cmd, 13);
}

void ZdtStepper::MotorSyncCtrl(u8 addr) {
  static uint8_t cmd[4] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0xFF; // 功能码
  cmd[2] = 0x66; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  serial_->Write(cmd, 4);
}

void ZdtStepper::ReadPos(u8 addr) {
  static uint8_t cmd[3] = {0};

  // 装载命令
  cmd[0] = addr;
  cmd[1] = 0x36;
  cmd[2] = 0x6B;

  // 发送命令
  serial_->Write(cmd, 3);
}

void ZdtStepper::ReadVel(u8 addr) {
  static uint8_t cmd[3] = {0};

  // 装载命令
  cmd[0] = addr;
  cmd[1] = 0x35;
  cmd[2] = 0x6B;

  // 发送命令
  serial_->Write(cmd, 3);
}

void ZdtStepper::RxCallback(const std::vector<rm::u8> &data, rm::u16 rx_len) {
  if (data[0] == motor_id_ && data[1] == 0x36 && rx_len == 8) {
    pos_ = static_cast<uint32_t>((static_cast<uint32_t>(data[3]) << 24) |
                                 (static_cast<uint32_t>(data[4]) << 16) |
                                 (static_cast<uint32_t>(data[5]) << 8) |
                                 (static_cast<uint32_t>(data[6]) << 0));
    motor_pos_ = (float)pos_ * 360.0f / 65536.0f;
    if (data[2]) {
      motor_pos_ = -motor_pos_;
    }
  } else if (data[0] == motor_id_ && data[1] == 0x35 && rx_len == 6) {
    vel_ = static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 8) |
                                 (static_cast<uint16_t>(data[4]) << 0));
    motor_vel_ = vel_;
    if (data[2]) {
      motor_vel_ = -motor_vel_;
    }
  }
}

}