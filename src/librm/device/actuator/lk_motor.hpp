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
 * @file  librm/device/actuator/lk_motor.hpp
 * @brief 瓴控电机类库
 * @todo  rs485模式
 * @note  http://www.lkmotor.cn/Download.aspx?ClassID=47
 */

#ifndef LIBRM_DEVICE_ACTUATOR_LK_MOTOR_HPP
#define LIBRM_DEVICE_ACTUATOR_LK_MOTOR_HPP

#include <cstring>
#include <cmath>

#include "librm/device/can_device.hpp"
#include "librm/modules/utils.hpp"

namespace rm::device {

/**
 * @brief 瓴控电机型号系列
 * @warning
 * 瓴控的电机驱动有一个已知的bug：发给电机的命令可能会滞后一帧，复现条件不明，所以在需要保证电机收到某一条命令的情况下（比如使能或者失能的时候），尽量把命令多发几次
 */
enum class LkMotorType {
  kMS,  ///< MS系列电机
  kMF,  ///< MF系列电机
  kMH,  ///< MH系列电机
  kMG,  ///< MG系列电机
};

/**
 * @brief  瓴控电机类库
 * @tparam type 电机型号系列
 * @tparam encoder_bits 编码器位数，仅支持15位和18位
 */
template <LkMotorType type, usize encoder_bits>
class LkMotor : public CanDevice {
  static_assert(encoder_bits == 15 || encoder_bits == 18, "Unsupported encoder bits");

 public:
  /**
   * @brief 电机命令对应的命令码
   */
  enum Instruction : u8 {
    kDisable = 0x80,                      ///< 电机关闭
    kEnable = 0x88,                       ///< 电机运行
    kStop = 0x81,                         ///< 电机停止
    kOpenLoopControl = 0xa0,              ///< 电机开环控制，仅在MS型号电机上实现
    kTorqueControl = 0xa1,                ///< 电机力矩控制，MS型号电机不支持
    kSpeedControl = 0xa2,                 ///< 电机速度控制
    kPositionControl1Multi = 0xa3,        ///< 电机多圈位置控制，不限速
    kPositionControl2Multi = 0xa4,        ///< 电机多圈位置控制，限速
    kPositionControl1 = 0xa5,             ///< 电机单圈位置控制，不限速
    kPositionControl2 = 0xa6,             ///< 电机单圈位置控制，限速
    kPositionControl1Incremental = 0xa7,  ///< 电机增量位置控制，不限速
    kPositionControl2Incremental = 0xa8,  ///< 电机增量位置控制，限速
    kReadPidParam = 0x30,                 ///< 读取电机PID参数
    kWritePidParamRam = 0x31,             ///< 写入电机PID参数到RAM，掉电失效
    kWritePidParamRom = 0x32,             ///< 写入电机PID参数到EEPROM，掉电保存
    kReadAccel = 0x33,                    ///< 读取电机加速度
    kWriteAccelRam = 0x34,                ///< 写入电机加速度到RAM，掉电失效
    kReadEncoder = 0x90,                  ///< 读取电机编码器值
    kSetEncoderZero = 0x91,               ///< 设置电机编码器零点到EEPROM
    kSetCurrentAsEncoderZero = 0x19,      ///< 设置当前电机编码器值为零点到EEPROM
    kReadPositionMulti = 0x92,            ///< 读取电机多圈位置值
    kReadPosition = 0x94,                 ///< 读取电机单圈位置
    // kClearPosition = 0x95,             ///< 清除电机位置值到0  ///< 文档里写未实现
    kReadState1 = 0x9a,      ///< 读取电机状态1和错误标志
    kClearErrorFlag = 0x9b,  ///< 清除电机错误标志
    kReadState2 = 0x9c,      ///< 读取电机状态2
    kReadState3 = 0x9d,      ///< 读取电机状态3
  };

 public:
  LkMotor() = delete;
  LkMotor(LkMotor &&other) noexcept = default;
  ~LkMotor() override {
    // why repeat: see warning in class description
    SendInstruction(Instruction::kDisable);
    SendInstruction(Instruction::kDisable);
    SendInstruction(Instruction::kDisable);
  }

  /**
   * @param can         CAN总线对象
   * @param id          电机ID
   * @param reversed    是否反转
   */
  LkMotor(hal::CanInterface &can, usize id, bool reversed = false)
      : CanDevice(can, 0x140 + id), id_{id}, reversed_{reversed} {}

  /**
   * @brief  开环控制函数，仅MS型号电机支持
   * @param  power      开环电压
   */
  template <LkMotorType t = type, std::enable_if_t<t == LkMotorType::kMS, int> = 0>
  void SetPower(f32 power) {
    if (reversed_) {
      power = -power;
    }
    i16 power_cmd = modules::Clamp(power, -850.f, 850.f);
    std::memset(&tx_buffer_[1], 0, sizeof(tx_buffer_) - 1);
    tx_buffer_[0] = Instruction::kOpenLoopControl;
    tx_buffer_[4] = *(u8 *)(&power_cmd);
    tx_buffer_[5] = *((u8 *)(&power_cmd) + 1);
    can_->Write(0x140 + id_, tx_buffer_, 8);
  }

  /**
   * @brief  力矩控制函数，MS型号电机不支持
   * @param  torque     期望力矩，单位：Nm
   */
  template <LkMotorType t = type, std::enable_if_t<t != LkMotorType::kMS, int> = 0>
  void SetTorque(f32 torque_nm) {
    if (reversed_) {
      torque_nm = -torque_nm;
    }
    i16 torque_cmd;
    if constexpr (type == LkMotorType::kMF) {
      torque_nm = modules::Clamp(torque_nm, -16.5f, 16.5f);
      torque_cmd = modules::Map(torque_nm, -16.5f, 16.5f, -2048, 2048);
    } else if constexpr (type == LkMotorType::kMG) {
      torque_nm = modules::Clamp(torque_nm, -33.f, 33.f);
      torque_cmd = modules::Map(torque_nm, -33.f, 33.f, -2048, 2048);
    }
    std::memset(&tx_buffer_[1], 0, sizeof(tx_buffer_) - 1);
    tx_buffer_[0] = Instruction::kTorqueControl;
    tx_buffer_[4] = *(u8 *)(&torque_cmd);
    tx_buffer_[5] = *((u8 *)(&torque_cmd) + 1);
    can_->Write(0x140 + id_, tx_buffer_, 8);
  }

  // TODO: 还有一大堆控制模式懒得写了，有需要再说

  /**
   * @brief 向电机发送功能指令
   * @param instruction 要发送的指令
   */
  void SendInstruction(Instruction instruction) {
    switch (instruction) {
      case kEnable: {
        feedback_.enabled = true;
        break;
      }
      case kDisable: {
        feedback_.enabled = false;
        break;
      }
      default: {
        break;
      }
    }
    std::memset(&tx_buffer_[1], 0, sizeof(tx_buffer_) - 1);
    tx_buffer_[0] = instruction;
    can_->Write(0x140 + id_, tx_buffer_, 8);
  }

  /** 取值函数 **/
  [[nodiscard]] const auto &feedback() const { return feedback_; }
  /*************/

 private:
  /**
   * @brief CAN回调函数，解码收到的反馈报文
   * @param msg   收到的报文
   */
  void RxCallback(const hal::CanMsg *msg) override {
    Heartbeat();
    switch (msg->data[0]) {
      case Instruction::kOpenLoopControl: {
        feedback_.temperature = *(i8 *)(&msg->data[1]);
        feedback_.power = (msg->data[2] | (msg->data[3] << 8));
        feedback_.speed_rad_ps = (msg->data[4] | (msg->data[5] << 8));
        const u16 encoder_raw = (msg->data[6] | (msg->data[7] << 8));
        if constexpr (encoder_bits == 15) {
          feedback_.position_rad = modules::Map(encoder_raw, 0, 32767, 0.f, 2.f * M_PI);
        } else if constexpr (encoder_bits == 18) {
          feedback_.position_rad = modules::Map(encoder_raw, 0, 65535, 0.f, 2.f * M_PI);
        }
        break;
      }
      case Instruction::kTorqueControl: {
        feedback_.temperature = *(i8 *)(&msg->data[1]);
        const i16 iq_raw = (msg->data[2] | (msg->data[3] << 8));
        if constexpr (type == LkMotorType::kMF) {
          feedback_.iq = modules::Map(iq_raw, -2048, 2048, -16.5f, 16.5f);
        } else if constexpr (type == LkMotorType::kMG) {
          feedback_.iq = modules::Map(iq_raw, -2048, 2048, -33.f, 33.f);
        }
        feedback_.speed_rad_ps = (msg->data[4] | (msg->data[5] << 8));
        const u16 encoder_raw = (msg->data[6] | (msg->data[7] << 8));
        if constexpr (encoder_bits == 15) {
          feedback_.position_rad = modules::Map(encoder_raw, 0, 32767, 0.f, 2.f * M_PI);
        } else if constexpr (encoder_bits == 18) {
          feedback_.position_rad = modules::Map(encoder_raw, 0, 65535, 0.f, 2.f * M_PI);
        }
        break;
      }
      case Instruction::kReadState1: {
        feedback_.temperature = msg->data[1];
        const u16 voltage_raw = (msg->data[3] | (msg->data[4] << 8));
        feedback_.voltage = voltage_raw / 10.f;
        *(u8 *)(&feedback_.error_state) = msg->data[7];
        break;
      }
      // TODO: 其他反馈报文，暂时用不到
      default: {
        break;
      }
    }
  }

  usize id_{};
  bool reversed_{};  ///< 是否反转
  u8 tx_buffer_[8]{0};
  struct {
    i8 temperature;
    f32 voltage;
    i16 power;         ///< MS系列电机的的开环电流值
    f32 iq;            ///< 转矩电流
    i16 speed_rad_ps;  ///< 电机速度，单位：rad/s
    f32 position_rad;  ///< 电机位置，单位：rad
    bool enabled;
    struct {
      u8 low_voltage : 1;       ///< 低压保护
      u8 over_voltage : 1;      ///< 高压保护
      u8 driver_over_temp : 1;  ///< 驱动过温
      u8 motor_over_temp : 1;   ///< 电机过温
      u8 over_current : 1;      ///< 电机过流
      u8 short_circuit : 1;     ///< 电机短路
      u8 stall : 1;             ///< 电机堵转
      u8 input_timeout : 1;     ///< 输入信号丢失超时
    } error_state;
  } feedback_{};
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_LK_MOTOR_HPP
