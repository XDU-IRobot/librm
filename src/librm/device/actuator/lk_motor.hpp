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
 * @file  librm/device/actuator/lk_motor.hpp
 * @brief 瓴控电机类库
 * @todo  rs485模式
 * @note  http://www.lkmotor.cn/Download.aspx?ClassID=47
 * @note  http://www.lkmotor.cn/upload/20250106105848f.pdf
 */

#ifndef LIBRM_DEVICE_ACTUATOR_LK_MOTOR_HPP
#define LIBRM_DEVICE_ACTUATOR_LK_MOTOR_HPP

#include <cstring>
#include <cmath>

#include "librm/device/can_device.hpp"
#include "librm/modules/angle.hpp"
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
    kReadControlParam = 0x30,             ///< 读取电机控制参数
    kWriteControlParamRam = 0x31,         ///< 写入电机控制参数到驱动RAM，掉电失效
    kWriteControlParamRom = 0x32,         ///< 写入电机控制参数到驱动EEPROM，掉电保存
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

  /**
   * @brief
   */
  enum ControlParamType : u8 {
    kPositionPid = 0x0a,
    kSpeedPid = 0x0b,
    kCurrentPid = 0x0c,
    kInputTorqueLimit = 0x1e,
    kInputSpeedLimit = 0x20,
    kInputPositionLimit = 0x22,
    kInputCurrentRamp = 0x24,
    kInputSpeedRamp = 0x26,
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
   * @param  torque_nm     期望力矩，单位：Nm
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

  /**
   * @brief  速度控制函数
   * @param  speed_rad_ps 期望速度，单位：rad/s
   */
  void SetSpeed(f32 speed_rad_ps) {
    if (reversed_) {
      speed_rad_ps = -speed_rad_ps;
    }
    const float dps_100x = speed_rad_ps * 180.f / M_PI * 100.f;  // rad/s -> dps, 0.01dps/LSB
    const i32 power_cmd = dps_100x;  // 上位机里还有一个Max Speed值限制，这个值如果大于上位机里设置的值，会被电机限制掉
    std::memset(&tx_buffer_[1], 0, sizeof(tx_buffer_) - 1);
    tx_buffer_[0] = Instruction::kSpeedControl;
    tx_buffer_[4] = *(u8 *)(&power_cmd);
    tx_buffer_[5] = *((u8 *)(&power_cmd) + 1);
    tx_buffer_[6] = *((u8 *)(&power_cmd) + 2);
    tx_buffer_[7] = *((u8 *)(&power_cmd) + 3);
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

  /**
   * @brief 查询电机驱动内部的控制参数，参数类型见ControlParamType枚举定义
   * @param param_type 要查询的参数
   */
  void ReadControlParam(ControlParamType param_type) {
    tx_buffer_[0] = Instruction::kReadControlParam;
    tx_buffer_[1] = param_type;
    std::memset(&tx_buffer_[2], 0, sizeof(tx_buffer_) - 2);
    can_->Write(0x140 + id_, tx_buffer_, 8);
  }

  /**
   * @brief 查询电机内部的控制参数
   */
  void ReadControlParam() {
    for (const auto param_type : {
             ControlParamType::kPositionPid,
             ControlParamType::kSpeedPid,
             ControlParamType::kCurrentPid,
             ControlParamType::kInputTorqueLimit,
             ControlParamType::kInputSpeedLimit,
             ControlParamType::kInputPositionLimit,
             ControlParamType::kInputCurrentRamp,
             ControlParamType::kInputCurrentLimit,
         }) {
      ReadControlParam(param_type);
      Sleep(std::chrono::microseconds(500));
    }
  }

  /** 取值函数 **/
  [[nodiscard]] const auto &feedback() const { return feedback_; }
  /*************/

 private:
  /**
   * @brief CAN回调函数，解码收到的反馈报文
   * @param msg   收到的报文
   */
  void RxCallback(const hal::CanFrame *msg) override {
    ReportStatus(kOk);
    switch (msg->data[0]) {
      case Instruction::kOpenLoopControl:
      case Instruction::kSpeedControl: {
        feedback_.temperature = *(i8 *)(&msg->data[1]);
        feedback_.power = (msg->data[2] | (msg->data[3] << 8));
        feedback_.speed_rad_ps = (i16)(msg->data[4] | (msg->data[5] << 8)) / 180.f * M_PI;
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
        feedback_.speed_rad_ps = (i16)(msg->data[4] | (msg->data[5] << 8)) / 180.f * M_PI;
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
      case Instruction::kReadControlParam: {
        switch (msg->data[1]) {
          case ControlParamType::kPositionPid: {
            feedback_.control_param.position_kp = (msg->data[2] | (msg->data[3] << 8));
            feedback_.control_param.position_ki = (msg->data[4] | (msg->data[5] << 8));
            feedback_.control_param.position_kd = (msg->data[6] | (msg->data[7] << 8));
            break;
          }
          case ControlParamType::kSpeedPid: {
            feedback_.control_param.speed_kp = (msg->data[2] | (msg->data[3] << 8));
            feedback_.control_param.speed_ki = (msg->data[4] | (msg->data[5] << 8));
            feedback_.control_param.speed_kd = (msg->data[6] | (msg->data[7] << 8));
            break;
          }
          case ControlParamType::kCurrentPid: {
            feedback_.control_param.current_kp = (msg->data[2] | (msg->data[3] << 8));
            feedback_.control_param.current_ki = (msg->data[4] | (msg->data[5] << 8));
            feedback_.control_param.current_kd = (msg->data[6] | (msg->data[7] << 8));
          }
          default: {
            break;
          }
        }
        break;
      }
      case Instruction::kReadPositionMulti: {
        // data 的元素类型是 u8，必须逐字节提升到 64 位再移位。
        // 否则会被整型提升到 int，移位量 >= 32 属于未定义行为。
        const u64 pos_multi_raw = (                   //
            static_cast<u64>(msg->data[1]) |          //
            (static_cast<u64>(msg->data[2]) << 8) |   //
            (static_cast<u64>(msg->data[3]) << 16) |  //
            (static_cast<u64>(msg->data[4]) << 24) |  //
            (static_cast<u64>(msg->data[5]) << 32) |  //
            (static_cast<u64>(msg->data[6]) << 40) |  //
            (static_cast<u64>(msg->data[7]) << 48)    //
        );
        feedback_.position_multi_rad = (static_cast<i64>(pos_multi_raw) / 100.f) / 180.f * M_PI;
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
    i16 power;               ///< MS系列电机的的开环电流值
    f32 iq;                  ///< 转矩电流
    f32 speed_rad_ps;        ///< 电机速度，单位：rad/s
    f32 position_rad;        ///< 电机位置，单位：rad
    f32 position_multi_rad;  ///< 多圈计数绝对位置，单位：rad
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
    struct {
      u16 position_kp;           ///< 位置环 0x0a
      u16 position_ki;           //
      u16 position_kd;           //
      u16 speed_kp;              ///< 速度环 0x0b
      u16 speed_ki;              //
      u16 speed_kd;              //
      u16 current_kp;            ///< 电流环 0x0c
      u16 current_ki;            //
      u16 current_kd;            //
      i16 input_torque_limit;    ///< 力矩电流限制 0x1e
      i32 input_speed_limit;     ///< 输入速度限制 0x20
      i32 input_position_limit;  ///< 输入角度限制 0x22
      i32 input_current_ramp;    ///< 电流斜率 0x24
      i32 input_speed_ramp;      ///< 速度斜率 0x26
    } control_param;             ///< 电机驱动内部的控制参数
  } feedback_{};
};

template <usize encoder_bits>
using LkMotorMS = LkMotor<LkMotorType::kMS, encoder_bits>;
template <usize encoder_bits>
using LkMotorMF = LkMotor<LkMotorType::kMF, encoder_bits>;
template <usize encoder_bits>
using LkMotorMH = LkMotor<LkMotorType::kMH, encoder_bits>;
template <usize encoder_bits>
using LkMotorMG = LkMotor<LkMotorType::kMG, encoder_bits>;

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_LK_MOTOR_HPP
