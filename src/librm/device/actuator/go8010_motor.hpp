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
 * @file  librm/device/actuator/go8010_motor.hpp
 * @brief GO8010电机类库
 * @todo  补充注释
 */

#ifndef LIBRM_DEVICE_ACTUATOR_GO8010_MOTOR_HPP
#define LIBRM_DEVICE_ACTUATOR_GO8010_MOTOR_HPP

#include "librm/device/actuator/unitree_motor.hpp"
#include "librm/hal/serial.hpp"
#include "librm/core/typedefs.hpp"
#include "librm/hal/serial_interface.hpp"

namespace rm::device {

class Go8010Motor : public Device {
 public:
  struct ControlParam {
    f32 tau;
    f32 vel;
    f32 pos;
    f32 kp;
    f32 kd;
  };

#pragma pack(1)

  /**
   * @brief 电机模式控制信息
   *
   */
  struct RIS_Mode {
    uint8_t id : 4;      // 电机ID: 0,1...,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status : 3;  // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t none : 1;    // 保留位
  };  // 控制模式 1Byte

  /**
   * @brief 电机状态控制信息
   *
   */
  struct RIS_Comd {
    int16_t tau_des;  // 期望关节输出扭矩 unit: N.m     (q8)
    int16_t vel_des;  // 期望关节输出速度 unit: rad/s   (q7)
    int32_t pos_des;  // 期望关节输出位置 unit: rad     (q15)
    uint16_t k_pos;   // 期望关节刚度系数 unit: 0.0-1.0 (q15)
    uint16_t k_spd;   // 期望关节阻尼系数 unit: 0.0-1.0 (q15)
  };  // 控制参数 12Byte

  /**
   * @brief 电机状态反馈信息
   *
   */
  struct RIS_Fbk {
    int16_t tau;          // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t vel;          // 实际关节输出速度 unit: rad/s   (q7)
    int32_t pos;          // 实际关节输出位置 unit: W       (q15)
    int8_t temp;          // 电机温度: -128~127°C 90°C时触发温度保护
    uint8_t MError : 3;   // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force : 12;  // 足端气压传感器数据 12bit (0-4095)
    uint8_t none : 1;     // 保留位
  };  // 状态数据 11Byte

#pragma pack()

#pragma pack(1)

  /**
   * @brief 控制数据包格式
   *
   */
  struct ControlData {
    uint8_t head[2];  // 包头         2Byte
    RIS_Mode mode;    // 电机控制模式  1Byte
    RIS_Comd comd;    // 电机期望数据 12Byte
    uint16_t CRC16;   // CRC          2Byte
  };  // 主机控制命令     17Byte

  /**
   * @brief 电机反馈数据包格式
   *
   */
  struct MotorData {
    uint8_t head[2];  // 包头         2Byte
    RIS_Mode mode;    // 电机控制模式  1Byte
    RIS_Fbk fbk;      // 电机反馈数据 11Byte
    uint16_t CRC16;   // CRC          2Byte
  };  // 电机返回数据     16Byte

  struct SendData {
    ControlData motor_send_data;

    unsigned short id;
    unsigned short mode;

    float tau;
    float vel;
    float pos;
    float kp;
    float kd;
  };

  struct ReceiveData {
    MotorData motor_recv_data;

    bool correct;

    unsigned short id;
    unsigned short mode;

    float tau;
    float vel;
    float pos;
    float temp;
    unsigned short MError;
    unsigned short force;
  };

#pragma pack()

 public:
  Go8010Motor(hal::SerialInterface &serial, u8 motor_id = 0x0);
  ~Go8010Motor() = default;

  void SetTau(f32 tau);

  void SendCommend();

  void RxCallback(const std::vector<u8> &data, u16 rx_len);

  [[nodiscard]] f32 tau() { return this->recv_data_.tau / 6.33f; }
  [[nodiscard]] f32 vel() { return this->recv_data_.vel / 6.33f; }
  [[nodiscard]] f32 pos() { return this->recv_data_.pos / 6.33f; }

 private:
  void SetParam(const SendData &send_data);

 private:
  hal::SerialInterface *serial_;

  SendData send_data_;
  ReceiveData recv_data_;

  u8 tx_buffer_[17]{0};
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_GO8010_MOTOR_HPP
