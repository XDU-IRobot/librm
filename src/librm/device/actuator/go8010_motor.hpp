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
 * @file  librm/device/actuator/go8010_motor.hpp
 * @brief 宇树GO-M8010-6电机类库
 */

#ifndef LIBRM_DEVICE_ACTUATOR_GO8010_MOTOR_HPP
#define LIBRM_DEVICE_ACTUATOR_GO8010_MOTOR_HPP

#include "librm/core/typedefs.hpp"
#include "librm/device/device.hpp"
#include "librm/hal/serial_interface.hpp"

namespace rm::device {

class Go8010Motor : public Device {
#pragma pack(1)

  /**
   * @brief 电机模式控制信息
   *
   */
  struct RISMode {
    u8 id : 4;      ///< 电机ID: 0,1...,14 15表示向所有电机广播数据(此时无返回)
    u8 status : 3;  ///< 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    u8 none : 1;    ///< 保留位
  };  ///< 控制模式 1Byte

  /**
   * @brief 电机状态控制信息
   *
   */
  struct RISComd {
    int16_t tau_des;  ///< 期望关节输出扭矩 unit: N.m     (q8)
    int16_t vel_des;  ///< 期望关节输出速度 unit: rad/s   (q7)
    int32_t pos_des;  ///< 期望关节输出位置 unit: rad     (q15)
    u16 k_pos;        ///< 期望关节刚度系数 unit: 0.0-1.0 (q15)
    u16 k_spd;        ///< 期望关节阻尼系数 unit: 0.0-1.0 (q15)
  };  ///< 控制参数 12Byte

  /**
   * @brief 电机状态反馈信息
   *
   */
  struct RISFbk {
    int16_t tau;     ///< 实际关节输出扭矩 unit: N.m     (q8)
    int16_t vel;     ///< 实际关节输出速度 unit: rad/s   (q7)
    int32_t pos;     ///< 实际关节输出位置 unit: W       (q15)
    int8_t temp;     ///< 电机温度: -128~127°C 90°C时触发温度保护
    u8 MError : 3;   ///< 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    u16 force : 12;  ///< 足端气压传感器数据 12bit (0-4095)
    u8 none : 1;     ///< 保留位
  };  ///< 状态数据 11Byte

  /**
   * @brief 控制数据包格式
   *
   */
  struct ControlData {
    u8 head[2];    ///< 包头         2Byte
    RISMode mode;  ///< 电机控制模式  1Byte
    RISComd comd;  ///< 电机期望数据 12Byte
    u16 crc16;     ///< CRC          2Byte
  };  ///< 主机控制命令     17Byte

  /**
   * @brief 电机反馈数据包格式
   *
   */
  struct MotorData {
    u8 head[2];       ///< 包头         2Byte
    RISMode mode;     ///< 电机控制模式  1Byte
    RISFbk feedback;  ///< 电机反馈数据 11Byte
    u16 crc16;        ///< CRC          2Byte
  };  ///< 电机返回数据     16Byte

#pragma pack()

 public:
  Go8010Motor(hal::SerialInterface &serial, u8 motor_id);
  ~Go8010Motor() override = default;

  void SetMitCommand(f32 position_rad, f32 speed_rad_per_sec, f32 torque_ff_nm, f32 kp, f32 kd);

  void RxCallback(const std::vector<u8> &data, u16 rx_len);

  [[nodiscard]] f32 tau() const { return this->recv_data_.tau / 6.33f; }
  [[nodiscard]] f32 vel() const { return this->recv_data_.vel / 6.33f; }
  [[nodiscard]] f32 pos() const { return this->recv_data_.pos / 6.33f; }

 private:
  hal::SerialInterface *serial_;
  u16 id_;
  struct {
    bool correct;

    unsigned short id;
    unsigned short mode;

    float tau;
    float vel;
    float pos;
    float temp;
    unsigned short MError;
    unsigned short force;
  } recv_data_{};
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_GO8010_MOTOR_HPP
