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
 * @file  librm/device/actuator/unitree_motor.hpp
 * @brief 宇树电机类库
 * @todo  补充注释
 */

#ifndef LIBRM_DEVICE_ACTUATOR_UNITREE_MOTOR_HPP
#define LIBRM_DEVICE_ACTUATOR_UNITREE_MOTOR_HPP

#include "librm/hal/serial.h"
#include "librm/core/typedefs.h"

#include <string>

namespace rm::device {

class UnitreeMotor {
 public:
  union ComData32 {
    i32 L;
    u8 ku8[4];
    u16 ku16[2];
    u32 ku32;
    f32 F;
  };

  struct ControlParam {
    f32 tau;
    f32 vel;
    f32 pos;
    f32 kp;
    f32 kd;
  };

  struct FeedbackParam {
    u8 mode;
    i8 temp;
    u8 m_error;

    f32 tau;
    f32 vel;

    i16 acc;
    f32 pos;

    f32 gyro[3];
    f32 accel[3];
  };

  struct ComHead {
    u8 head[2]{0xFE, 0xEE};
    u8 motor_id;
    u8 reserved;
  };

  struct ComDataSend {
    u8 mode;
    u8 modify_bit;
    u8 read_bit;
    u8 reserved;

    ComData32 Modify;

    i16 tau;
    i16 vel;
    i32 pos;
    i16 kp;
    i16 kd;

    u8 LowHzMotorCmdIndex;
    u8 LowHzMotorCmdByte;

    ComData32 Res[1];
  } __attribute__((packed));

  struct ComDataReceive {
    u8 mode;
    u8 read_bit;
    i8 temp;
    u8 m_error;

    ComData32 read;
    i16 tau;

    i16 vel;
    f32 low_vel;

    i16 vel_ref;
    f32 low_vel_ref;

    i16 acc;
    i16 out_acc;

    i32 pos;
    i32 out_pos;

    i16 gyro[3];
    i16 accel[3];

    i16 f_gyro[3];
    i16 f_acc[3];
    i16 f_mag[3];
    u8 f_temp;

    i16 force16;
    i8 force8;

    u8 f_error;

    i8 res[1];
  } __attribute__((packed));

  struct SendData {
    ComHead head;
    ComDataSend data;
    ComData32 crc;
  } __attribute__((packed));

  struct ReceiveData {
    ComHead head;
    ComDataReceive data;
    ComData32 crc;
  } __attribute__((packed));

 public:
  UnitreeMotor(hal::SerialInterface &serial, u8 motor_id = 0x0);
  ~UnitreeMotor() = default;

  void SetTau(f32 tau);

  void SendCommend();

  void RxCallback(const std::vector<u8> &data, u16 rx_len);

  [[nodiscard]] f32 tau() { return this->fb_param_.tau / 9.1f; }
  [[nodiscard]] f32 vel() { return this->fb_param_.vel / 9.1f; }
  [[nodiscard]] i16 acc() { return this->fb_param_.acc; }
  [[nodiscard]] f32 pos() { return this->fb_param_.pos / 9.1f; }

 private:
  void SetParam(const ControlParam &ctrl_param);

 private:
  hal::SerialInterface *serial_;

  SendData send_data_;
  ReceiveData recv_data_;
  ControlParam ctrl_param_;
  FeedbackParam fb_param_;
  u8 tx_buffer_[34]{0};
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_UNITREE_MOTOR_HPP