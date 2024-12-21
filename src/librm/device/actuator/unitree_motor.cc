/*
  Copyright (c) 2024 XDU-IRobot

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
 * @file  librm/device/actuator/unitree_motor.cc
 * @brief 宇树电机类库
 */

#include "unitree_motor.hpp"

#include "librm/hal/serial_interface.h"
#include "librm/modules/algorithm/crc.h"

namespace rm::device {

/**
 * @brief 串口接收回调函数键值对
 * @note  用于存储串口接收回调函数
 * @note  key: 串口对象指针
 * @note  value: 电机ID和回调函数的键值对
 * @note  key: 电机ID
 * @note  value: 回调函数
 * 解释：相当于两个键对应一个值，第一个键是串口对象指针，第二个键是电机ID，值是回调函数，用于实现多个电机的回调函数
 */
std::unordered_map<rm::hal::SerialInterface *,
                   std::unordered_map<u8, std::function<void(const std::vector<u8> &, u16)>>>
    rx_callback_map;

/**
 * @param[in]      serial     串口对象
 * @param[in]      motor_id   电机ID
 * @returns        None
 */

UnitreeMotor::UnitreeMotor(hal::SerialInterface &serial, u8 motor_id) : serial_(&serial) {
  rx_callback_map[&serial][motor_id] =
      std::bind(&UnitreeMotor::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  serial_->AttachRxCallback(rx_callback_map[&serial][motor_id]);

  send_data_.head.motor_id = motor_id;
  send_data_.head.reserved = 0x0;
}

/**
 * @brief          设置电机力矩
 * @param[in]      tau    力矩
 * @returns        None
 */
void UnitreeMotor::SetTau(f32 tau) {
  ctrl_param_.tau = tau;
  ctrl_param_.vel = 0;
  ctrl_param_.pos = 0;
  ctrl_param_.kp = 0;
  ctrl_param_.kd = 0;

  SetParam(ctrl_param_);
}

/**
 * @brief          发送电机控制指令
 * @returns        None
 */
void UnitreeMotor::SendCommend() { serial_->Write(tx_buffer_, sizeof(tx_buffer_)); }

/**
 * @brief          串口接收完成回调函数，解包电机发回来的反馈数据
 * @note           不要手动调用
 * @param[in]      data    串口接收到的数据
 * @param[in]      rx_len  数据长度
 * @returns        None
 */
void UnitreeMotor::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  if (rx_len != 78) {
    return;
  }
  std::copy(data.begin(), data.end(), reinterpret_cast<u8 *>(&recv_data_));

  if (recv_data_.head.head[0] != 0xFE || recv_data_.head.head[1] != 0xEE) {
    return;
  }

  if (recv_data_.head.motor_id == send_data_.head.motor_id) {
    fb_param_.mode = recv_data_.data.mode;
    fb_param_.temp = recv_data_.data.temp;
    fb_param_.m_error = recv_data_.data.m_error;

    fb_param_.tau = (f32)((f32)recv_data_.data.tau / 256.0f);
    fb_param_.vel = (f32)((f32)recv_data_.data.vel / 128.0f);

    fb_param_.acc = (i16)recv_data_.data.acc;
    fb_param_.pos = (f32)((f32)recv_data_.data.pos * 6.2832f / 16384.0f);

    fb_param_.gyro[0] = (f32)(((f32)recv_data_.data.gyro[0]) * 0.00107993176f);
    fb_param_.gyro[1] = (f32)(((f32)recv_data_.data.gyro[1]) * 0.00107993176f);
    fb_param_.gyro[2] = (f32)(((f32)recv_data_.data.gyro[2]) * 0.00107993176f);

    fb_param_.accel[0] = (f32)(((f32)recv_data_.data.accel[0]) * 0.0023911132f);
    fb_param_.accel[1] = (f32)(((f32)recv_data_.data.accel[1]) * 0.0023911132f);
    fb_param_.accel[2] = (f32)(((f32)recv_data_.data.accel[2]) * 0.0023911132f);
  }
}

/**
 * @brief          设置电机控制参数
 * @param[in]      ctrl_param    控制参数
 * @returns        None
 */
void UnitreeMotor::SetParam(const ControlParam &ctrl_param) {
  send_data_.data.mode = 10;
  send_data_.data.modify_bit = 0xFF;
  send_data_.data.read_bit = 0x0;
  send_data_.data.reserved = 0x0;
  send_data_.data.Modify.L = 0;
  send_data_.data.tau = ctrl_param.tau * 256;
  send_data_.data.vel = ctrl_param.vel * 128;
  send_data_.data.pos = (f32)((ctrl_param.pos / 6.2832) * 16384.f);
  send_data_.data.kp = ctrl_param.kp * 2048;
  send_data_.data.kd = ctrl_param.kd * 1024;

  send_data_.data.LowHzMotorCmdIndex = 0;
  send_data_.data.LowHzMotorCmdByte = 0;

  send_data_.crc.ku32 = modules::algorithm::Crc32((u32 *)(&send_data_), 7, modules::algorithm::CRC32_INIT);

  std::copy(reinterpret_cast<u8 *>(&send_data_), reinterpret_cast<u8 *>(&send_data_) + sizeof(send_data_), tx_buffer_);
}

}  // namespace rm::device