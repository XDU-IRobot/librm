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
 * @file  librm/device/actuator/go8010_motor.cc
 * @brief GO8010电机类库
 */

#include "go8010_motor.hpp"
#include <cstdint>

#include "librm/core/typedefs.hpp"
#include "librm/hal/serial_interface.hpp"
#include "librm/modules/crc.hpp"

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
                   std::unordered_map<rm::u8, std::function<void(const std::vector<rm::u8> &, rm::u16)>>>
    rx_callback_map_go8010;

namespace rm::device {

/**
 * @param[in]      serial     串口对象
 * @param[in]      motor_id   电机ID
 * @returns        None
 */
Go8010Motor::Go8010Motor(hal::SerialInterface &serial, u8 motor_id) : serial_(&serial) {
  rx_callback_map_go8010[&serial][motor_id] =
      std::bind(&Go8010Motor::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  serial_->AttachRxCallback(rx_callback_map_go8010[&serial][motor_id]);

  send_data_.id = motor_id;
}

/**
 * @brief          设置电机力矩
 * @param[in]      tau    力矩
 * @returns        None
 */
void Go8010Motor::SetTau(f32 tau) {
  send_data_.mode = 1;

  send_data_.tau = tau;
  send_data_.vel = 0;
  send_data_.pos = 0;
  send_data_.kp = 0;
  send_data_.kd = 0;

  SetParam(send_data_);
}

/**
 * @brief          发送电机控制指令
 * @returns        None
 */
void Go8010Motor::SendCommend() { serial_->Write(tx_buffer_, sizeof(tx_buffer_)); }

/**
 * @brief          串口接收完成回调函数，解包电机发回来的反馈数据
 * @note           不要手动调用
 * @param[in]      data    串口接收到的数据
 * @param[in]      rx_len  数据长度
 * @returns        None
 */
void Go8010Motor::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  if (rx_len != 16) {
    return;
  }
  std::copy(data.begin(), data.end(), reinterpret_cast<u8 *>(&recv_data_.motor_recv_data));

  // if (recv_data_.motor_recv_data.head[0] != 0xFE || recv_data_.motor_recv_data.head[1] != 0xEE) {
  //   return;
  // }

  if (recv_data_.motor_recv_data.mode.id == send_data_.motor_send_data.mode.id) {
    recv_data_.id = recv_data_.motor_recv_data.mode.id;
    recv_data_.mode = recv_data_.motor_recv_data.mode.status;
    recv_data_.tau = recv_data_.motor_recv_data.fbk.tau / 256.f;
    recv_data_.vel = recv_data_.motor_recv_data.fbk.vel * 2.f * 3.1415926f / 256.f;
    recv_data_.pos = recv_data_.motor_recv_data.fbk.pos * 2.f * 3.1415926f / 32768.f;
    recv_data_.temp = recv_data_.motor_recv_data.fbk.temp;
    recv_data_.MError = recv_data_.motor_recv_data.fbk.MError;
    recv_data_.force = recv_data_.motor_recv_data.fbk.force;
    recv_data_.correct = true;
  }
}

/**
 * @brief          设置电机控制参数
 * @param[in]      send_data    电机控制参数
 * @returns        None
 */
void Go8010Motor::SetParam(const SendData &send_data) {
  send_data_.motor_send_data.head[0] = 0xFE;
  send_data_.motor_send_data.head[1] = 0xEE;
  send_data_.motor_send_data.mode.id = send_data.id;
  send_data_.motor_send_data.mode.status = send_data.mode;
  send_data_.motor_send_data.comd.tau_des = send_data.tau * 256.f;
  send_data_.motor_send_data.comd.vel_des = send_data.vel / 6.2831f * 256.f;
  send_data_.motor_send_data.comd.pos_des = send_data.pos / 6.2831f * 32768.f;
  send_data_.motor_send_data.comd.k_pos = send_data.kp * 1280;
  send_data_.motor_send_data.comd.k_spd = send_data.kd * 1280;

  send_data_.motor_send_data.CRC16 = rm::modules::CrcCcitt((rm::u8 *)&send_data_.motor_send_data, 15, 0x0);

  std::copy(reinterpret_cast<u8 *>(&send_data_.motor_send_data),
            reinterpret_cast<u8 *>(&send_data_.motor_send_data) + sizeof(send_data_.motor_send_data), tx_buffer_);
}

}  // namespace rm::device