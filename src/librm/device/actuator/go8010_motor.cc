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
 * @file  librm/device/actuator/go8010_motor.cc
 * @brief GO8010电机类库
 */

#include "go8010_motor.hpp"

#include <etl/span.h>

#include "librm/core/typedefs.hpp"
#include "librm/hal/serial_interface.hpp"
#include "librm/modules/crc.hpp"

namespace rm::device {

/**
 * @param[in]      serial     串口对象
 * @param[in]      motor_id   电机ID
 * @returns        None
 */
Go8010Motor::Go8010Motor(hal::SerialInterface &serial, u8 motor_id) : serial_(&serial), id_{motor_id} {
  serial_->AttachRxCallback([this](etl::span<const u8> data) { RxCallback(data); });
}

/**
 * @brief  FOC模式下发送MIT控制命令
 * @param  position_rad           期望位置
 * @param  speed_rad_per_sec      期望速度
 * @param  torque_ff_nm           前馈力矩
 * @param  kp                     位置误差比例系数
 * @param  kd                     速度误差比例系数
 */
void Go8010Motor::SetMitCommand(f32 position_rad, f32 speed_rad_per_sec, f32 torque_ff_nm, f32 kp, f32 kd) {
  ControlData tx;
  tx.head[0] = 0xfe;
  tx.head[1] = 0xee;
  tx.mode.id = id_;
  tx.mode.status = 1;
  tx.comd.tau_des = torque_ff_nm * 256.f;
  tx.comd.vel_des = speed_rad_per_sec / 6.2831f * 256.f;
  tx.comd.pos_des = position_rad / 6.2831f * 32768.f;
  tx.comd.k_pos = kp * 1280;
  tx.comd.k_spd = kd * 1280;
  tx.crc16 = modules::CrcCcitt((u8 *)&tx, sizeof(tx) - 2, 0);

  serial_->Write(reinterpret_cast<u8 *>(&tx), sizeof(tx));
}

/**
 * @brief          串口接收完成回调函数，解包电机发回来的反馈数据
 * @note           不要手动调用
 * @param[in]      data    串口接收到的数据
 * @param[in]      rx_len  数据长度
 * @returns        None
 */
void Go8010Motor::RxCallback(etl::span<const u8> data) {
  if (data.size() != 16) {
    return;
  }
  const MotorData *rx_data = reinterpret_cast<const MotorData *>(data.data());

  if (rx_data->mode.id != id_) {
    return;
  }
  ReportStatus(kOk);
  recv_data_.id = rx_data->mode.id;
  recv_data_.mode = rx_data->mode.status;
  recv_data_.tau = rx_data->feedback.tau / 256.f;
  recv_data_.vel = rx_data->feedback.vel * 2.f * 3.1415926f / 256.f;
  recv_data_.pos = rx_data->feedback.pos * 2.f * 3.1415926f / 32768.f;
  recv_data_.temp = rx_data->feedback.temp;
  recv_data_.MError = rx_data->feedback.MError;
  recv_data_.force = rx_data->feedback.force;
  recv_data_.correct = true;
}

}  // namespace rm::device