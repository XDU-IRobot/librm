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
 * @file    librm/device/sensor/hipnuc_imu.hpp
 * @brief   HiPNUC CH0x0 IMU 串口驱动
 */

#ifndef LIBRM_DEVICE_SENSOR_HIPNUC_IMU_HPP
#define LIBRM_DEVICE_SENSOR_HIPNUC_IMU_HPP

#include <hipnuc/hipnuc_dec.h>

#include "librm/core/typedefs.hpp"
#include "librm/hal/serial.hpp"
#include "librm/device/device.hpp"

namespace rm::device {

/**
 * @brief HiPNUC CH0x0 IMU模块
 *
 * 支持通过串口接收HiPNUC协议的IMU数据，包括：
 * - 0x91: IMU数据包，包含加速度、角速度、磁力计、姿态角、四元数等
 * - 0x81: INS数据包，包含GPS信息等
 * - 0x83: 自定义数据包
 */
class HipnucImu : public Device {
  /**
   * @brief HiPNUC CH0x0 IMU数据包类型
   */
  enum HipnucPacketType : u8 {
    kNone = 0x00,
    kHi91 = 0x91,  ///< IMU数据包(浮点)
    kHi81 = 0x81,  ///< INS数据包
    kHi83 = 0x83,  ///< 自定义数据包
  };

 public:
  HipnucImu() = delete;
  explicit HipnucImu(hal::SerialInterface &serial);

  void Begin();
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

  // 数据包类型
  [[nodiscard]] HipnucPacketType last_packet_type() const { return last_packet_type_; }

  // 通用状态
  [[nodiscard]] u16 main_status() const { return main_status_; }
  [[nodiscard]] i8 temperature() const { return temperature_; }
  [[nodiscard]] u32 system_time() const { return system_time_; }

  // 加速度 (m/s²)
  [[nodiscard]] f32 acc_x() const { return acc_[0]; }
  [[nodiscard]] f32 acc_y() const { return acc_[1]; }
  [[nodiscard]] f32 acc_z() const { return acc_[2]; }

  // 角速度 (rad/s)
  [[nodiscard]] f32 gyro_x() const { return gyro_[0]; }
  [[nodiscard]] f32 gyro_y() const { return gyro_[1]; }
  [[nodiscard]] f32 gyro_z() const { return gyro_[2]; }

  // 磁力计 (uT)
  [[nodiscard]] f32 mag_x() const { return mag_[0]; }
  [[nodiscard]] f32 mag_y() const { return mag_[1]; }
  [[nodiscard]] f32 mag_z() const { return mag_[2]; }

  // 姿态角 (rad)
  [[nodiscard]] f32 roll() const { return roll_; }
  [[nodiscard]] f32 pitch() const { return pitch_; }
  [[nodiscard]] f32 yaw() const { return yaw_; }

  // 四元数
  [[nodiscard]] f32 quat_w() const { return quat_[0]; }
  [[nodiscard]] f32 quat_x() const { return quat_[1]; }
  [[nodiscard]] f32 quat_y() const { return quat_[2]; }
  [[nodiscard]] f32 quat_z() const { return quat_[3]; }

  // 气压 (Pa)
  [[nodiscard]] f32 air_pressure() const { return air_pressure_; }

  // 获取原始解码器数据(用于高级功能)
  [[nodiscard]] const hipnuc_raw_t &raw_data() const { return raw_; }

 private:
  void ProcessHi91Packet();
  void ProcessHi81Packet();
  void ProcessHi83Packet();

  hal::SerialInterface *serial_;
  hipnuc_raw_t raw_{};  ///< HiPNUC SDK里定义的原始数据

  HipnucPacketType last_packet_type_{kNone};

  // 通用数据
  u16 main_status_{0};
  i8 temperature_{0};
  u32 system_time_{0};

  // IMU数据
  f32 acc_[3]{0.0f};        ///< 加速度 (m/s²)
  f32 gyro_[3]{0.0f};       ///< 角速度 (rad/s)
  f32 mag_[3]{0.0f};        ///< 磁力计 (uT)
  f32 roll_{0.0f};          ///< 横滚角 (rad)
  f32 pitch_{0.0f};         ///< 俯仰角 (rad)
  f32 yaw_{0.0f};           ///< 航向角 (rad)
  f32 quat_[4]{0.0f};       ///< 四元数 (w, x, y, z)
  f32 air_pressure_{0.0f};  ///< 气压 (Pa)
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_SENSOR_HIPNUC_IMU_HPP
