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
 * @file  librm/device/sensor/hipnuc_imu.cc
 * @brief HiPNUC CH0x0 IMU 串口驱动
 */

#include "hipnuc_imu.hpp"

#include <cmath>
#include <cstring>

namespace rm::device {

/**
 * @brief 构造函数
 * @param serial 串口对象
 */
HipnucImu::HipnucImu(hal::SerialInterface &serial) : serial_(&serial) {
  // 初始化解码器
  std::memset(&raw_, 0, sizeof(hipnuc_raw_t));

  // 绑定串口接收回调
  static hal::SerialRxCallbackFunction rx_callback =
      std::bind(&HipnucImu::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  this->serial_->AttachRxCallback(rx_callback);
}

/**
 * @brief 开始接收IMU数据
 */
void HipnucImu::Begin() { this->serial_->Begin(); }

/**
 * @brief 串口接收完成中断回调函数
 * @param data      接收到的数据
 * @param rx_len    接收到的数据长度
 */
void HipnucImu::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  // 逐字节喂入解码器
  for (u16 i = 0; i < rx_len; i++) {
    const int result = hipnuc_input(&raw_, data[i]);

    if (result > 0) {
      // 成功解码一帧数据
      ReportStatus(kOk);

      // 根据数据包类型处理
      if (raw_.hi91.tag == kHi91) {
        last_packet_type_ = kHi91;
        ProcessHi91Packet();
      } else if (raw_.hi81.tag == kHi81) {
        last_packet_type_ = kHi81;
        ProcessHi81Packet();
      } else if (raw_.hi83.tag == kHi83) {
        last_packet_type_ = kHi83;
        ProcessHi83Packet();
      }
    } else if (result < 0) {
      // 解码错误
      ReportStatus(kFault);
    }
  }
}

/**
 * @brief 处理0x91数据包(IMU数据，浮点)
 */
void HipnucImu::ProcessHi91Packet() {
  const hi91_t &hi91 = raw_.hi91;

  main_status_ = hi91.main_status;
  temperature_ = hi91.temp;
  system_time_ = hi91.system_time;

  // 加速度 (原始单位: g -> m/s²)
  acc_[0] = hi91.acc[0] * 9.81f;
  acc_[1] = hi91.acc[1] * 9.81f;
  acc_[2] = hi91.acc[2] * 9.81f;

  // 角速度 (原始单位: deg/s -> rad/s)
  gyro_[0] = hi91.gyr[0] * 0.017453292519943295f;
  gyro_[1] = hi91.gyr[1] * 0.017453292519943295f;
  gyro_[2] = hi91.gyr[2] * 0.017453292519943295f;

  // 磁力计 (uT)
  mag_[0] = hi91.mag[0];
  mag_[1] = hi91.mag[1];
  mag_[2] = hi91.mag[2];

  // 姿态角 (原始单位: deg -> rad)
  roll_ = hi91.roll * 0.017453292519943295f;
  pitch_ = hi91.pitch * 0.017453292519943295f;
  yaw_ = hi91.yaw * 0.017453292519943295f;

  // 四元数
  quat_[0] = hi91.quat[0];
  quat_[1] = hi91.quat[1];
  quat_[2] = hi91.quat[2];
  quat_[3] = hi91.quat[3];

  // 气压
  air_pressure_ = hi91.air_pressure;
}

/**
 * @brief 处理0x81数据包(INS数据)
 */
void HipnucImu::ProcessHi81Packet() {
  const hi81_t &hi81 = raw_.hi81;

  main_status_ = hi81.main_status;
  temperature_ = hi81.temperature;

  // 加速度 (原始数据是int16，需要根据量程转换)
  // 假设量程为±16g，转换系数为16.0/32768
  const f32 acc_scale = 16.0f * 9.81f / 32768.0f;
  acc_[0] = hi81.acc_b[0] * acc_scale;
  acc_[1] = hi81.acc_b[1] * acc_scale;
  acc_[2] = hi81.acc_b[2] * acc_scale;

  // 角速度 (原始数据是int16，需要根据量程转换)
  // 假设量程为±2000deg/s，转换系数为2000.0/32768
  const f32 gyro_scale = 2000.0f * 0.017453292519943295f / 32768.0f;
  gyro_[0] = hi81.gyr_b[0] * gyro_scale;
  gyro_[1] = hi81.gyr_b[1] * gyro_scale;
  gyro_[2] = hi81.gyr_b[2] * gyro_scale;

  // 磁力计 (原始数据是int16，需要根据量程转换)
  const f32 mag_scale = 1.0f / 10.0f;  // 单位转换
  mag_[0] = hi81.mag_b[0] * mag_scale;
  mag_[1] = hi81.mag_b[1] * mag_scale;
  mag_[2] = hi81.mag_b[2] * mag_scale;

  // 姿态角 (原始单位: 0.01deg -> rad)
  roll_ = hi81.roll * 0.01f * 0.017453292519943295f;
  pitch_ = hi81.pitch * 0.01f * 0.017453292519943295f;
  yaw_ = hi81.yaw * 0.01f * 0.017453292519943295f;

  // 四元数 (原始单位: 缩放因子0.0001)
  quat_[0] = hi81.quat[0] * 0.0001f;
  quat_[1] = hi81.quat[1] * 0.0001f;
  quat_[2] = hi81.quat[2] * 0.0001f;
  quat_[3] = hi81.quat[3] * 0.0001f;

  // 气压
  air_pressure_ = hi81.air_pressure;
}

/**
 * @brief 处理0x83数据包(自定义数据包)
 */
void HipnucImu::ProcessHi83Packet() {
  const hi83_t &hi83 = raw_.hi83;

  main_status_ = hi83.main_status;
  system_time_ = hi83.system_time;

  // 根据bitmap判断哪些数据可用
  u32 bm = hi83.data_bitmap;

  // 加速度 (m/s²)
  if (bm & HI83_BMAP_ACC_B) {
    acc_[0] = hi83.acc_b[0];
    acc_[1] = hi83.acc_b[1];
    acc_[2] = hi83.acc_b[2];
  }

  // 角速度 (原始单位: deg/s -> rad/s)
  if (bm & HI83_BMAP_GYR_B) {
    gyro_[0] = hi83.gyr_b[0] * 0.017453292519943295f;
    gyro_[1] = hi83.gyr_b[1] * 0.017453292519943295f;
    gyro_[2] = hi83.gyr_b[2] * 0.017453292519943295f;
  }

  // 磁力计 (uT)
  if (bm & HI83_BMAP_MAG_B) {
    mag_[0] = hi83.mag_b[0];
    mag_[1] = hi83.mag_b[1];
    mag_[2] = hi83.mag_b[2];
  }

  // 姿态角 (原始单位: deg -> rad)
  if (bm & HI83_BMAP_RPY) {
    roll_ = hi83.rpy[0] * 0.017453292519943295f;
    pitch_ = hi83.rpy[1] * 0.017453292519943295f;
    yaw_ = hi83.rpy[2] * 0.017453292519943295f;
  }

  // 四元数
  if (bm & HI83_BMAP_QUAT) {
    quat_[0] = hi83.quat[0];
    quat_[1] = hi83.quat[1];
    quat_[2] = hi83.quat[2];
    quat_[3] = hi83.quat[3];
  }

  // 气压
  if (bm & HI83_BMAP_AIR_PRESSURE) {
    air_pressure_ = hi83.air_pressure;
  }

  // 温度
  if (bm & HI83_BMAP_TEMPERATURE) {
    temperature_ = static_cast<i8>(hi83.temperature);
  }
}

}  // namespace rm::device
