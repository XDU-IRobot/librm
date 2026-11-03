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
 * @file    librm/device/sensor/hipnuc_imu_can.hpp
 * @brief   HiPNUC CH0x0 IMU CAN驱动 (CANOPEN协议)
 */

#ifndef LIBRM_DEVICE_SENSOR_HIPNUC_IMU_CAN_HPP
#define LIBRM_DEVICE_SENSOR_HIPNUC_IMU_CAN_HPP

#include <hipnuc/canopen_parser.h>

#include "librm/device/can_device.hpp"

namespace rm::device {

/**
 * @brief HiPNUC CH0x0 IMU模块，CANOPEN协议版本
 *
 * 支持通过CAN总线接收HiPNUC CANOPEN协议的IMU数据，包括：
 * - TPDO1 (0x180+NodeID): 加速度数据
 * - TPDO2 (0x280+NodeID): 角速度数据
 * - TPDO3 (0x380+NodeID): 欧拉角数据
 * - TPDO4 (0x480+NodeID): 四元数数据
 * - TPDO6 (0x680+NodeID): 气压数据
 * - TPDO7 (0x780+NodeID): 倾角数据
 *
 * @note 需要在构造时指定节点ID (node_id)，默认为1
 */
class HipnucImuCan final : public CanDevice {
 public:
  ~HipnucImuCan() override = default;
  HipnucImuCan(HipnucImuCan &&) noexcept = default;

  /**
   * @brief 构造函数
   * @param can       所属CAN总线
   * @param node_id   IMU的CANOPEN节点ID，默认为1
   */
  explicit HipnucImuCan(hal::CanInterface &can, u8 node_id = 1)
      : CanDevice(can,
                  static_cast<u16>(0x180 + node_id),   // TPDO1: 加速度
                  static_cast<u16>(0x280 + node_id),   // TPDO2: 角速度
                  static_cast<u16>(0x380 + node_id),   // TPDO3: 欧拉角
                  static_cast<u16>(0x480 + node_id),   // TPDO4: 四元数
                  static_cast<u16>(0x680 + node_id),   // TPDO6: 气压
                  static_cast<u16>(0x780 + node_id)),  // TPDO7: 倾角
        node_id_(node_id) {}

  /**
   * @brief CAN接收回调函数
   * @param msg 接收到的CAN报文
   */
  void RxCallback(const hal::CanFrame *msg) override {
    // 转换为HIPNUC SDK的帧格式
    hipnuc_can_frame_t frame;
    frame.can_id = msg->std_id;
    frame.can_dlc = static_cast<u8>(msg->dlc);
    for (u8 i = 0; (i < static_cast<u8>(msg->dlc)) && (i < 8); ++i) {
      frame.data[i] = msg->data[i];
    }
    frame.hw_ts_us = 0;  // 时间戳暂不使用

    // 使用HIPNUC SDK解析帧
    int msg_type = canopen_parse_frame(&frame, &sensor_data_);

    // 根据消息类型更新对应的数据
    if (msg_type != CAN_MSG_ERROR && msg_type != CAN_MSG_UNKNOWN) {
      ReportStatus(kOk);
      last_msg_type_ = msg_type;

      // 更新对应的数据字段
      switch (msg_type) {
        case CAN_MSG_ACCEL:
          acc_[0] = sensor_data_.acc_x;
          acc_[1] = sensor_data_.acc_y;
          acc_[2] = sensor_data_.acc_z;
          break;
        case CAN_MSG_GYRO:
          gyro_[0] = sensor_data_.gyr_x;
          gyro_[1] = sensor_data_.gyr_y;
          gyro_[2] = sensor_data_.gyr_z;
          break;
        case CAN_MSG_EULER:
          roll_ = sensor_data_.roll;
          pitch_ = sensor_data_.pitch;
          yaw_ = sensor_data_.imu_yaw;
          break;
        case CAN_MSG_QUAT:
          quat_[0] = sensor_data_.quat_w;
          quat_[1] = sensor_data_.quat_x;
          quat_[2] = sensor_data_.quat_y;
          quat_[3] = sensor_data_.quat_z;
          break;
        case CAN_MSG_PRESSURE:
          pressure_ = sensor_data_.pressure;
          break;
        case CAN_MSG_INCLI:
          incli_x_ = sensor_data_.incli_x;
          incli_y_ = sensor_data_.incli_y;
          break;
        default:
          break;
      }
    }
  }

  // 访问器方法

  /**
   * @brief 获取节点ID
   * @return 节点ID
   */
  [[nodiscard]] u8 node_id() const { return node_id_; }

  /**
   * @brief 获取最后一次接收的消息类型
   * @return 消息类型 (CAN_MSG_xxx)
   */
  [[nodiscard]] int last_msg_type() const { return last_msg_type_; }

  // 加速度 (m/s²)
  [[nodiscard]] f32 accel_x() const { return acc_[0]; }
  [[nodiscard]] f32 accel_y() const { return acc_[1]; }
  [[nodiscard]] f32 accel_z() const { return acc_[2]; }

  // 角速度 (rad/s)
  [[nodiscard]] f32 gyro_x() const { return gyro_[0]; }
  [[nodiscard]] f32 gyro_y() const { return gyro_[1]; }
  [[nodiscard]] f32 gyro_z() const { return gyro_[2]; }

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
  [[nodiscard]] f32 pressure() const { return pressure_; }

  // 倾角 (度)
  [[nodiscard]] f32 incli_x() const { return incli_x_; }
  [[nodiscard]] f32 incli_y() const { return incli_y_; }

  /**
   * @brief 获取原始传感器数据结构（用于高级功能）
   * @return 原始传感器数据
   */
  [[nodiscard]] const can_sensor_data_t &raw_data() const { return sensor_data_; }

 private:
  u8 node_id_{1};                       ///< CANOPEN节点ID
  int last_msg_type_{CAN_MSG_UNKNOWN};  ///< 最后一次接收的消息类型
  can_sensor_data_t sensor_data_{};     ///< HIPNUC SDK解析后的传感器数据

  // 缓存的IMU数据
  f32 acc_[3]{0.0f};    ///< 加速度 (m/s²)
  f32 gyro_[3]{0.0f};   ///< 角速度 (rad/s)
  f32 roll_{0.0f};      ///< 横滚角 (rad)
  f32 pitch_{0.0f};     ///< 俯仰角 (rad)
  f32 yaw_{0.0f};       ///< 航向角 (rad)
  f32 quat_[4]{0.0f};   ///< 四元数 (w, x, y, z)
  f32 pressure_{0.0f};  ///< 气压 (Pa)
  f32 incli_x_{0.0f};   ///< X轴倾角 (度)
  f32 incli_y_{0.0f};   ///< Y轴倾角 (度)
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_SENSOR_HIPNUC_IMU_CAN_HPP
