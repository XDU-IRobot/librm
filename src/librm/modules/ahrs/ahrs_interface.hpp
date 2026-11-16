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
 * @file  librm/modules/ahrs/ahrs_interface.hpp
 * @brief AHRS算法接口
 *
 * 坐标系为ROS REP-105的机体坐标系：x向前、y向左、z向上。世界坐标系为 ENU：x向东、y向北、z向上。
 * 姿态四元数表示从机体坐标系到世界坐标系的旋转。
 */

#ifndef LIBRM_MODULES_AHRS_AHRS_INTERFACE_HPP
#define LIBRM_MODULES_AHRS_AHRS_INTERFACE_HPP

#include <Eigen/Geometry>

namespace rm::modules {

/**
 * @brief 6轴IMU数据
 */
struct ImuData6Dof {
  Eigen::Vector3f gyro;   ///< 角速度，rad/s，机体系
  Eigen::Vector3f accel;  ///< 加速度，m/s^2，机体系
};

/**
 * @brief 9轴IMU数据
 */
struct ImuData9Dof {
  Eigen::Vector3f gyro;  ///< 角速度，rad/s，机体系
  Eigen::Vector3f accel; ///< 加速度，m/s^2，机体系
  Eigen::Vector3f mag;   ///< 磁场强度，任意单位，机体系
};

class AhrsInterface {
 public:
  virtual ~AhrsInterface() = default;

  /**
   * @brief 更新AHRS数据
   * @param data 6轴IMU数据
   */
  virtual void Update(const ImuData6Dof &data) = 0;

  /**
   * @brief 更新AHRS数据
   * @param data 9轴IMU+磁力计数据
   */
  virtual void Update(const ImuData9Dof &data) = 0;

  /**
   * @return 姿态欧拉角，单位为弧度，顺序为 roll(X) pitch(Y) yaw(Z)
   */
  [[nodiscard]] virtual const Eigen::Vector3f &euler_angle() const = 0;

  /**
   * @return 姿态四元数，机体到世界的旋转
   */
  [[nodiscard]] virtual const Eigen::Quaternionf &quaternion() const = 0;
};

}  // namespace rm::modules

#endif  // LIBRM_MODULES_AHRS_AHRS_INTERFACE_H