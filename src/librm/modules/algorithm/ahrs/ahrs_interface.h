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
 * @file  librm/modules/algorithm/ahrs/ahrs_interface.h
 * @brief AHRS算法接口
 */

#ifndef LIBRM_MODULES_ALGORITHM_AHRS_AHRS_INTERFACE_H
#define LIBRM_MODULES_ALGORITHM_AHRS_AHRS_INTERFACE_H

#include "librm/core/typedefs.h"

namespace rm::modules::algorithm {

struct ImuData6Dof {
  f32 gx;
  f32 gy;
  f32 gz;
  f32 ax;
  f32 ay;
  f32 az;
};

struct ImuData9Dof {
  f32 gx;
  f32 gy;
  f32 gz;
  f32 ax;
  f32 ay;
  f32 az;
  f32 mx;
  f32 my;
  f32 mz;
};

struct EulerAngle {
  f32 roll;
  f32 pitch;
  f32 yaw;
};

struct Quaternion {
  f32 w;
  f32 x;
  f32 y;
  f32 z;
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
   * @return 姿态欧拉角
   */
  [[nodiscard]] virtual const EulerAngle &euler_angle() const = 0;

  /**
   * @return 姿态四元数
   */
  [[nodiscard]] virtual const Quaternion &quaternion() const = 0;
};

}  // namespace rm::modules::algorithm

#endif  // LIBRM_MODULES_ALGORITHM_AHRS_AHRS_INTERFACE_H