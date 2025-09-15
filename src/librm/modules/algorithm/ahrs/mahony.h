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
 * @file  modules/algorithm/mahony.h
 * @brief Mahony姿态解算算法
 */

#ifndef LIBRM_MODULES_ALGORITHM_MAHONY_H
#define LIBRM_MODULES_ALGORITHM_MAHONY_H

#include <array>

#include "librm/core/typedefs.h"
#include "librm/modules/algorithm/ahrs/ahrs_interface.h"

namespace rm::modules::algorithm {

class MahonyAhrs : public AhrsInterface {
 public:
  explicit MahonyAhrs(f32 sample_freq = 1000.0f, f32 kp = 1.0f, f32 ki = 0.0f);
  ~MahonyAhrs() override = default;

  void Update(const ImuData9Dof &data) override;
  void Update(const ImuData6Dof &data) override;
  [[nodiscard]] const EulerAngle &euler_angle() const override;
  [[nodiscard]] const Quaternion &quaternion() const override;

 private:
  f32 two_kp_, two_ki_;
  f32 q0_, q1_, q2_, q3_;
  f32 recip_norm_;
  f32 q0q0_, q0q1_, q0q2_, q0q3_, q1q1_, q1q2_, q1q3_, q2q2_, q2q3_, q3q3_;
  f32 hx_, hy_, bx_, bz_;
  f32 halfvx_, halfvy_, halfvz_, halfwx_, halfwy_, halfwz_;
  f32 halfex_, halfey_, halfez_;
  f32 qa_, qb_, qc_;
  Quaternion quaternion_;
  EulerAngle euler_ypr_;
  f32 sample_freq_;
  volatile f32 integral_fb_x_ = 0.0f, integral_fb_y_ = 0.0f,
               integral_fb_z_ = 0.0f;  // integral error terms scaled by Ki
};

}  // namespace rm::modules::algorithm

#endif  // LIBRM_MODULES_ALGORITHM_MAHONY_H