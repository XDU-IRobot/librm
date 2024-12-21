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
 * @file  librm/modules/algorithm/mahony.cc
 * @brief Mahony姿态解算算法
 */

#include "mahony.h"

#include "librm/modules/algorithm/utils.hpp"

#if defined(LIBRM_PLATFORM_STM32)
#include "librm/hal/stm32/hal.h"

#if !defined(__FPU_PRESENT)
#define __FPU_PRESENT 1  // fallback to use sqrtf if we can't determine if FPU is present
#endif

#endif

namespace rm::modules::algorithm {

inline f32 InvSqrt(f32 x) {
#if defined(LIBRM_PLATFORM_STM32) && __FPU_PRESENT == 0
  // Fast inverse square-root
  // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
  // for stm32 models that do not have a fpu, we use this algorithm
  f32 halfx = 0.5f * x;
  f32 y = x;
  long i = *reinterpret_cast<long *>(&y);
  i = 0x5f3759df - (i >> 1);
  y = *reinterpret_cast<f32 *>(&i);
  y = y * (1.5f - (halfx * y * y));
  return y;
#else
  return 1.f / sqrtf(x);
#endif
}

/**
 * @param sample_freq 采样频率(Hz)，默认值1000Hz
 * @param kp          比例增益，默认值2.0f
 * @param ki          积分增益，默认值0.0f
 */
MahonyAhrs::MahonyAhrs(f32 sample_freq, f32 kp, f32 ki)
    : two_kp_(2.f * kp),
      two_ki_(2.f * ki),
      sample_freq_(sample_freq),
      q0_(1.0f),
      q1_(0.0f),
      q2_(0.0f),
      q3_(0.0f),
      quaternion_{1.0f, 0.0f, 0.0f, 0.0f},
      euler_ypr_{0.0f, 0.0f, 0.0f} {}

/**
 * @brief       更新数据（无磁力计）
 * @param data  IMU数据
 */
void MahonyAhrs::Update(const rm::modules::algorithm::ImuData6Dof &data) {
  f32 recipNorm;
  f32 halfvx, halfvy, halfvz;
  f32 halfex, halfey, halfez;
  f32 qa, qb, qc;
  f32 gx = data.gx, gy = data.gy, gz = data.gz, ax = data.ax, ay = data.ay, az = data.az;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = quaternion_.x * quaternion_.z - quaternion_.w * quaternion_.y;
    halfvy = quaternion_.w * quaternion_.x + quaternion_.y * quaternion_.z;
    halfvz = quaternion_.w * quaternion_.w - 0.5f + quaternion_.z * quaternion_.z;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (two_ki_ > 0.0f) {
      integral_fb_x_ = two_ki_ * halfex * (1.0f / sample_freq_) + integral_fb_x_;  // integral error scaled by Ki
      integral_fb_y_ = two_ki_ * halfey * (1.0f / sample_freq_) + integral_fb_y_;
      integral_fb_z_ = two_ki_ * halfez * (1.0f / sample_freq_) + integral_fb_z_;
      gx += integral_fb_x_;  // apply integral feedback
      gy += integral_fb_y_;
      gz += integral_fb_z_;
    } else {
      integral_fb_x_ = 0.0f;  // prevent integral windup
      integral_fb_y_ = 0.0f;
      integral_fb_z_ = 0.0f;
    }

    // Apply proportional feedback
    gx += two_kp_ * halfex;
    gy += two_kp_ * halfey;
    gz += two_kp_ * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sample_freq_));  // pre-multiply common factors
  gy *= (0.5f * (1.0f / sample_freq_));
  gz *= (0.5f * (1.0f / sample_freq_));
  qa = quaternion_.w;
  qb = quaternion_.x;
  qc = quaternion_.y;
  quaternion_.w += (-qb * gx - qc * gy - quaternion_.z * gz);
  quaternion_.x += (qa * gx + qc * gz - quaternion_.z * gy);
  quaternion_.y += (qa * gy - qb * gz + quaternion_.z * gx);
  quaternion_.z += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = InvSqrt(quaternion_.w * quaternion_.w + quaternion_.x * quaternion_.x + quaternion_.y * quaternion_.y +
                      quaternion_.z * quaternion_.z);
  quaternion_.w *= recipNorm;
  quaternion_.x *= recipNorm;
  quaternion_.y *= recipNorm;
  quaternion_.z *= recipNorm;

  // Convert quaternion to Euler angles
  f32 euler_ypr_temp[3], quaternion_temp[4] = {quaternion_.w, quaternion_.x, quaternion_.y, quaternion_.z};
  utils::QuatToEuler(quaternion_temp, euler_ypr_temp);
  euler_ypr_.yaw = euler_ypr_temp[0];
  euler_ypr_.pitch = euler_ypr_temp[1];
  euler_ypr_.roll = euler_ypr_temp[2];
}

/**
 * @brief       更新数据（有磁力计）
 * @param data  IMU+磁力计数据
 */
void MahonyAhrs::Update(const rm::modules::algorithm::ImuData9Dof &data) {
  f32 gx = data.gx, gy = data.gy, gz = data.gz, ax = data.ax, ay = data.ay, az = data.az, mx = data.mx, my = data.my,
      mz = data.mz;
  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Update(ImuData6Dof{gx, gy, gz, ax, ay, az});
    return;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recip_norm_ = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recip_norm_;
    ay *= recip_norm_;
    az *= recip_norm_;

    // Normalise magnetometer measurement
    recip_norm_ = InvSqrt(mx * mx + my * my + mz * mz);
    mx *= recip_norm_;
    my *= recip_norm_;
    mz *= recip_norm_;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0_ = q0_ * q0_;
    q0q1_ = q0_ * q1_;
    q0q2_ = q0_ * q2_;
    q0q3_ = q0_ * q3_;
    q1q1_ = q1_ * q1_;
    q1q2_ = q1_ * q2_;
    q1q3_ = q1_ * q3_;
    q2q2_ = q2_ * q2_;
    q2q3_ = q2_ * q3_;
    q3q3_ = q3_ * q3_;

    // Reference direction of Earth's magnetic field
    hx_ = 2.0f * (mx * (0.5f - q2q2_ - q3q3_) + my * (q1q2_ - q0q3_) + mz * (q1q3_ + q0q2_));
    hy_ = 2.0f * (mx * (q1q2_ + q0q3_) + my * (0.5f - q1q1_ - q3q3_) + mz * (q2q3_ - q0q1_));
    bx_ = sqrtf(hx_ * hx_ + hy_ * hy_);
    bz_ = 2.0f * (mx * (q1q3_ - q0q2_) + my * (q2q3_ + q0q1_) + mz * (0.5f - q1q1_ - q2q2_));

    // Estimated direction of gravity and magnetic field
    halfvx_ = q1q3_ - q0q2_;
    halfvy_ = q0q1_ + q2q3_;
    halfvz_ = q0q0_ - 0.5f + q3q3_;
    halfwx_ = bx_ * (0.5f - q2q2_ - q3q3_) + bz_ * (q1q3_ - q0q2_);
    halfwy_ = bx_ * (q1q2_ - q0q3_) + bz_ * (q0q1_ + q2q3_);
    halfwz_ = bx_ * (q0q2_ + q1q3_) + bz_ * (0.5f - q1q1_ - q2q2_);

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex_ = (ay * halfvz_ - az * halfvy_) + (my * halfwz_ - mz * halfwy_);
    halfey_ = (az * halfvx_ - ax * halfvz_) + (mz * halfwx_ - mx * halfwz_);
    halfez_ = (ax * halfvy_ - ay * halfvx_) + (mx * halfwy_ - my * halfwx_);

    // Compute and apply integral feedback if enabled
    if (two_ki_ > 0.0f) {
      integral_fb_x_ = two_ki_ * halfex_ * (1.0f / sample_freq_) + integral_fb_x_;  // integral error scaled by Ki
      integral_fb_y_ = two_ki_ * halfey_ * (1.0f / sample_freq_) + integral_fb_y_;
      integral_fb_z_ = two_ki_ * halfez_ * (1.0f / sample_freq_) + integral_fb_z_;
      gx += integral_fb_x_;  // apply integral feedback
      gy += integral_fb_y_;
      gz += integral_fb_z_;
    } else {
      integral_fb_x_ = 0.0f;  // prevent integral windup
      integral_fb_y_ = 0.0f;
      integral_fb_z_ = 0.0f;
    }

    // Apply proportional feedback
    gx += two_kp_ * halfex_;
    gy += two_kp_ * halfey_;
    gz += two_kp_ * halfez_;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sample_freq_));  // pre-multiply common factors
  gy *= (0.5f * (1.0f / sample_freq_));
  gz *= (0.5f * (1.0f / sample_freq_));
  qa_ = quaternion_.w;
  qb_ = quaternion_.x;
  qc_ = quaternion_.y;
  quaternion_.w += (-qb_ * gx - qc_ * gy - quaternion_.z * gz);
  quaternion_.x += (qa_ * gx + qc_ * gz - quaternion_.z * gy);
  quaternion_.y += (qa_ * gy - qb_ * gz + quaternion_.z * gx);
  quaternion_.z += (qa_ * gz + qb_ * gy - qc_ * gx);

  // Normalise quaternion
  recip_norm_ = InvSqrt(quaternion_.w * quaternion_.w + quaternion_.x * quaternion_.x + quaternion_.y * quaternion_.y +
                        quaternion_.z * quaternion_.z);
  quaternion_.w *= recip_norm_;
  quaternion_.x *= recip_norm_;
  quaternion_.y *= recip_norm_;
  quaternion_.z *= recip_norm_;

  // Convert quaternion to Euler angles
  f32 euler_ypr_temp[3], quaternion_temp[4] = {quaternion_.w, quaternion_.x, quaternion_.y, quaternion_.z};
  utils::QuatToEuler(quaternion_temp, euler_ypr_temp);
  euler_ypr_.yaw = euler_ypr_temp[0];
  euler_ypr_.pitch = euler_ypr_temp[1];
  euler_ypr_.roll = euler_ypr_temp[2];
}

/**
 * @return 姿态欧拉角
 */
[[nodiscard]] const EulerAngle &MahonyAhrs::euler_angle() const { return euler_ypr_; }

/**
 * @return 姿态四元数
 */
[[nodiscard]] const Quaternion &MahonyAhrs::quaternion() const { return quaternion_; }

}  // namespace rm::modules::algorithm