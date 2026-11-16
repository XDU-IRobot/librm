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
 * @file  librm/modules/ahrs/quaternion_ekf.hpp
 * @brief 四元数EKF姿态解算
 */

#ifndef LIBRM_MODULES_AHRS_QUATERNION_EKF_HPP
#define LIBRM_MODULES_AHRS_QUATERNION_EKF_HPP

#include <Eigen/Core>

#include "librm/core/typedefs.hpp"
#include "librm/modules/ahrs/ahrs_interface.hpp"

namespace rm::modules {

/**
 * @brief 基于扩展卡尔曼滤波的四元数姿态解算
 *
 * @note  credit: 算法来自 https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example
 *
 * 状态量 x = [q0, q1, q2, q3, bgx, bgy]，量测量为归一化加速度。
 * 绕z轴的陀螺零偏不可观测，恒当作0处理。
 */
class QuaternionEkfAhrs : public AhrsInterface {
 public:
  /**
   * @param sample_freq    采样频率(Hz)
   * @param initial        初始姿态四元数
   * @param attitude_noise 四元数的过程噪声
   * @param bias_noise     陀螺零偏的过程噪声
   * @param measurement_noise 加速度的量测噪声方差，单位为g的平方。
   *        取值决定了新息门限的灵敏度：过大则无法区分平动加速度和噪声，
   *        过小则静止时也会频繁拒绝量测。默认 1e-3 对应约 0.03g 的标准差。
   * @param fading         渐消因子，用于防止零偏协方差过度收敛
   * @param accel_lpf      加速度一阶低通的时间常数
   */
  explicit QuaternionEkfAhrs(f32 sample_freq = 1000.0f, const Eigen::Quaternionf &initial = Eigen::Quaternionf::Identity(),
                             f32 attitude_noise = 1e-5f, f32 bias_noise = 1e-6f, f32 measurement_noise = 1e-3f,
                             f32 fading = 1.0f, f32 accel_lpf = 0.001f);
  ~QuaternionEkfAhrs() override = default;

  QuaternionEkfAhrs(const QuaternionEkfAhrs &) = delete;
  QuaternionEkfAhrs &operator=(const QuaternionEkfAhrs &) = delete;

  void Update(const ImuData6Dof &data) override;
  void Update(const ImuData9Dof &data) override;
  [[nodiscard]] const Eigen::Vector3f &euler_angle() const override;
  [[nodiscard]] const Eigen::Quaternionf &quaternion() const override;

 private:
  static constexpr int kStateDim = 6;
  static constexpr int kMeasDim = 3;

  using StateVector = Eigen::Matrix<f32, kStateDim, 1>;
  using StateMatrix = Eigen::Matrix<f32, kStateDim, kStateDim>;
  using MeasVector = Eigen::Matrix<f32, kMeasDim, 1>;
  using MeasMatrix = Eigen::Matrix<f32, kMeasDim, kMeasDim>;
  using MeasJacobian = Eigen::Matrix<f32, kMeasDim, kStateDim>;
  using GainMatrix = Eigen::Matrix<f32, kStateDim, kMeasDim>;

  void Prepare(const ImuData6Dof &data);
  void Predict();
  void Correct();

  f32 dt_;
  f32 fading_;
  f32 accel_lpf_;
  StateMatrix process_cov_;
  MeasMatrix measure_cov_;

  bool accel_initialized_ = false;
  u8 converged_ = 0;
  u16 error_count_ = 0;
  f32 accel_weight_ = 1.0f;  // 加速度计方向的可信度，由合加速度相对重力的偏离量决定

  MeasVector gyro_;
  MeasVector accel_;
  MeasVector measured_;
  MeasVector predicted_;
  StateVector state_;
  StateVector state_prior_;
  StateMatrix covariance_;
  StateMatrix covariance_prior_;

  Eigen::Quaternionf quaternion_ = Eigen::Quaternionf::Identity();
  Eigen::Vector3f euler_ypr_ = Eigen::Vector3f::Zero();
};

}  // namespace rm::modules

#endif  // LIBRM_MODULES_AHRS_QUATERNION_EKF_HPP
