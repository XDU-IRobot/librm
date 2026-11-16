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
 * @file  librm/modules/ahrs/quaternion_ekf.cc
 * @brief 四元数EKF姿态解算
 */

#include "quaternion_ekf.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>

#include "librm/modules/utils.hpp"

namespace rm::modules {

namespace {

constexpr f32 kChiSquareThreshold = 16.0f;  // 3自由度的NIS门限，约3sigma
constexpr f32 kHalfPi = 1.5707963f;
constexpr f32 kMaxBiasVariance = 10000.0f;  // 零偏协方差上限，防止发散
constexpr f32 kMaxBiasRate = 1e-2f;         // 每次更新的零偏修正上限
constexpr u16 kErrorCountLimit = 50;        // 连续异常量测多少次后重新打开协方差更新
constexpr f32 kGravity = 9.8f;

// 合加速度偏离重力越大，说明平动加速度越强，方向越不可信
constexpr f32 kAccelTrustFull = 0.03f;  // 偏离 3%（约0.3m/s^2）以内完全信任
constexpr f32 kAccelTrustNone = 0.20f;  // 偏离 20%（约2m/s^2）以上基本丢弃
constexpr f32 kAccelWeightMin = 0.02f;  // 权重下限，避免噪声尺度发散

}  // namespace

QuaternionEkfAhrs::QuaternionEkfAhrs(f32 sample_freq, const Eigen::Quaternionf &initial, f32 attitude_noise,
                                     f32 bias_noise, f32 measurement_noise, f32 fading, f32 accel_lpf)
    : dt_(1.0f / sample_freq), fading_(fading > 1.0f ? 1.0f : fading), accel_lpf_(accel_lpf), quaternion_(initial) {
  process_cov_.setZero();
  process_cov_.diagonal() << attitude_noise * dt_, attitude_noise * dt_, attitude_noise * dt_, attitude_noise * dt_,
      bias_noise * dt_, bias_noise * dt_;

  measure_cov_.setIdentity();
  measure_cov_ *= measurement_noise;

  state_.setZero();
  state_(0) = initial.w();
  state_(1) = initial.x();
  state_(2) = initial.y();
  state_(3) = initial.z();

  // 初始姿态完全不确定，零偏初值相对可信
  covariance_.setConstant(0.1f);
  covariance_.diagonal() << 1e5f, 1e5f, 1e5f, 1e5f, 100.0f, 100.0f;
}

void QuaternionEkfAhrs::Update(const ImuData6Dof &data) {
  Prepare(data);
  Predict();
  Correct();

  // 融合后的状态不再严格满足单位化，对外输出前统一归一化
  state_.head<4>().normalize();

  quaternion_ = Eigen::Quaternionf(state_(0), state_(1), state_(2), state_(3));
  euler_ypr_ = QuatToEuler(quaternion_);
}

void QuaternionEkfAhrs::Update(const ImuData9Dof &data) {
  // 磁力计不参与解算
  Update(ImuData6Dof{data.gyro, data.accel});
}

const Eigen::Vector3f &QuaternionEkfAhrs::euler_angle() const { return euler_ypr_; }

const Eigen::Quaternionf &QuaternionEkfAhrs::quaternion() const { return quaternion_; }

void QuaternionEkfAhrs::Prepare(const ImuData6Dof &data) {
  gyro_ << data.gyro.x() - state_(4), data.gyro.y() - state_(5), data.gyro.z();

  const MeasVector accel = data.accel;
  if (!accel_initialized_) {
    accel_ = accel;
    accel_initialized_ = true;
  }
  const f32 new_weight = dt_ / (dt_ + accel_lpf_);
  accel_ = accel_ * (1.0f - new_weight) + accel * new_weight;

  const f32 accel_norm = accel_.norm();
  measured_ = accel_ / accel_norm;

  // 平动加速度会让合加速度偏离重力。偏离量越小，加速度计给出的方向越可信；
  // 偏离过大时把量测噪声等效放大，等于本次不采纳这个方向。
  const f32 deviation = std::fabs(accel_norm - kGravity) / kGravity;
  if (deviation <= kAccelTrustFull) {
    accel_weight_ = 1.0f;
  } else if (deviation >= kAccelTrustNone) {
    accel_weight_ = kAccelWeightMin;
  } else {
    const f32 t = (deviation - kAccelTrustFull) / (kAccelTrustNone - kAccelTrustFull);
    accel_weight_ = 1.0f - t * t * (3.0f - 2.0f * t);  // smoothstep，从 1 平滑过渡到 0
    if (accel_weight_ < kAccelWeightMin) {
      accel_weight_ = kAccelWeightMin;
    }
  }
}

void QuaternionEkfAhrs::Predict() {
  const f32 hgx = 0.5f * gyro_(0) * dt_;
  const f32 hgy = 0.5f * gyro_(1) * dt_;
  const f32 hgz = 0.5f * gyro_(2) * dt_;

  // 状态转移矩阵：单位阵叠加陀螺积分项，零偏为常值模型
  StateMatrix f = StateMatrix::Identity();
  f(0, 1) = -hgx;
  f(0, 2) = -hgy;
  f(0, 3) = -hgz;
  f(1, 0) = hgx;
  f(1, 2) = hgz;
  f(1, 3) = -hgy;
  f(2, 0) = hgy;
  f(2, 1) = -hgz;
  f(2, 3) = hgx;
  f(3, 0) = hgz;
  f(3, 1) = hgy;
  f(3, 2) = -hgx;

  state_prior_ = f * state_;

  const f32 q0 = state_prior_(0);
  const f32 q1 = state_prior_(1);
  const f32 q2 = state_prior_(2);
  const f32 q3 = state_prior_(3);
  state_prior_.head<4>().normalize();

  // 四元数对水平轴陀螺零偏的偏导
  f.block<4, 2>(0, 4) << 0.5f * q1 * dt_, 0.5f * q2 * dt_,  //
      -0.5f * q0 * dt_, 0.5f * q3 * dt_,                    //
      -0.5f * q3 * dt_, -0.5f * q0 * dt_,                   //
      0.5f * q2 * dt_, -0.5f * q1 * dt_;

  // 渐消记忆，防止零偏协方差过度收敛而对真实漂移失去跟踪能力
  for (int i = 4; i < kStateDim; ++i) {
    covariance_(i, i) = std::min(covariance_(i, i) / fading_, kMaxBiasVariance);
  }

  covariance_prior_ = f * covariance_ * f.transpose() + process_cov_;
}

void QuaternionEkfAhrs::Correct() {
  const f32 q0 = state_prior_(0);
  const f32 q1 = state_prior_(1);
  const f32 q2 = state_prior_(2);
  const f32 q3 = state_prior_(3);

  // 归一化加速度对四元数的雅可比
  MeasJacobian jacobian = MeasJacobian::Zero();
  jacobian(0, 0) = -2.0f * q2;
  jacobian(0, 1) = 2.0f * q3;
  jacobian(0, 2) = -2.0f * q0;
  jacobian(0, 3) = 2.0f * q1;
  jacobian(1, 0) = 2.0f * q1;
  jacobian(1, 1) = 2.0f * q0;
  jacobian(1, 2) = 2.0f * q3;
  jacobian(1, 3) = 2.0f * q2;
  jacobian(2, 0) = 2.0f * q0;
  jacobian(2, 1) = -2.0f * q1;
  jacobian(2, 2) = -2.0f * q2;
  jacobian(2, 3) = 2.0f * q3;

  predicted_ << 2.0f * (q1 * q3 - q0 * q2), 2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  const MeasVector innovation = measured_ - predicted_;

  // 可信度越低，等效噪声越大；平方是因为协方差与标准差是平方关系
  const f32 noise_scale = 1.0f / accel_weight_;
  const MeasMatrix measure_cov = measure_cov_ * (noise_scale * noise_scale);
  const MeasMatrix innovation_cov = jacobian * covariance_prior_ * jacobian.transpose() + measure_cov;
  const MeasMatrix innovation_cov_inv = innovation_cov.inverse();

  // 归一化新息平方（NIS），用于识别模值正常但方向异常的残差
  const f32 chi_square = innovation.dot(innovation_cov_inv * innovation);
  if (chi_square < 0.5f * kChiSquareThreshold) {
    converged_ = 1;
  }

  f32 adaptive_scale = 1.0f;
  if (chi_square > kChiSquareThreshold && converged_ != 0) {
    error_count_ = error_count_ + 1;
    if (error_count_ <= kErrorCountLimit) {
      // 量测不可信，本次只保留预测值
      state_ = state_prior_;
      covariance_ = covariance_prior_;
      return;
    }
    converged_ = 0;
  } else if (chi_square > 0.1f * kChiSquareThreshold && converged_ != 0) {
    adaptive_scale = (kChiSquareThreshold - chi_square) / (0.9f * kChiSquareThreshold);
    error_count_ = 0;
  } else {
    error_count_ = 0;
  }

  GainMatrix gain = covariance_prior_ * jacobian.transpose() * innovation_cov_inv;
  gain *= adaptive_scale;

  // 重力方向与某个水平轴夹角越接近90度，该轴零偏的可观测性越差，相应减小增益
  const f32 v0 = 2.0f * (q1 * q3 - q0 * q2);
  const f32 v1 = 2.0f * (q0 * q1 + q2 * q3);
  gain.row(4) *= std::acos(std::fabs(v0)) / kHalfPi;
  gain.row(5) *= std::acos(std::fabs(v1)) / kHalfPi;

  StateVector delta = gain * innovation;

  // 收敛后限制零偏的修正速度
  if (converged_ != 0) {
    const f32 limit = kMaxBiasRate * dt_;
    delta(4) = std::clamp(delta(4), -limit, limit);
    delta(5) = std::clamp(delta(5), -limit, limit);
  }

  state_ = state_prior_ + delta;
  covariance_ = covariance_prior_ - gain * jacobian * covariance_prior_;
}

}  // namespace rm::modules
