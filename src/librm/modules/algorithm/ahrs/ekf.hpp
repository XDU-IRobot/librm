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
 * @file  librm/modules/algorithm/ahrs/ekf.hpp
 * @brief EKF姿态解算算法
 */

#ifndef LIBRM_MODULES_ALGORITHM_EKF_HPP
#define LIBRM_MODULES_ALGORITHM_EKF_HPP

#include "librm/core/typedefs.hpp"
#include "librm/modules/algorithm/ahrs/ahrs_interface.hpp"

#include "kalman/LinearizedSystemModel.hpp"
#include "kalman/LinearizedMeasurementModel.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"

namespace rm::modules::algorithm {

class State : public Kalman::Vector<f32, 6> {
 public:
  KALMAN_VECTOR(State, f32, 6)

  //! Q0
  static constexpr size_t Q0 = 0;
  //! Q1
  static constexpr size_t Q1 = 1;
  //! Q2
  static constexpr size_t Q2 = 2;
  //! Q3
  static constexpr size_t Q3 = 3;
  //! WX_BIAS
  static constexpr size_t WX_BIAS = 4;
  //! WY_BIAS
  static constexpr size_t WY_BIAS = 5;

  f32 q0() const { return (*this)[Q0]; }
  f32 q1() const { return (*this)[Q1]; }
  f32 q2() const { return (*this)[Q2]; }
  f32 q3() const { return (*this)[Q3]; }
  f32 wx_bias() const { return (*this)[WX_BIAS]; }
  f32 wy_bias() const { return (*this)[WY_BIAS]; }

  f32 &q0() { return (*this)[Q0]; }
  f32 &q1() { return (*this)[Q1]; }
  f32 &q2() { return (*this)[Q2]; }
  f32 &q3() { return (*this)[Q3]; }
  f32 &wx_bias() { return (*this)[WX_BIAS]; }
  f32 &wy_bias() { return (*this)[WY_BIAS]; }
};

class Control : public Kalman::Vector<f32, 1> {
 public:
  KALMAN_VECTOR(Control, f32, 1)
};

template <template <class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State, Control, CovarianceBase> {
 public:
  //! 状态向量简写
  using S = rm::modules::algorithm::State;
  //! 控制向量简写
  using C = rm::modules::algorithm::Control;

  /**
   * @brief 构造函数
   *
   * 将采样频率传进来，供预测更新使用
   *
   * @param sample_freq 采样频率
   *
   */
  explicit SystemModel(const f32 sample_freq = 1000.0f) : sample_freq_(sample_freq) {}

  /**
   * @brief 获取陀螺仪数据
   *
   * 该函数用于获取陀螺仪数据
   *
   * @param gx 陀螺仪x轴数据
   * @param gy 陀螺仪y轴数据
   * @param gz 陀螺仪z轴数据
   */
  void GetGyro(f32 gx, f32 gy, f32 gz) {
    gx_ = gx;
    gy_ = gy;
    gz_ = gz;
  }

  /**
   * @brief 预测更新
   *
   * 该函数用于预测更新状态向量
   *
   * @param x 状态向量
   * @param u 控制向量
   * @returns 预测更新后的状态向量
   */
  S f(const S &x, const C &u) const {
    //! 预测更新状态向量
    S x_;

    // 中间量
    f32 dt = 1.0f / sample_freq_;
    f32 gx_real = gx_ - x.wx_bias();
    f32 gy_real = gy_ - x.wy_bias();
    f32 gz_real = gz_;  // 大部分时间机体z轴通天，无法用加速度计校准

    f32 q0_last = x.q0();
    f32 q1_last = x.q1();
    f32 q2_last = x.q2();
    f32 q3_last = x.q3();

    // 四元数预测更新
    // -  0 -gx -gy -gz  -     -  q0  -
    // |  gx  0  gz -gy  |  *  |  q1  |  *  0.5 * dt
    // |  gy -gz  0  gx  |     |  q2  |
    // -  gz  gy -gx  0  -     -  q3  -
    x_.q0() = q0_last + (-q1_last * gx_real - q2_last * gy_real - q3_last * gz_real) * dt * 0.5f;
    x_.q1() = q1_last + (q0_last * gx_real + q2_last * gz_real - q3_last * gy_real) * dt * 0.5f;
    x_.q2() = q2_last + (q0_last * gy_real - q1_last * gz_real + q3_last * gx_real) * dt * 0.5f;
    x_.q3() = q3_last + (q0_last * gz_real + q1_last * gy_real - q2_last * gx_real) * dt * 0.5f;

    // 陀螺仪x，y轴误差预测更新
    x_.wx_bias() = x.wx_bias();
    x_.wy_bias() = x.wy_bias();

    return x_;
  }

 protected:
  /**
   * @brief 更新雅可比矩阵
   *
   * 该函数用于更新雅可比矩阵
   *
   * @param x 状态向量
   * @param u 控制向量
   */
  void updateJacobians(const S &x, const C &u) {
    // 中间量
    f32 dt = 1.0f / sample_freq_;
    f32 gx_real = gx_ - x.wx_bias();
    f32 gy_real = gy_ - x.wy_bias();
    f32 gz_real = gz_;  // 大部分时间机体z轴通天，无法用加速度计校准

    f32 q0_last = x.q0();
    f32 q1_last = x.q1();
    f32 q2_last = x.q2();
    f32 q3_last = x.q3();

    // F = df/dx
    this->F.setIdentity();  // 现在 F = I

    Kalman::Matrix<f32, 4, 4> Fq;
    Fq << 0, -gx_real, -gy_real, -gz_real,  //
        gx_real, 0, gz_real, -gy_real,      //
        gy_real, -gz_real, 0, gx_real,      //
        gz_real, gy_real, -gx_real, 0;      //
    this->F.template block<4, 4>(S::Q0, S::Q0) += Fq * dt * 0.5f;

    Kalman::Matrix<f32, 4, 2> Fw;
    Fw << q1_last, q2_last,  //
        -q0_last, q3_last,   //
        -q3_last, -q0_last,  //
        q2_last, -q1_last;   //
    this->F.template block<4, 2>(S::Q0, S::WX_BIAS) += Fw * dt * 0.5f;

    // W = df/dw
    this->W.setIdentity();  // TODO: 过程模型噪声协方差矩阵
  }

 private:
  f32 sample_freq_;
  f32 gx_, gy_, gz_;
};

class accelMeasurement : public Kalman::Vector<f32, 3> {
 public:
  KALMAN_VECTOR(accelMeasurement, f32, 3)

  //! 加速度计x轴数据
  static constexpr size_t AX = 0;
  //! 加速度计y轴数据
  static constexpr size_t AY = 1;
  //! 加速度计z轴数据
  static constexpr size_t AZ = 2;

  f32 ax() const { return (*this)[AX]; }
  f32 ay() const { return (*this)[AY]; }
  f32 az() const { return (*this)[AZ]; }

  f32 &ax() { return (*this)[AX]; }
  f32 &ay() { return (*this)[AY]; }
  f32 &az() { return (*this)[AZ]; }
};

template <template <class> class CovarianceBase = Kalman::StandardBase>
class accelMeasurementModel : public Kalman::LinearizedMeasurementModel<State, accelMeasurement, CovarianceBase> {
 public:
  //! 状态向量简写
  using S = rm::modules::algorithm::State;
  //! 测量向量简写
  using M = rm::modules::algorithm::accelMeasurement;

  // 构造函数
  accelMeasurementModel() {
    this->H.setIdentity();
    this->V.setIdentity();
  }

  M h(const S &x) const {
    M measurement;

    measurement << 2 * (x.q1() * x.q3() - x.q0() * x.q2()),  //
        2 * (x.q2() * x.q3() + x.q0() * x.q1()),             //
        1 - 2 * (x.q1() * x.q1() + x.q2() * x.q2());         //

    return measurement;
  }

 protected:
  void updateJacobians(const S &x) {
    this->H.setZero();

    this->H.template block<3, 4>(M::AX, S::Q0) << -2 * x.q2(), 2 * x.q3(), -2 * x.q0(), 2 * x.q1(),  //
        2 * x.q1(), 2 * x.q0(), 2 * x.q3(), 2 * x.q2(),                                              //
        2 * x.q0(), -2 * x.q1(), -2 * x.q2(), 2 * x.q3();                                            //

    this->V.setIdentity();
  }
};

class EkfAhrs : public AhrsInterface {
 public:
  explicit EkfAhrs(f32 sample_freq);
  ~EkfAhrs() override = default;

  void Update(const ImuData9Dof &data) override;
  void Update(const ImuData6Dof &data) override;
  [[nodiscard]] const EulerAngle &euler_angle() const override;
  [[nodiscard]] const Quaternion &quaternion() const override;

 private:
  f32 sample_freq_;
  f32 q0_, q1_, q2_, q3_;
  Quaternion quaternion_;
  EulerAngle euler_ypr_;

  State x_;
  Control u_;
  SystemModel<> sys_;
  accelMeasurement am_;
  accelMeasurementModel<> amm_;
  Kalman::ExtendedKalmanFilter<State> ekf_;
};

}  // namespace rm::modules::algorithm

#endif  // LIBRM_MODULES_ALGORITHM_EKF_H