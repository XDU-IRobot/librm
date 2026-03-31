/*
  Copyright (c) 2026 XDU-IRobot

  Permission is hereby granted, free of charge ...
*/

/**
 * @file  librm/modules/ahrs/quaternion_ekf.hpp
 * @brief Quaternion EKF AHRS
 */

#ifndef LIBRM_MODULES_AHRS_QUATERNION_EKF_HPP
#define LIBRM_MODULES_AHRS_QUATERNION_EKF_HPP

#include "librm/modules/ahrs/ahrs_interface.hpp"
#include "librm/modules/ahrs/kalman_filter.hpp"
#include <memory>

namespace rm::modules {

class QuaternionEkfAhrs : public AhrsInterface {
public:
  explicit QuaternionEkfAhrs(f32 sample_freq = 1000.0f,
                             const f32* init_quaternion = nullptr,
                             f32 process_noise1 = 1e-5f,
                             f32 process_noise2 = 1e-6f,
                             f32 measure_noise = 1e-2f,
                             f32 lambda = 1.0f,
                             f32 lpf = 0.001f);
  ~QuaternionEkfAhrs() override = default;void SetUpdatePeriod(float x);

  // 禁用拷贝构造和赋值
  QuaternionEkfAhrs(const QuaternionEkfAhrs&) = delete;
  QuaternionEkfAhrs& operator=(const QuaternionEkfAhrs&) = delete;

  /**
   * @brief 更新AHRS (6DOF)
   */
  void Update(const ImuData6Dof &data) override;

  /**
   * @brief 更新AHRS (9DOF) 不支持，内部将等同于6DOF调用但忽略磁力计
   */
  void Update(const ImuData9Dof &data) override;

  [[nodiscard]] const EulerAngle &euler_angle() const override;
  [[nodiscard]] const Quaternion &quaternion() const override;

  u8 Initialized{0};
  std::unique_ptr<KalmanFilter> IMU_QuaternionEKF;
  u8 ConvergeFlag{0};
  u8 StableFlag{0};
  u64 ErrorCount{0};
  u64 UpdateCount{0};

  EulerAngle euler_ypr_{0, 0, 0};
  Quaternion quaternion_{1, 0, 0, 0};

  f32 GyroBias[3]{0};  // 陀螺仪零偏估计值
  f32 Gyro[3]{0};
  f32 Accel[3]{0};

  f32 OrientationCosine[3]{0};

  f32 accLPFcoef{0};
  f32 gyro_norm{0};
  f32 accl_norm{0};
  f32 AdaptiveGainScale{0};

  f32 YawTotalAngle{0};

  f32 Q1{0};  // 四元数更新过程噪声
  f32 Q2{0};  // 陀螺仪零偏过程噪声
  f32 R{0};   // 加速度计观测噪声

  f32 dt{0.001f};  // 姿态更新周期, 默认1ms
  mat ChiSquare;
  f32 ChiSquare_Data[1]{0};       // 卡方检验检测函数
  f32 ChiSquareTestThreshold{0};  // 卡方检验阈值
  f32 lambda{0};                  // 渐消因子

  i16 YawRoundCount{0};
  f32 YawAngleLast{0};

private:
  void Observe(KalmanFilter *kf);
  void F_Linearization_P_Fading(KalmanFilter *kf);
  void SetH(KalmanFilter *kf);
  void xhatUpdate(KalmanFilter *kf);

  static f32 invSqrt(f32 x);

  f32 IMU_QuaternionEKF_P[36];
  f32 IMU_QuaternionEKF_K[18];
  f32 IMU_QuaternionEKF_H[18];
  static const f32 IMU_QuaternionEKF_F[36];
};

}  // namespace rm::modules

#endif  // LIBRM_MODULES_AHRS_QUATERNION_EKF_HPP

