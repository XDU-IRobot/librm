/*
  Copyright (c) 2026 XDU-IRobot

  Permission is hereby granted, free of charge ...
*/

#include "librm/modules/ahrs/quaternion_ekf.hpp"

namespace rm::modules {

const f32 QuaternionEkfAhrs::IMU_QuaternionEKF_F[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

QuaternionEkfAhrs::QuaternionEkfAhrs(f32 sample_freq, const f32 *init_quaternion, f32 process_noise1, f32 process_noise2,
                                     f32 measure_noise, f32 lambda, f32 lpf) {
  dt = 1.0f / sample_freq;
  Initialized = 1;
  Q1 = process_noise1;
  Q2 = process_noise2;
  R = measure_noise;
  ChiSquareTestThreshold = 1e-8;
  ConvergeFlag = 0;
  ErrorCount = 0;
  UpdateCount = 0;
  if (lambda > 1) {
    lambda = 1;
  }
  this->lambda = lambda;
  accLPFcoef = lpf;

  f32 init_P[36] = {100000, 0.1, 0.1,    0.1, 0.1, 0.1,
                    0.1, 100000, 0.1, 0.1,    0.1, 0.1,
                    0.1,    0.1, 100000, 0.1, 0.1, 0.1,
                    0.1, 0.1,    0.1, 100000, 0.1, 0.1,
                    0.1,    0.1, 0.1,    0.1, 100, 0.1,
                    0.1, 0.1,    0.1, 0.1,    0.1, 100};
  std::memcpy(IMU_QuaternionEKF_P, init_P, sizeof(init_P));
  std::memset(IMU_QuaternionEKF_K, 0, sizeof(IMU_QuaternionEKF_K));
  std::memset(IMU_QuaternionEKF_H, 0, sizeof(IMU_QuaternionEKF_H));

  IMU_QuaternionEKF = std::unique_ptr<KalmanFilter>(new KalmanFilter(6, 0, 3));
  Matrix_Init(&ChiSquare, 1, 1, ChiSquare_Data);

  for (int i = 0; i < 4; i++) {
    IMU_QuaternionEKF->xhat_data[i] = init_quaternion != nullptr ? init_quaternion[i] : (i == 0 ? 1.0f : 0.0f);
  }

  if (init_quaternion != nullptr) {
    quaternion_.w = init_quaternion[0];
    quaternion_.x = init_quaternion[1];
    quaternion_.y = init_quaternion[2];
    quaternion_.z = init_quaternion[3];
  } else {
    quaternion_.w = 1.0f;
    quaternion_.x = 0.0f;
    quaternion_.y = 0.0f;
    quaternion_.z = 0.0f;
  }

  IMU_QuaternionEKF->User_Func0_f = [this](KalmanFilter* kf) { this->Observe(kf); };
  IMU_QuaternionEKF->User_Func1_f = [this](KalmanFilter* kf) { this->F_Linearization_P_Fading(kf); };
  IMU_QuaternionEKF->User_Func2_f = [this](KalmanFilter* kf) { this->SetH(kf); };
  IMU_QuaternionEKF->User_Func3_f = [this](KalmanFilter* kf) { this->xhatUpdate(kf); };

  IMU_QuaternionEKF->SkipEq3 = true;
  IMU_QuaternionEKF->SkipEq4 = true;

  std::memcpy(IMU_QuaternionEKF->F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
  std::memcpy(IMU_QuaternionEKF->P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

void QuaternionEkfAhrs::Update(const ImuData9Dof &data) {
  ImuData6Dof d6{data.gx, data.gy, data.gz, data.ax, data.ay, data.az};
  Update(d6);
}

void QuaternionEkfAhrs::Update(const ImuData6Dof &data) {
  f32 gx = data.gx;
  f32 gy = data.gy;
  f32 gz = data.gz;
  f32 ax = data.ax;
  f32 ay = data.ay;
  f32 az = data.az;

  f32 halfgxdt, halfgydt, halfgzdt;
  f32 accelInvNorm;

  Gyro[0] = gx - GyroBias[0];
  Gyro[1] = gy - GyroBias[1];
  Gyro[2] = gz - GyroBias[2];

  halfgxdt = 0.5f * Gyro[0] * dt;
  halfgydt = 0.5f * Gyro[1] * dt;
  halfgzdt = 0.5f * Gyro[2] * dt;

  std::memcpy(IMU_QuaternionEKF->F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

  IMU_QuaternionEKF->F_data[1] = -halfgxdt;
  IMU_QuaternionEKF->F_data[2] = -halfgydt;
  IMU_QuaternionEKF->F_data[3] = -halfgzdt;

  IMU_QuaternionEKF->F_data[6] = halfgxdt;
  IMU_QuaternionEKF->F_data[8] = halfgzdt;
  IMU_QuaternionEKF->F_data[9] = -halfgydt;

  IMU_QuaternionEKF->F_data[12] = halfgydt;
  IMU_QuaternionEKF->F_data[13] = -halfgzdt;
  IMU_QuaternionEKF->F_data[15] = halfgxdt;

  IMU_QuaternionEKF->F_data[18] = halfgzdt;
  IMU_QuaternionEKF->F_data[19] = halfgydt;
  IMU_QuaternionEKF->F_data[20] = -halfgxdt;

  if (UpdateCount == 0) {
    Accel[0] = ax;
    Accel[1] = ay;
    Accel[2] = az;
  }
  Accel[0] = Accel[0] * accLPFcoef / (this->dt + accLPFcoef) + ax * this->dt / (this->dt + accLPFcoef);
  Accel[1] = Accel[1] * accLPFcoef / (this->dt + accLPFcoef) + ay * this->dt / (this->dt + accLPFcoef);
  Accel[2] = Accel[2] * accLPFcoef / (this->dt + accLPFcoef) + az * this->dt / (this->dt + accLPFcoef);

  accelInvNorm = invSqrt(Accel[0] * Accel[0] + Accel[1] * Accel[1] + Accel[2] * Accel[2]);
  for (u8 i = 0; i < 3; i++) {
    IMU_QuaternionEKF->MeasuredVector[i] = Accel[i] * accelInvNorm;
  }

  gyro_norm = 1.0f / invSqrt(Gyro[0] * Gyro[0] + Gyro[1] * Gyro[1] + Gyro[2] * Gyro[2]);
  accl_norm = 1.0f / accelInvNorm;

  if (gyro_norm < 0.3f && accl_norm > 9.8f - 0.5f && accl_norm < 9.8f + 0.5f) {
    StableFlag = 1;
  } else {
    StableFlag = 0;
  }

  IMU_QuaternionEKF->Q_data[0] = Q1 * this->dt;
  IMU_QuaternionEKF->Q_data[7] = Q1 * this->dt;
  IMU_QuaternionEKF->Q_data[14] = Q1 * this->dt;
  IMU_QuaternionEKF->Q_data[21] = Q1 * this->dt;
  IMU_QuaternionEKF->Q_data[28] = Q2 * this->dt;
  IMU_QuaternionEKF->Q_data[35] = Q2 * this->dt;
  IMU_QuaternionEKF->R_data[0] = R;
  IMU_QuaternionEKF->R_data[4] = R;
  IMU_QuaternionEKF->R_data[8] = R;

  IMU_QuaternionEKF->Update();

  quaternion_.w = IMU_QuaternionEKF->FilteredValue[0];
  quaternion_.x = IMU_QuaternionEKF->FilteredValue[1];
  quaternion_.y = IMU_QuaternionEKF->FilteredValue[2];
  quaternion_.z = IMU_QuaternionEKF->FilteredValue[3];

  GyroBias[0] = IMU_QuaternionEKF->FilteredValue[4];
  GyroBias[1] = IMU_QuaternionEKF->FilteredValue[5];
  GyroBias[2] = 0;  // z轴无法观测漂移

  f32 sinp = 2.0f * (quaternion_.w * quaternion_.y - quaternion_.z * quaternion_.x);
  f32 cosp = 1.0f - 2.0f * (quaternion_.y * quaternion_.y + quaternion_.x * quaternion_.x);
  euler_ypr_.pitch = std::atan2(sinp, cosp) * 57.295779513f;

  f32 yaw_angle = std::atan2(2.0f * (quaternion_.w * quaternion_.z + quaternion_.x * quaternion_.y),
                   2.0f * (quaternion_.w * quaternion_.w + quaternion_.x * quaternion_.x) - 1.0f) *
        57.295779513f;
  euler_ypr_.yaw = yaw_angle;
  euler_ypr_.roll = std::atan2(2.0f * (quaternion_.w * quaternion_.x + quaternion_.y * quaternion_.z),
                    2.0f * (quaternion_.w * quaternion_.w + quaternion_.z * quaternion_.z) - 1.0f) *
         57.295779513f;

  if (yaw_angle - YawAngleLast > 180.0f) {
    YawRoundCount--;
  } else if (yaw_angle - YawAngleLast < -180.0f) {
    YawRoundCount++;
  }
  YawTotalAngle = 360.0f * YawRoundCount + yaw_angle;
  YawAngleLast = yaw_angle;
  UpdateCount++;
}

/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新
 *        并对零漂的方差进行限制 防止过度收敛并限幅防止发散
 */
void QuaternionEkfAhrs::F_Linearization_P_Fading(KalmanFilter *kf) {
  f32 q0, q1, q2, q3;
  f32 qInvNorm;

  q0 = kf->xhatminus_data[0];
  q1 = kf->xhatminus_data[1];
  q2 = kf->xhatminus_data[2];
  q3 = kf->xhatminus_data[3];

  qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  for (u8 i = 0; i < 4; i++) {
    kf->xhatminus_data[i] *= qInvNorm;
  }

  kf->F_data[4] = q1 * dt / 2.0f;
  kf->F_data[5] = q2 * dt / 2.0f;

  kf->F_data[10] = -q0 * dt / 2.0f;
  kf->F_data[11] = q3 * dt / 2.0f;

  kf->F_data[16] = -q3 * dt / 2.0f;
  kf->F_data[17] = -q0 * dt / 2.0f;

  kf->F_data[22] = q2 * dt / 2.0f;
  kf->F_data[23] = -q1 * dt / 2.0f;

  kf->P_data[28] /= lambda;
  kf->P_data[35] /= lambda;

  if (kf->P_data[28] > 10000.0f) {
    kf->P_data[28] = 10000.0f;
  }
  if (kf->P_data[35] > 10000.0f) {
    kf->P_data[35] = 10000.0f;
  }
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 */
void QuaternionEkfAhrs::SetH(KalmanFilter *kf) {
  f32 doubleq0, doubleq1, doubleq2, doubleq3;

  doubleq0 = 2.0f * kf->xhatminus_data[0];
  doubleq1 = 2.0f * kf->xhatminus_data[1];
  doubleq2 = 2.0f * kf->xhatminus_data[2];
  doubleq3 = 2.0f * kf->xhatminus_data[3];

  std::memset(kf->H_data, 0, sizeof(f32) * kf->zSize * kf->xhatSize);

  kf->H_data[0] = -doubleq2;
  kf->H_data[1] = doubleq3;
  kf->H_data[2] = -doubleq0;
  kf->H_data[3] = doubleq1;

  kf->H_data[6] = doubleq1;
  kf->H_data[7] = doubleq0;
  kf->H_data[8] = doubleq3;
  kf->H_data[9] = doubleq2;

  kf->H_data[12] = doubleq0;
  kf->H_data[13] = -doubleq1;
  kf->H_data[14] = -doubleq2;
  kf->H_data[15] = doubleq3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 */
void QuaternionEkfAhrs::xhatUpdate(KalmanFilter *kf) {
  f32 q0, q1, q2, q3;

  kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);
  kf->temp_matrix.numRows = kf->H.numRows;
  kf->temp_matrix.numCols = kf->Pminus.numCols;
  kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);
  kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
  kf->temp_matrix1.numCols = kf->HT.numCols;
  kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);
  kf->S.numRows = kf->R.numRows;
  kf->S.numCols = kf->R.numCols;
  kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);
  kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);

  q0 = kf->xhatminus_data[0];
  q1 = kf->xhatminus_data[1];
  q2 = kf->xhatminus_data[2];
  q3 = kf->xhatminus_data[3];

  kf->temp_vector.numRows = kf->H.numRows;
  kf->temp_vector.numCols = 1;

  kf->temp_vector_data[0] = 2.0f * (q1 * q3 - q0 * q2);
  kf->temp_vector_data[1] = 2.0f * (q0 * q1 + q2 * q3);
  kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  for (u8 i = 0; i < 3; i++) {
    OrientationCosine[i] = std::acos(std::fabs(kf->temp_vector_data[i]));
  }

  kf->temp_vector1.numRows = kf->z.numRows;
  kf->temp_vector1.numCols = 1;
  kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);

  kf->temp_matrix.numRows = kf->temp_vector1.numRows;
  kf->temp_matrix.numCols = 1;
  kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix);
  kf->temp_vector.numRows = 1;
  kf->temp_vector.numCols = kf->temp_vector1.numRows;
  kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector);
  kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &ChiSquare);

  if (ChiSquare_Data[0] < 0.5f * ChiSquareTestThreshold) {
    ConvergeFlag = 1;
  }

  if (ChiSquare_Data[0] > ChiSquareTestThreshold && ConvergeFlag) {
    if (StableFlag) {
      ErrorCount++;
    } else {
      ErrorCount = 0;
    }

    if (ErrorCount > 50) {
      ConvergeFlag = 0;
      kf->SkipEq5 = false;
    } else {
      std::memcpy(kf->xhat_data, kf->xhatminus_data, sizeof(f32) * kf->xhatSize);
      std::memcpy(kf->P_data, kf->Pminus_data, sizeof(f32) * kf->xhatSize * kf->xhatSize);
      kf->SkipEq5 = true;
      return;
    }
  } else {
    if (ChiSquare_Data[0] > 0.1f * ChiSquareTestThreshold && ConvergeFlag) {
      AdaptiveGainScale =
          (ChiSquareTestThreshold - ChiSquare_Data[0]) / (0.9f * ChiSquareTestThreshold);
    } else {
      AdaptiveGainScale = 1.0f;
    }
    ErrorCount = 0;
    kf->SkipEq5 = false;
  }

  kf->temp_matrix.numRows = kf->Pminus.numRows;
  kf->temp_matrix.numCols = kf->HT.numCols;
  kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);
  kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

  for (u8 i = 0; i < kf->K.numRows * kf->K.numCols; i++) {
    kf->K_data[i] *= AdaptiveGainScale;
  }
  for (u8 i = 4; i < 6; i++) {
    for (u8 j = 0; j < 3; j++) {
      kf->K_data[i * 3 + j] *= OrientationCosine[i - 4] / 1.5707963f;
    }
  }

  kf->temp_vector.numRows = kf->K.numRows;
  kf->temp_vector.numCols = 1;
  kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);

  if (ConvergeFlag) {
    for (u8 i = 4; i < 6; i++) {
      if (kf->temp_vector.pData[i] > 1e-2f * dt) {
        kf->temp_vector.pData[i] = 1e-2f * dt;
      }
      if (kf->temp_vector.pData[i] < -1e-2f * dt) {
        kf->temp_vector.pData[i] = -1e-2f * dt;
      }
    }
  }

  kf->temp_vector.pData[3] = 0;
  kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

void QuaternionEkfAhrs::Observe(KalmanFilter *kf) {
  std::memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
  std::memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
  std::memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

f32 QuaternionEkfAhrs::invSqrt(f32 x) {
  f32 halfx = 0.5f * x;
  f32 y = x;
  long i = *(long *)&y;
  i = 0x5f375a86 - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

const EulerAngle &QuaternionEkfAhrs::euler_angle() const {
  return euler_ypr_;
}

const Quaternion &QuaternionEkfAhrs::quaternion() const {
  return quaternion_;
}

}  // namespace rm::modules

