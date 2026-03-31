/*
  Copyright (c) 2026 XDU-IRobot

  Permission is hereby granted, free of charge, to any person obtaining a copy ...
*/

/**
 * @file  librm/modules/ahrs/kalman_filter.hpp
 * @brief Kalman Filter
 */

#ifndef LIBRM_MODULES_AHRS_KALMAN_FILTER_HPP
#define LIBRM_MODULES_AHRS_KALMAN_FILTER_HPP

#include "librm/core/typedefs.hpp"

#ifdef __cplusplus

#if defined(__arm__) || defined(__aarch64__) || defined(STM32)
#include "arm_math.h"
#else
#include <cstdint>
typedef struct {
  uint16_t numRows;
  uint16_t numCols;
  float *pData;
} arm_matrix_instance_f32;

inline void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nCols, float *pData) {
  S->numRows = nRows;
  S->numCols = nCols;
  S->pData = pData;
}
inline int arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst) {
  return 0;
}
inline int arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst) {
  return 0;
}
inline int arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst) {
  return 0;
}
inline int arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst) { return 0; }
inline int arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst) { return 0; }
#endif

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <functional>

namespace rm::modules {

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

class KalmanFilter {
 public:
  KalmanFilter(u8 xhatSize, u8 uSize, u8 zSize);
  ~KalmanFilter();

  // 禁用拷贝构造和赋值
  KalmanFilter(const KalmanFilter&) = delete;
  KalmanFilter& operator=(const KalmanFilter&) = delete;

  void Measure();
  void UpdateXhatMinus();
  void UpdatePMinus();
  void SetK();
  void UpdateXhat();
  void UpdateP();
  f32* Update();

  f32* FilteredValue;
  f32* MeasuredVector;
  f32* ControlVector;

  u8 xhatSize;
  u8 uSize;
  u8 zSize;

  u8 UseAutoAdjustment;
  u8 MeasurementValidNum;

  u8* MeasurementMap;          // 量测与状态的关系
  f32* MeasurementDegree;      // 测量值对应H矩阵元素值
  f32* MatR_DiagonalElements;  // 量测方差
  f32* StateMinVariance;       // 最小方差
  u8* temp;

  bool SkipEq1 = false;
  bool SkipEq2 = false;
  bool SkipEq3 = false;
  bool SkipEq4 = false;
  bool SkipEq5 = false;

  mat xhat, xhatminus, u, z, P, Pminus, F, FT, B, H, HT, Q, R, K;
  mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

  i8 MatStatus;

  // 用户定义函数（C++11风格）
  std::function<void(KalmanFilter*)> User_Func0_f;
  std::function<void(KalmanFilter*)> User_Func1_f;
  std::function<void(KalmanFilter*)> User_Func2_f;
  std::function<void(KalmanFilter*)> User_Func3_f;
  std::function<void(KalmanFilter*)> User_Func4_f;
  std::function<void(KalmanFilter*)> User_Func5_f;
  std::function<void(KalmanFilter*)> User_Func6_f;

  f32 *xhat_data, *xhatminus_data;
  f32* u_data;
  f32* z_data;
  f32 *P_data, *Pminus_data;
  f32 *F_data, *FT_data;
  f32* B_data;
  f32 *H_data, *HT_data;
  f32* Q_data;
  f32* R_data;
  f32* K_data;
  f32 *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;

 private:
  void HKRAdjustment();
};

}  // namespace rm::modules

#endif

#endif  // LIBRM_MODULES_AHRS_KALMAN_FILTER_HPP
