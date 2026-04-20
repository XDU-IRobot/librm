/*
  Copyright (c) 2026 XDU-IRobot

  Permission is hereby granted, free of charge, to any person obtaining a copy ...
*/

#include "librm/modules/ahrs/kalman_filter.hpp"

namespace rm::modules {

template <typename T>
static T* AllocateArray(size_t len) {
  T* ptr = new T[len]();
  return ptr;
}

template <typename T>
static void FreeArray(T* ptr) {
  if (ptr) {
    delete[] ptr;
  }
}

KalmanFilter::KalmanFilter(u8 xhatSize, u8 uSize, u8 zSize)
    : xhatSize(xhatSize),
      uSize(uSize),
      zSize(zSize),
      UseAutoAdjustment(0),
      MeasurementValidNum(0),
      SkipEq1(false),
      SkipEq2(false),
      SkipEq3(false),
      SkipEq4(false),
      SkipEq5(false),
      u_data(nullptr),
      B_data(nullptr) {
  const u16 max_dim = (xhatSize > zSize) ? xhatSize : zSize;

  MeasurementMap = AllocateArray<u8>(zSize);
  MeasurementDegree = AllocateArray<f32>(zSize);
  MatR_DiagonalElements = AllocateArray<f32>(zSize);
  StateMinVariance = AllocateArray<f32>(xhatSize);
  temp = AllocateArray<u8>(zSize);

  FilteredValue = AllocateArray<f32>(xhatSize);
  MeasuredVector = AllocateArray<f32>(zSize);
  ControlVector = AllocateArray<f32>(uSize);

  xhat_data = AllocateArray<f32>(xhatSize);
  Matrix_Init(&xhat, xhatSize, 1, xhat_data);

  xhatminus_data = AllocateArray<f32>(xhatSize);
  Matrix_Init(&xhatminus, xhatSize, 1, xhatminus_data);

  if (uSize != 0) {
    u_data = AllocateArray<f32>(uSize);
    Matrix_Init(&u, uSize, 1, u_data);
    B_data = AllocateArray<f32>(xhatSize * uSize);
    Matrix_Init(&B, xhatSize, uSize, B_data);
  }

  z_data = AllocateArray<f32>(zSize);
  Matrix_Init(&z, zSize, 1, z_data);

  P_data = AllocateArray<f32>(xhatSize * xhatSize);
  Matrix_Init(&P, xhatSize, xhatSize, P_data);

  Pminus_data = AllocateArray<f32>(xhatSize * xhatSize);
  Matrix_Init(&Pminus, xhatSize, xhatSize, Pminus_data);

  F_data = AllocateArray<f32>(xhatSize * xhatSize);
  FT_data = AllocateArray<f32>(xhatSize * xhatSize);
  Matrix_Init(&F, xhatSize, xhatSize, F_data);
  Matrix_Init(&FT, xhatSize, xhatSize, FT_data);

  H_data = AllocateArray<f32>(zSize * xhatSize);
  HT_data = AllocateArray<f32>(xhatSize * zSize);
  Matrix_Init(&H, zSize, xhatSize, H_data);
  Matrix_Init(&HT, xhatSize, zSize, HT_data);

  Q_data = AllocateArray<f32>(xhatSize * xhatSize);
  Matrix_Init(&Q, xhatSize, xhatSize, Q_data);

  R_data = AllocateArray<f32>(zSize * zSize);
  Matrix_Init(&R, zSize, zSize, R_data);

  K_data = AllocateArray<f32>(xhatSize * zSize);
  Matrix_Init(&K, xhatSize, zSize, K_data);

  S_data = AllocateArray<f32>(max_dim * max_dim);
  temp_matrix_data = AllocateArray<f32>(max_dim * max_dim);
  temp_matrix_data1 = AllocateArray<f32>(max_dim * max_dim);
  temp_vector_data = AllocateArray<f32>(max_dim);
  temp_vector_data1 = AllocateArray<f32>(max_dim);

  Matrix_Init(&S, max_dim, max_dim, S_data);
  Matrix_Init(&temp_matrix, max_dim, max_dim, temp_matrix_data);
  Matrix_Init(&temp_matrix1, max_dim, max_dim, temp_matrix_data1);
  Matrix_Init(&temp_vector, max_dim, 1, temp_vector_data);
  Matrix_Init(&temp_vector1, max_dim, 1, temp_vector_data1);
}

KalmanFilter::~KalmanFilter() {
  FreeArray(MeasurementMap);
  FreeArray(MeasurementDegree);
  FreeArray(MatR_DiagonalElements);
  FreeArray(StateMinVariance);
  FreeArray(temp);
  FreeArray(FilteredValue);
  FreeArray(MeasuredVector);
  FreeArray(ControlVector);
  FreeArray(xhat_data);
  FreeArray(xhatminus_data);
  FreeArray(u_data);
  FreeArray(z_data);
  FreeArray(P_data);
  FreeArray(Pminus_data);
  FreeArray(F_data);
  FreeArray(FT_data);
  FreeArray(B_data);
  FreeArray(H_data);
  FreeArray(HT_data);
  FreeArray(Q_data);
  FreeArray(R_data);
  FreeArray(K_data);
  FreeArray(S_data);
  FreeArray(temp_matrix_data);
  FreeArray(temp_matrix_data1);
  FreeArray(temp_vector_data);
  FreeArray(temp_vector_data1);
}

void KalmanFilter::Measure() {
  if (UseAutoAdjustment != 0) {
    HKRAdjustment();
  } else {
    std::memcpy(z_data, MeasuredVector, sizeof(f32) * zSize);
    std::memset(MeasuredVector, 0, sizeof(f32) * zSize);
  }

  if (uSize != 0) {
    std::memcpy(u_data, ControlVector, sizeof(f32) * uSize);
  }
}

void KalmanFilter::UpdateXhatMinus() {
  if (!SkipEq1) {
    if (uSize > 0) {
      temp_vector.numRows = xhatSize;
      temp_vector.numCols = 1;
      MatStatus = Matrix_Multiply(&F, &xhat, &temp_vector);

      temp_vector1.numRows = xhatSize;
      temp_vector1.numCols = 1;
      MatStatus = Matrix_Multiply(&B, &u, &temp_vector1);

      MatStatus = Matrix_Add(&temp_vector, &temp_vector1, &xhatminus);
    } else {
      MatStatus = Matrix_Multiply(&F, &xhat, &xhatminus);
    }
  }
}

void KalmanFilter::UpdatePMinus() {
  if (!SkipEq2) {
    MatStatus = Matrix_Transpose(&F, &FT);
    MatStatus = Matrix_Multiply(&F, &P, &Pminus);

    temp_matrix.numRows = Pminus.numRows;
    temp_matrix.numCols = FT.numCols;
    MatStatus = Matrix_Multiply(&Pminus, &FT, &temp_matrix);

    MatStatus = Matrix_Add(&temp_matrix, &Q, &Pminus);
  }
}

void KalmanFilter::SetK() {
  if (!SkipEq3) {
    MatStatus = Matrix_Transpose(&H, &HT);

    temp_matrix.numRows = H.numRows;
    temp_matrix.numCols = Pminus.numCols;
    MatStatus = Matrix_Multiply(&H, &Pminus, &temp_matrix);

    temp_matrix1.numRows = temp_matrix.numRows;
    temp_matrix1.numCols = HT.numCols;
    MatStatus = Matrix_Multiply(&temp_matrix, &HT, &temp_matrix1);

    S.numRows = R.numRows;
    S.numCols = R.numCols;
    MatStatus = Matrix_Add(&temp_matrix1, &R, &S);
    MatStatus = Matrix_Inverse(&S, &temp_matrix1);

    temp_matrix.numRows = Pminus.numRows;
    temp_matrix.numCols = HT.numCols;
    MatStatus = Matrix_Multiply(&Pminus, &HT, &temp_matrix);
    MatStatus = Matrix_Multiply(&temp_matrix, &temp_matrix1, &K);
  }
}

void KalmanFilter::UpdateXhat() {
  if (!SkipEq4) {
    temp_vector.numRows = H.numRows;
    temp_vector.numCols = 1;
    MatStatus = Matrix_Multiply(&H, &xhatminus, &temp_vector);

    temp_vector1.numRows = z.numRows;
    temp_vector1.numCols = 1;
    MatStatus = Matrix_Subtract(&z, &temp_vector, &temp_vector1);

    temp_vector.numRows = K.numRows;
    temp_vector.numCols = 1;
    MatStatus = Matrix_Multiply(&K, &temp_vector1, &temp_vector);

    MatStatus = Matrix_Add(&xhatminus, &temp_vector, &xhat);
  }
}

void KalmanFilter::UpdateP() {
  if (!SkipEq5) {
    temp_matrix.numRows = K.numRows;
    temp_matrix.numCols = H.numCols;

    temp_matrix1.numRows = temp_matrix.numRows;
    temp_matrix1.numCols = Pminus.numCols;

    MatStatus = Matrix_Multiply(&K, &H, &temp_matrix);
    MatStatus = Matrix_Multiply(&temp_matrix, &Pminus, &temp_matrix1);
    MatStatus = Matrix_Subtract(&Pminus, &temp_matrix1, &P);
  }
}

f32* KalmanFilter::Update() {
  // 0. 获取量测信息
  Measure();
  if (User_Func0_f) User_Func0_f(this);

  // 1. 先验估计
  UpdateXhatMinus();
  if (User_Func1_f) User_Func1_f(this);

  // 2. 预测更新
  UpdatePMinus();
  if (User_Func2_f) User_Func2_f(this);

  if (MeasurementValidNum != 0 || UseAutoAdjustment == 0) {
    // 3. 量测更新
    SetK();
    if (User_Func3_f) User_Func3_f(this);

    // 4. 融合
    UpdateXhat();
    if (User_Func4_f) User_Func4_f(this);

    // 5. 修正方差
    UpdateP();
  } else {
    // 无有效量测，仅预测
    std::memcpy(xhat_data, xhatminus_data, sizeof(f32) * xhatSize);
    std::memcpy(P_data, Pminus_data, sizeof(f32) * xhatSize * xhatSize);
  }

  if (User_Func5_f) User_Func5_f(this);

  // 避免滤波器过度收敛
  for (u8 i = 0; i < xhatSize; i++) {
    if (P_data[i * xhatSize + i] < StateMinVariance[i]) {
      P_data[i * xhatSize + i] = StateMinVariance[i];
    }
  }

  std::memcpy(FilteredValue, xhat_data, sizeof(f32) * xhatSize);

  if (User_Func6_f) User_Func6_f(this);

  return FilteredValue;
}

void KalmanFilter::HKRAdjustment() {
  MeasurementValidNum = 0;

  std::memcpy(z_data, MeasuredVector, sizeof(f32) * zSize);
  std::memset(MeasuredVector, 0, sizeof(f32) * zSize);

  std::memset(R_data, 0, sizeof(f32) * zSize * zSize);
  std::memset(H_data, 0, sizeof(f32) * xhatSize * zSize);

  for (u8 i = 0; i < zSize; i++) {
    if (z_data[i] != 0) {
      z_data[MeasurementValidNum] = z_data[i];
      temp[MeasurementValidNum] = i;
      H_data[xhatSize * MeasurementValidNum + MeasurementMap[i] - 1] = MeasurementDegree[i];
      MeasurementValidNum++;
    }
  }

  for (u8 i = 0; i < MeasurementValidNum; i++) {
    R_data[i * MeasurementValidNum + i] = MatR_DiagonalElements[temp[i]];
  }

  H.numRows = MeasurementValidNum;
  H.numCols = xhatSize;
  HT.numRows = xhatSize;
  HT.numCols = MeasurementValidNum;
  R.numRows = MeasurementValidNum;
  R.numCols = MeasurementValidNum;
  K.numRows = xhatSize;
  K.numCols = MeasurementValidNum;
  z.numRows = MeasurementValidNum;
}

}  // namespace rm::modules
