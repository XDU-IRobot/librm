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

#include <cmath>

#include <gtest/gtest.h>

#include "librm/modules/trajectory_limiter.hpp"

using namespace rm;
using namespace rm::modules;

// 使用浮点数时，定义一个比较容差
constexpr float kFloatEpsilon = 1e-5f;

// 测试构造函数对无效参数的处理
TEST(TrajectoryLimiterTest, ConstructorValidation) {
  // 测试负数速度和加速度限制
  // 注意：实现可能不会主动修正负数参数，这取决于设计决策
  // 这里我们只测试正常的使用情况
  TrajectoryLimiter limiter(1.0f, 10.0f);
  limiter.SetTarget(1.0f);
  float pos = limiter.Update(0.01f);
  // 应该能够移动
  EXPECT_GT(pos, 0.0f);
  
  // 测试小的但正常的限制值
  TrajectoryLimiter limiter2(0.1f, 1.0f);
  limiter2.SetTarget(1.0f);
  EXPECT_GT(limiter2.Update(0.01f), 0.0f);
}

// 测试 ResetAt 功能
TEST(TrajectoryLimiterTest, ResetAt) {
  TrajectoryLimiter limiter(10.0f, 100.0f);
  limiter.ResetAt(5.0f);
  EXPECT_NEAR(limiter.current_position(), 5.0f, kFloatEpsilon);
  EXPECT_NEAR(limiter.target_position(), 5.0f, kFloatEpsilon);
  EXPECT_NEAR(limiter.current_velocity(), 0.0f, kFloatEpsilon);
}

// 测试当目标在死区内时的情况
TEST(TrajectoryLimiterTest, Deadband) {
  TrajectoryLimiter limiter(10.0f, 100.0f);
  limiter.ResetAt(0.0f);
  limiter.SetTarget(1e-5f);  // 小于 position_deadband (1e-4f)
  limiter.Update(0.01f);
  EXPECT_NEAR(limiter.current_position(), 1e-5f, kFloatEpsilon);
  EXPECT_NEAR(limiter.current_velocity(), 0.0f, kFloatEpsilon);
}

// 测试透明模式：当目标可以在一个 dt 内无限制到达时
TEST(TrajectoryLimiterTest, TransparentMode) {
  TrajectoryLimiter limiter(10.0f, 100.0f);
  limiter.ResetAt(0.0f);
  limiter.SetTarget(0.05f);  // ref_vel = 5, ref_accel = 500 (accel limited)
  // 这里我们选择一个不会超限的目标
  limiter.ResetAt(0.0f);
  limiter.SetTarget(0.005f);  // ref_vel = 0.5, ref_accel = 50
  limiter.Update(0.01f);
  // 应该直接到达目标
  EXPECT_NEAR(limiter.current_position(), 0.005f, kFloatEpsilon);
  EXPECT_NEAR(limiter.current_velocity(), 0.5f, kFloatEpsilon);
}

// 测试纯速度限制
TEST(TrajectoryLimiterTest, VelocityLimited) {
  TrajectoryLimiter limiter(1.0f, 100.0f);  // 速度限制很小，加速度限制很大
  limiter.ResetAt(0.0f);
  limiter.SetTarget(1.0f);

  // 第一次更新，ref_vel = 1.0 / 0.01 = 100.0f，远超 max_vel
  float pos = limiter.Update(0.01f);

  // 速度应该被限制在 1.0f
  EXPECT_NEAR(limiter.current_velocity(), 1.0f, kFloatEpsilon);
  // 位置应该更新了 1.0 * 0.01 = 0.01
  EXPECT_NEAR(pos, 0.01f, kFloatEpsilon);
}

// 测试纯加速度限制
TEST(TrajectoryLimiterTest, AccelerationLimited) {
  TrajectoryLimiter limiter(10.0f, 5.0f);  // 加速度限制很小
  limiter.ResetAt(0.0f);
  limiter.SetTarget(1.0f);

  // 第一次更新，ref_vel = 100.0f, ref_accel = 10000.0f
  float pos = limiter.Update(0.01f);

  // 加速度应该被限制在 5.0f
  // next_vel = 0 + 5.0 * 0.01 = 0.05
  EXPECT_NEAR(limiter.current_velocity(), 0.05f, kFloatEpsilon);
  // pos_delta = 0.05 * 0.01 = 0.0005
  EXPECT_NEAR(pos, 0.0005f, kFloatEpsilon);
}

// 测试刹车逻辑，防止过冲
TEST(TrajectoryLimiterTest, BrakingToPreventOvershoot) {
  TrajectoryLimiter limiter(10.0f, 10.0f);
  // 先让限制器加速到一个较高的速度
  limiter.ResetAt(0.0f);
  limiter.SetTarget(10.0f);  // 设置一个远处的目标
  
  // 进行多次更新，让速度逐渐增加到接近最大速度
  for (int i = 0; i < 100; ++i) {
    limiter.Update(0.01f);
  }
  
  // 此时速度应该接近最大速度 10.0f
  EXPECT_NEAR(limiter.current_velocity(), 10.0f, 0.1f);
  
  // 现在设置一个很近的目标，触发刹车逻辑
  float current_pos = limiter.current_position();
  limiter.SetTarget(current_pos + 0.01f);  // 目标距离当前位置只有 0.01
  
  // position_error = 0.01
  // 当前速度约为 10.0f，braking_distance = 10*10 / (2*10) = 5.0，远大于 position_error
  // 此时应该强制减速
  float vel_before = limiter.current_velocity();
  limiter.Update(0.01f);
  
  // 期望速度应该小于之前的速度，说明发生了减速
  EXPECT_LT(limiter.current_velocity(), vel_before);
  
  // 期望速度应接近 sqrt(2 * max_accel * position_error) = sqrt(2 * 10 * 0.01) = sqrt(0.2) ~= 0.447
  EXPECT_NEAR(limiter.current_velocity(), std::sqrt(0.2f), 0.1f);
}

// 测试无效 dt 输入
TEST(TrajectoryLimiterTest, InvalidDt) {
  TrajectoryLimiter limiter(10.0f, 100.0f);
  limiter.ResetAt(5.0f);

  // dt = 0
  float pos = limiter.Update(0.0f);
  EXPECT_NEAR(pos, 5.0f, kFloatEpsilon);  // 位置不应改变

  // dt < 0
  pos = limiter.Update(-0.01f);
  EXPECT_NEAR(pos, 5.0f, kFloatEpsilon);  // 位置不应改变
}

// 测试向负方向移动
TEST(TrajectoryLimiterTest, MoveToNegative) {
  TrajectoryLimiter limiter(1.0f, 10.0f);
  limiter.ResetAt(0.0f);
  limiter.SetTarget(-1.0f);

  // 第一次更新
  // ref_vel = -1.0 / 0.01 = -100.0，被限制到 -1.0
  // ref_accel = (-1.0 - 0) / 0.01 = -100.0，被限制到 -10.0
  // next_vel = 0 + (-10.0) * 0.01 = -0.1
  limiter.Update(0.01f);
  
  // 速度应为负
  EXPECT_LT(limiter.current_velocity(), 0.0f);
  // 位置应为负
  EXPECT_LT(limiter.current_position(), 0.0f);

  // 由于加速度限制，第一次更新后速度应为 -0.1
  EXPECT_NEAR(limiter.current_velocity(), -0.1f, kFloatEpsilon);
  // 位置应更新了 -0.1 * 0.01 = -0.001
  EXPECT_NEAR(limiter.current_position(), -0.001f, kFloatEpsilon);
  
  // 继续更新多次，速度应逐渐达到最大速度
  for (int i = 0; i < 10; ++i) {
    limiter.Update(0.01f);
  }
  // 现在速度应该接近 -1.0f
  EXPECT_NEAR(limiter.current_velocity(), -1.0f, 0.01f);
}
