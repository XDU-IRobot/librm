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
 * @file  librm/modules/trajectory_limiter.cc
 * @brief 轨迹限制器
 */

#include "trajectory_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace rm::modules {

TrajectoryLimiter::TrajectoryLimiter(f32 max_vel, f32 max_accel) : max_vel_(max_vel), max_accel_(max_accel) {}

void TrajectoryLimiter::SetTarget(f32 target) { target_pos_ = target; }

void TrajectoryLimiter::ResetAt(f32 position) {
  current_pos_ = position;
  target_pos_ = position;
  current_vel_ = 0.0f;
}

f32 TrajectoryLimiter::Update(f32 dt) {
  const f32 position_error = target_pos_ - current_pos_;

  // 死区处理：如果位置误差很小且速度也很小，直接到达目标
  const f32 position_deadband = 1e-4f;
  const f32 velocity_deadband = 1e-4f;
  if (std::abs(position_error) < position_deadband && std::abs(current_vel_) < velocity_deadband) {
    current_pos_ = target_pos_;
    current_vel_ = 0.0f;
    return current_pos_;
  }

  // 1. 计算为了在 dt 时间内到达目标位置所需的参考速度
  f32 reference_vel = position_error / dt;

  // 2. 首先检查参考速度是否超出速度限制
  const bool ref_vel_exceeds_limit = std::abs(reference_vel) > max_vel_;

  // 3. 计算达到参考速度所需的加速度
  f32 reference_accel = (reference_vel - current_vel_) / dt;

  // 4. 检查参考加速度是否超出加速度限制
  const bool ref_accel_exceeds_limit = std::abs(reference_accel) > max_accel_;

  // 5. 只有当参考轨迹超出限制时，才需要介入
  if (ref_vel_exceeds_limit || ref_accel_exceeds_limit) {
    // 先限制速度
    reference_vel = std::clamp(reference_vel, -max_vel_, max_vel_);

    // 重新计算加速度
    reference_accel = (reference_vel - current_vel_) / dt;

    // 再限制加速度
    const f32 limited_accel = std::clamp(reference_accel, -max_accel_, max_accel_);

    // 计算实际能达到的速度
    f32 next_vel = current_vel_ + limited_accel * dt;

    // 安全检查：确保不会冲过目标
    // 计算按当前速度刹车需要的距离
    const f32 braking_distance = (next_vel * next_vel) / (2.0f * max_accel_);
    const bool moving_towards_target = (next_vel * position_error) > 0;

    if (moving_towards_target && std::abs(braking_distance) > std::abs(position_error)) {
      // 需要减速以避免冲过目标
      const f32 sign = (position_error > 0) ? 1.0f : -1.0f;
      next_vel = sign * std::sqrt(2.0f * max_accel_ * std::abs(position_error));
    }

    // 更新位置
    const f32 position_delta = next_vel * dt;
    if (std::abs(position_delta) > std::abs(position_error)) {
      current_pos_ = target_pos_;
      current_vel_ = 0.0f;
    } else {
      current_pos_ += position_delta;
      current_vel_ = next_vel;
    }
  } else {
    // 参考轨迹在限制范围内，保持透明，直接跟随
    current_pos_ = target_pos_;
    current_vel_ = reference_vel;
  }

  return current_pos_;
}

bool TrajectoryLimiter::IsAtTarget(f32 tolerance) const { return std::abs(target_pos_ - current_pos_) < tolerance; }

f32 TrajectoryLimiter::current_position() const { return current_pos_; }
f32 TrajectoryLimiter::current_velocity() const { return current_vel_; }
f32 TrajectoryLimiter::target_position() const { return target_pos_; }

}  // namespace rm::modules
