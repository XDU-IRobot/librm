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

namespace rm::modules {

TrajectoryLimiter::TrajectoryLimiter(f32 max_vel, f32 max_accel) : max_vel_(max_vel), max_accel_(max_accel) {}

void TrajectoryLimiter::SetTarget(f32 target) { target_pos_ = target; }

void TrajectoryLimiter::ResetAt(f32 position) {
  current_pos_ = position;
  target_pos_ = position;
  current_vel_ = 0.0f;
}

f32 TrajectoryLimiter::Update(f32 dt) {
  // 1. 计算为了在 dt 时间内到达目标位置所需的速度 (v_d)
  const f32 desired_vel = (target_pos_ - current_pos_) / dt;

  // 2. 计算为了在 dt 时间内达到该速度所需的加速度 (a_d)
  const f32 desired_accel = (desired_vel - current_vel_) / dt;

  // 3. 限制加速度
  const f32 accel = std::clamp(desired_accel, -max_accel_, max_accel_);

  // 4. 根据限制后的加速度更新速度
  f32 next_vel = current_vel_ + accel * dt;

  // 5. 限制速度
  next_vel = std::clamp(next_vel, -max_vel_, max_vel_);

  // 6. 根据最终的速度更新位置
  current_pos_ += next_vel * dt;
  current_vel_ = next_vel;

  return current_pos_;
}

bool TrajectoryLimiter::IsAtTarget(f32 tolerance) const { return std::abs(target_pos_ - current_pos_) < tolerance; }

f32 TrajectoryLimiter::current_position() const { return current_pos_; }
f32 TrajectoryLimiter::current_velocity() const { return current_vel_; }
f32 TrajectoryLimiter::target_position() const { return target_pos_; }

}  // namespace rm::modules
