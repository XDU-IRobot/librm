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
 * @file  librm/modules/trajectory_limiter.hpp
 * @brief 轨迹限制器
 */

#include "librm/core/typedefs.hpp"

namespace rm::modules {

/**
 * @brief   轨迹限制器
 *          给定最大速度和最大加速度，限制输入信号，使其不超过速度和加速度约束。
 *          如果输入信号本身未超出约束，则输出与输入相同。
 */
class TrajectoryLimiter {
 public:
  /**
   * @param max_vel 最大速度，单位位置/秒
   * @param max_accel 最大加速度，单位位置/秒^2
   */
  TrajectoryLimiter(f32 max_vel, f32 max_accel);

  /**
   * @brief 设置目标位置
   * @param target 目标位置
   */
  void SetTarget(f32 target);

  /**
   * @brief 重置限制器，并设置初始位置
   * @param position 初始位置
   */
  void ResetAt(f32 position);

  /**
   * @brief         更新位置和速度
   * @param     dt  时间增量，单位秒
   * @return        更新后的当前位置
   */
  f32 Update(f32 dt);

  /**
   * @brief     检查是否到达目标位置
   * @param     tolerance 容差
   * @return    若已到达目标位置则返回true，否则返回false
   */
  bool IsAtTarget(f32 tolerance = 1e-6f) const;

  /** Getters */
  f32 current_position() const;
  f32 current_velocity() const;
  f32 target_position() const;

 private:
  f32 max_vel_;
  f32 max_accel_;
  f32 current_pos_{0.0f};
  f32 target_pos_{0.0f};
  f32 current_vel_{0.0f};
};

}  // namespace rm::modules
