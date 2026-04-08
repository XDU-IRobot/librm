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
 * @file  librm/modules/chassis_fkik.hpp
 * @brief 各种常见底盘构型的正逆运动学解算
 */

#ifndef LIBRM_MODULES_CHASSIS_FKIK_HPP
#define LIBRM_MODULES_CHASSIS_FKIK_HPP

#include <cmath>
#include <initializer_list>

#include "librm/core/typedefs.hpp"
#include "librm/modules/utils.hpp"

namespace rm::modules {

/**
 * @brief max函数，由于一些神秘原因gcc-arm-none-eabi没有实现这个函数？？？？？？？（问号脸
 */
template <typename Tp>
static constexpr Tp max_(std::initializer_list<Tp> il) {
  if (il.size() == 0) {
    return 0;
  }
  Tp max = *il.begin();
  for (auto &i : il) {
    if (i > max) {
      max = i;
    }
  }
  return max;
}

/**
 * @brief 麦轮底盘
 */
class MecanumChassis {
 public:
  MecanumChassis() = delete;
  ~MecanumChassis() = default;
  MecanumChassis(f32 wheel_base, f32 wheel_track) : wheel_base_(wheel_base), wheel_track_(wheel_track) {}

  /**
   * @param vx    x轴方向的速度
   * @param vy    y轴方向的速度
   * @param wz    z轴方向的角速度
   */
  auto Forward(f32 vx, f32 vy, f32 wz) {
    forward_result_.lf_speed = vx - vy - (wheel_base_ + wheel_track_) * wz;
    forward_result_.lr_speed = vx + vy + (wheel_base_ + wheel_track_) * wz;
    forward_result_.rf_speed = vx + vy - (wheel_base_ + wheel_track_) * wz;
    forward_result_.rr_speed = vx - vy + (wheel_base_ + wheel_track_) * wz;
    return forward_result_;
  }
  // TODO: ik
  auto forward_result() const { return forward_result_; }

 private:
  struct {
    f32 lf_speed, rf_speed, lr_speed, rr_speed;
  } forward_result_{};
  f32 wheel_base_;   // 轮子间距
  f32 wheel_track_;  // 轮子轴距
};

/**
 * @brief 四舵轮底盘
 */
class SteeringChassis {
 public:
  SteeringChassis() = delete;
  ~SteeringChassis() = default;
  explicit SteeringChassis(f32 chassis_radius) : chassis_radius_(chassis_radius) {}

  /**
   * @brief 360度四舵轮正运动学
   * @param vx                左右方向速度
   * @param vy                前后方向速度
   * @param wz                旋转速度，从上向下看顺时针为正
   * @note
   * 这个函数不考虑当前舵角与目标角度是否大于90度而反转舵，效率较低，如果有条件还是建议用另外一个Forward函数的重载版本
   */
  auto Forward(f32 vx, f32 vy, f32 w) {
    if (vx == 0.f && vy == 0.f && w == 0.f) {  // 理论上不应该直接比较浮点数，但是两个同类型浮点字面量比较应该没问题
      forward_result_.lf_steer_position = M_PI / 4.f;
      forward_result_.rf_steer_position = 3 * M_PI / 4.f;
      forward_result_.lr_steer_position = 3 * M_PI / 4.f;
      forward_result_.rr_steer_position = M_PI / 4.f;
    } else {
      forward_result_.lf_steer_position =
          atan2f(vy + w * chassis_radius_ * sqrtf(2) / 2, vx + w * chassis_radius_ * sqrtf(2) / 2);
      forward_result_.rf_steer_position =
          atan2f(vy + w * chassis_radius_ * sqrtf(2) / 2, vx - w * chassis_radius_ * sqrtf(2) / 2);
      forward_result_.lr_steer_position =
          atan2f(vy - w * chassis_radius_ * sqrtf(2) / 2, vx + w * chassis_radius_ * sqrtf(2) / 2);
      forward_result_.rr_steer_position =
          atan2f(vy - w * chassis_radius_ * sqrtf(2) / 2, vx - w * chassis_radius_ * sqrtf(2) / 2);
    }

    forward_result_.lf_wheel_speed =
        sqrtf(pow(vy - w * chassis_radius_ * sqrtf(2) / 2, 2) + pow(vx - w * chassis_radius_ * sqrtf(2) / 2, 2));
    forward_result_.rf_wheel_speed =
        sqrtf(pow(vy + w * chassis_radius_ * sqrtf(2) / 2, 2) + pow(vx - w * chassis_radius_ * sqrtf(2) / 2, 2));
    forward_result_.lr_wheel_speed =
        sqrtf(pow(vy + w * chassis_radius_ * sqrtf(2) / 2, 2) + pow(vx + w * chassis_radius_ * sqrtf(2) / 2, 2));
    forward_result_.rr_wheel_speed =
        sqrtf(pow(vy - w * chassis_radius_ * sqrtf(2) / 2, 2) + pow(vx + w * chassis_radius_ * sqrtf(2) / 2, 2));

    return forward_result_;
  }

  /**
   * @brief 180度四舵轮正运动学
   * @param vx                左右方向速度
   * @param vy                前后方向速度
   * @param wz                旋转速度，从上向下看顺时针为正
   * @param current_lf_angle  当前的左前舵角度，弧度制，底盘前进方向为0
   * @param current_rf_angle  当前的右前舵角度，弧度制，底盘前进方向为0
   * @param current_lr_angle  当前的左后舵角度，弧度制，底盘前进方向为0
   * @param current_rr_angle  当前的右后舵角度，弧度制，底盘前进方向为0
   */
  auto Forward(f32 vx, f32 vy, f32 w, f32 current_lf_angle, f32 current_rf_angle, f32 current_lr_angle,
               f32 current_rr_angle) {
    Forward(vx, vy, w);

    // 依次计算每个舵的目标角度和目前角度的差值，如果差值大于90度，就把目标舵角加180度，轮速取反。
    // 这样可以保证舵角变化量始终小于90度，加快舵的响应速度
    if (std::abs(Wrap(current_lf_angle - forward_result_.lf_steer_position, -M_PI, M_PI)) > M_PI / 2) {
      forward_result_.lf_steer_position = Wrap(forward_result_.lf_steer_position + M_PI, 0, 2 * M_PI);
      forward_result_.lf_wheel_speed = -forward_result_.lf_wheel_speed;
    }
    if (std::abs(Wrap(current_rf_angle - forward_result_.rf_steer_position, -M_PI, M_PI)) > M_PI / 2) {
      forward_result_.rf_steer_position = Wrap(forward_result_.rf_steer_position + M_PI, 0, 2 * M_PI);
      forward_result_.rf_wheel_speed = -forward_result_.rf_wheel_speed;
    }
    if (std::abs(Wrap(current_lr_angle - forward_result_.lr_steer_position, -M_PI, M_PI)) > M_PI / 2) {
      forward_result_.lr_steer_position = Wrap(forward_result_.lr_steer_position + M_PI, 0, 2 * M_PI);
      forward_result_.lr_wheel_speed = -forward_result_.lr_wheel_speed;
    }
    if (std::abs(Wrap(current_rr_angle - forward_result_.rr_steer_position, -M_PI, M_PI)) > M_PI / 2) {
      forward_result_.rr_steer_position = Wrap(forward_result_.rr_steer_position + M_PI, 0, 2 * M_PI);
      forward_result_.rr_wheel_speed = -forward_result_.rr_wheel_speed;
    }

    return forward_result_;
  }

 private:
  struct {
    f32 lf_steer_position, rf_steer_position, lr_steer_position, rr_steer_position;
    f32 lf_wheel_speed, rf_wheel_speed, lr_wheel_speed, rr_wheel_speed;
  } forward_result_{};
  f32 chassis_radius_;
};

/**
 * @brief 等腰三角形排列的三舵轮底盘（前、左后、右后）
 */
class TriSteeringChassis {
 public:
  TriSteeringChassis() = delete;
  ~TriSteeringChassis() = default;

  /**
   * @brief 构造函数
   * @param base_len 两后轮的间距（等腰三角形底边长）
   * @param height   前轮到两后轮连线的垂直距离（等腰三角形高）
   */
  TriSteeringChassis(f32 base_len, f32 height) {
    rear_x_ = base_len / 2.0f;
    chassis_radius_ = (base_len * base_len + 4.0f * height * height) / (8.0f * height);
    rear_y_ = chassis_radius_ - height;
  }

  /**
   * @brief 360度三舵轮正运动学
   * @param vx                左右方向速度
   * @param vy                前后方向速度
   * @param w                 旋转速度，从上向下看顺时针为正
   * @note
   * 三舵轮排列为等腰三角形，旋转中心为三角形外接圆圆心：
   * 这个函数不考虑当前舵角与目标角度是否大于90度而反转舵
   */
  auto Forward(f32 vx, f32 vy, f32 w) {
    using modules::IsNear;
    if (IsNear(vx, 0.f, 1e-6f) &&  //
        IsNear(vy, 0.f, 1e-6f) &&  //
        IsNear(w, 0.f, 1e-6f)) {
      // 三个舵摆成一个圆周阵型（各轮切线方向），这样可以随时开始小陀螺旋转
      forward_result_.front_steer_position = 0.f;                     // 前轮朝右
      forward_result_.lr_steer_position = atan2f(rear_x_, rear_y_);   // 左后轮朝左前
      forward_result_.rr_steer_position = atan2f(-rear_x_, rear_y_);  // 右后轮朝左后
    } else {
      // 前轮位于 (0, chassis_radius_)
      const f32 front_vx = vx - w * chassis_radius_;
      const f32 front_vy = vy;
      forward_result_.front_steer_position = atan2f(front_vy, front_vx);

      // 左后轮位于 (-rear_x_, rear_y_)
      const f32 lr_vx = vx - w * rear_y_;
      const f32 lr_vy = vy - w * rear_x_;
      forward_result_.lr_steer_position = atan2f(lr_vy, lr_vx);

      // 右后轮位于 (rear_x_, rear_y_)
      const f32 rr_vx = vx - w * rear_y_;
      const f32 rr_vy = vy + w * rear_x_;
      forward_result_.rr_steer_position = atan2f(rr_vy, rr_vx);
    }

    // 计算轮速（速度向量的模）
    const f32 front_vx = vx - w * chassis_radius_;
    const f32 front_vy = vy;
    forward_result_.front_wheel_speed = sqrtf(front_vx * front_vx + front_vy * front_vy);

    const f32 lr_vx = vx - w * rear_y_;
    const f32 lr_vy = vy - w * rear_x_;
    forward_result_.lr_wheel_speed = sqrtf(lr_vx * lr_vx + lr_vy * lr_vy);

    const f32 rr_vx = vx - w * rear_y_;
    const f32 rr_vy = vy + w * rear_x_;
    forward_result_.rr_wheel_speed = sqrtf(rr_vx * rr_vx + rr_vy * rr_vy);

    return forward_result_;
  }

  /**
   * @brief 180度三舵轮正运动学
   * @param vx                    左右方向速度
   * @param vy                    前后方向速度
   * @param w                     旋转速度，从上向下看顺时针为正
   * @param current_front_angle   当前的前舵角度，弧度制，底盘前进方向为0
   * @param current_lr_angle      当前的左后舵角度，弧度制，底盘前进方向为0
   * @param current_rr_angle      当前的右后舵角度，弧度制，底盘前进方向为0
   */
  auto Forward(f32 vx, f32 vy, f32 w, f32 current_front_angle, f32 current_lr_angle, f32 current_rr_angle) {
    Forward(vx, vy, w);

    // 依次计算每个舵的目标角度和目前角度的差值，如果差值大于90度，就把目标舵角加180度，轮速取反。
    // 这样可以保证舵角变化量始终小于90度，加快舵的响应速度
    if (std::abs(Wrap(current_front_angle - forward_result_.front_steer_position, -M_PI, M_PI)) > M_PI / 2) {
      forward_result_.front_steer_position = Wrap(forward_result_.front_steer_position + M_PI, 0, 2 * M_PI);
      forward_result_.front_wheel_speed = -forward_result_.front_wheel_speed;
    }
    if (std::abs(Wrap(current_lr_angle - forward_result_.lr_steer_position, -M_PI, M_PI)) > M_PI / 2) {
      forward_result_.lr_steer_position = Wrap(forward_result_.lr_steer_position + M_PI, 0, 2 * M_PI);
      forward_result_.lr_wheel_speed = -forward_result_.lr_wheel_speed;
    }
    if (std::abs(Wrap(current_rr_angle - forward_result_.rr_steer_position, -M_PI, M_PI)) > M_PI / 2) {
      forward_result_.rr_steer_position = Wrap(forward_result_.rr_steer_position + M_PI, 0, 2 * M_PI);
      forward_result_.rr_wheel_speed = -forward_result_.rr_wheel_speed;
    }

    return forward_result_;
  }

  auto forward_result() const { return forward_result_; }

 private:
  struct {
    f32 front_steer_position, lr_steer_position, rr_steer_position;
    f32 front_wheel_speed, lr_wheel_speed, rr_wheel_speed;
  } forward_result_{};
  f32 chassis_radius_;  // 旋转中心到各轮子的距离（也是前轮的 y 坐标）
  f32 rear_x_;          // 后轮 x 坐标的绝对值
  f32 rear_y_;          // 后轮 y 坐标
};

/**
 * @brief 四全向轮底盘
 */
class QuadOmniChassis {
 public:
  QuadOmniChassis() = default;
  ~QuadOmniChassis() = default;

  /**
   * @param vx    x轴方向的速度
   * @param vy    y轴方向的速度
   * @param wz    z轴方向的角速度
   */
  auto Forward(f32 vx, f32 vy, f32 wz, bool normalize = false) {
    forward_result_.lf_speed = vx + vy + wz;
    forward_result_.rf_speed = vx - vy + wz;
    forward_result_.lr_speed = -vx + vy + wz;
    forward_result_.rr_speed = -vx - vy + wz;
    if (normalize) {
      const f32 max_speed = max_({std::abs(forward_result_.lf_speed), std::abs(forward_result_.rf_speed),
                                  std::abs(forward_result_.lr_speed), std::abs(forward_result_.rr_speed)});
      if (max_speed > 1) {
        forward_result_.lf_speed /= max_speed;
        forward_result_.rf_speed /= max_speed;
        forward_result_.lr_speed /= max_speed;
        forward_result_.rr_speed /= max_speed;
      }
    }

    return forward_result_;
  }

  /**
   * @param lf_speed    左前轮速度
   * @param rf_speed    右前轮速度
   * @param lr_speed    左后轮速度
   * @param rr_speed    右后轮速度
   */
  auto Inverse(f32 lf_speed, f32 rf_speed, f32 lr_speed, f32 rr_speed) {
    inverse_result_.vx = (lf_speed + rf_speed - lr_speed - rr_speed) / 4;
    inverse_result_.vy = (-lf_speed + rf_speed - lr_speed + rr_speed) / 4;
    inverse_result_.wz = (lf_speed + rf_speed + lr_speed + rr_speed) / 4;
    // normalize
    f32 max_speed = max_({std::abs(inverse_result_.vx), std::abs(inverse_result_.vy), std::abs(inverse_result_.wz)});
    if (max_speed > 1) {
      inverse_result_.vx /= max_speed;
      inverse_result_.vy /= max_speed;
      inverse_result_.wz /= max_speed;
    }

    return inverse_result_;
  }
  auto forward_result() const { return forward_result_; }
  auto inverse_result() const { return inverse_result_; }

  struct {
    f32 vx, vy, wz;
  } inverse_result_{};
  struct {
    f32 lf_speed, rf_speed, lr_speed, rr_speed;
  } forward_result_{};
};

}  // namespace rm::modules

#endif  // LIBRM_MODULES_CHASSIS_FKIK_HPP
