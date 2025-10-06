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
 * @file  librm/modules/pid.cc
 * @brief PID控制器
 */

#include "pid.hpp"

#include <cstring>
#include <cmath>
#include <limits>

#include "librm/modules/utils.hpp"

namespace rm::modules {

PID::PID(f32 kp, f32 ki, f32 kd, f32 max_out, f32 max_iout)
    : kp_(kp), ki_(ki), kd_(kd), max_out_(max_out), max_iout_(max_iout) {}

PID::~PID() = default;

void PID::Update(f32 set, f32 ref, f32 dt) {
  set_ = set;
  ref_[1] = ref_[0];
  ref_[0] = ref;
  dt_ = dt;
  error_[1] = error_[0];
  error_[0] = set - ref;

  // p
  p_out_ = kp_ * error_[0];

  // i
  trapezoid_ = (error_[0] + error_[1]) / 2 * dt_;  // 梯形积分
  dynamic_ki_ = ki_ / (1 + std::abs(error_[0]));   // 变速积分
  i_out_ += (enable_dynamic_ki_ ? dynamic_ki_ : ki_) * trapezoid_;
  i_out_ = Clamp(i_out_, -max_iout_, max_iout_);

  // d
  d_out_[1] = d_out_[0];
  if (enable_diff_first_) {
    d_out_[0] = kd_ * (ref_[0] - ref_[1]) / dt;  // 微分先行
  } else {
    d_out_[0] = kd_ * (error_[0] - error_[1]) / dt;
  }
  d_out_[0] = diff_lpf_alpha_ * d_out_[0] + (1 - diff_lpf_alpha_) * d_out_[1];  // 微分项滤波

  out_ = Clamp(p_out_ + i_out_ + d_out_[0], -max_out_, max_out_);
}

void PID::UpdateExtDiff(f32 set, f32 ref, f32 external_diff, f32 dt) {
  set_ = set;
  ref_[1] = ref_[0];
  ref_[0] = ref;
  dt_ = dt;
  error_[1] = error_[0];
  error_[0] = set - ref;

  // p
  p_out_ = kp_ * error_[0];

  // i
  trapezoid_ = (error_[0] + error_[1]) / 2 * dt_;  // 梯形积分
  dynamic_ki_ = ki_ / (1 + std::abs(error_[0]));   // 变速积分
  i_out_ += (enable_dynamic_ki_ ? dynamic_ki_ : ki_) * trapezoid_;
  i_out_ = Clamp(i_out_, -max_iout_, max_iout_);

  // d
  d_out_[1] = d_out_[0];
  d_out_[0] = kd_ * external_diff;
  d_out_[0] = diff_lpf_alpha_ * d_out_[0] + (1 - diff_lpf_alpha_) * d_out_[1];  // 微分项滤波

  out_ = Clamp(p_out_ + i_out_ + d_out_[0], -max_out_, max_out_);
}

void PID::Clear() {
  set_ = 0;
  out_ = 0;
  p_out_ = 0;
  i_out_ = 0;
  std::memset(d_out_, 0, sizeof(d_out_));
  std::memset(ref_, 0, sizeof(ref_));
  std::memset(error_, 0, sizeof(error_));
}

PID &PID::SetKp(f32 value) {
  kp_ = value;
  return *this;
}
PID &PID::SetKi(f32 value) {
  ki_ = value;
  return *this;
}
PID &PID::SetKd(f32 value) {
  kd_ = value;
  return *this;
}
PID &PID::SetMaxOut(f32 value) {
  max_out_ = value;
  return *this;
}
PID &PID::SetMaxIout(f32 value) {
  max_iout_ = value;
  return *this;
}
PID &PID::SetDiffLpfAlpha(f32 value) {
  diff_lpf_alpha_ = value;
  return *this;
}
PID &PID::SetDiffFirst(bool enable) {
  enable_diff_first_ = enable;
  return *this;
}
PID &PID::SetDynamicKi(bool enable) {
  enable_dynamic_ki_ = enable;
  return *this;
}

f32 PID::kp() const { return kp_; }
f32 PID::ki() const { return ki_; }
f32 PID::kd() const { return kd_; }
f32 PID::max_out() const { return max_out_; }
f32 PID::max_iout() const { return max_iout_; }
f32 PID::set() const { return set_; }
const f32 *PID::ref() const { return ref_; }
f32 PID::out() const { return out_; }
f32 PID::p_out() const { return p_out_; }
f32 PID::i_out() const { return i_out_; }
const f32 *PID::d_out() const { return d_out_; }
const f32 *PID::error() const { return error_; }
f32 PID::dt() const { return dt_; }
f32 PID::trapezoid() const { return trapezoid_; }
f32 PID::diff_lpf_alpha() const { return diff_lpf_alpha_; }
bool PID::enable_diff_first() const { return enable_diff_first_; }
bool PID::enable_dynamic_ki() const { return enable_dynamic_ki_; }
f32 PID::dynamic_ki() const { return dynamic_ki_; }

RingPID::RingPID() : PID(), cycle_(std::numeric_limits<f32>::max()) {}
RingPID::~RingPID() = default;
RingPID::RingPID(f32 kp, f32 ki, f32 kd, f32 max_out, f32 max_iout, f32 cycle)
    : PID(kp, ki, kd, max_out, max_iout), cycle_(cycle) {}

void RingPID::Update(f32 set, f32 ref, f32 dt) {
  set_ = set;
  ref_[1] = ref_[0];
  ref_[0] = ref;
  dt_ = dt;
  error_[1] = error_[0];
  error_[0] = Wrap(set - ref, -cycle_ / 2, cycle_ / 2);  // 处理过零

  // p
  p_out_ = kp_ * error_[0];

  // i
  trapezoid_ = (error_[0] + error_[1]) / 2 * dt_;  // 梯形积分
  dynamic_ki_ = ki_ / (1 + std::abs(error_[0]));   // 变速积分
  i_out_ += (enable_dynamic_ki_ ? dynamic_ki_ : ki_) * trapezoid_;
  i_out_ = Clamp(i_out_, -max_iout_, max_iout_);

  // d
  d_out_[1] = d_out_[0];
  if (enable_diff_first_) {
    d_out_[0] = kd_ * (ref_[0] - ref_[1]) / dt;
  } else {
    d_out_[0] = kd_ * (error_[0] - error_[1]) / dt;
  }
  d_out_[0] = diff_lpf_alpha_ * d_out_[0] + (1 - diff_lpf_alpha_) * d_out_[1];  // 微分项滤波

  out_ = Clamp(p_out_ + i_out_ + d_out_[0], -max_out_, max_out_);
}

void RingPID::UpdateExtDiff(f32 set, f32 ref, f32 external_diff, f32 dt) {
  set_ = set;
  ref_[1] = ref_[0];
  ref_[0] = ref;
  dt_ = dt;
  error_[1] = error_[0];
  error_[0] = Wrap(set - ref, -cycle_ / 2, cycle_ / 2);  // 处理过零

  // p
  p_out_ = kp_ * error_[0];

  // i
  trapezoid_ = (error_[0] + error_[1]) / 2 * dt_;  // 梯形积分
  dynamic_ki_ = ki_ / (1 + std::abs(error_[0]));   // 变速积分
  i_out_ += (enable_dynamic_ki_ ? dynamic_ki_ : ki_) * trapezoid_;
  i_out_ = Clamp(i_out_, -max_iout_, max_iout_);

  // d
  d_out_[1] = d_out_[0];
  d_out_[0] = kd_ * external_diff;
  d_out_[0] = diff_lpf_alpha_ * d_out_[0] + (1 - diff_lpf_alpha_) * d_out_[1];  // 微分项滤波

  out_ = Clamp(p_out_ + i_out_ + d_out_[0], -max_out_, max_out_);
}

}  // namespace rm::modules
