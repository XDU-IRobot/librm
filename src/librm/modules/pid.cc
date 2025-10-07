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

#include "librm/modules/utils.hpp"

namespace rm::modules {

PID::PID() = default;

PID::PID(f32 kp, f32 ki, f32 kd, f32 max_out, f32 max_iout)
    : kp_(kp), ki_(ki), kd_(kd), max_out_(max_out), max_iout_(max_iout) {}

PID::~PID() = default;

void PID::Update(f32 set, f32 ref, f32 dt) {
  set_ = set;
  ref_[1] = ref_[0];
  ref_[0] = ref;
  dt_ = dt;
  error_[1] = error_[0];
  if (enable_circular_) {
    error_[0] = Wrap(set - ref, -circular_cycle_ / 2, circular_cycle_ / 2);  // 处理过零
  } else {
    error_[0] = set - ref;
  }

  f32 calc_kp{kp_}, calc_ki{(enable_dynamic_ki_ ? dynamic_ki_ : calc_ki)}, calc_kd{kd_};
  if (enable_fuzzy_) {
    const f32 d_error = (error_[0] - error_[1]) / dt_;
    const f32 delta_kp = kp_fuzzy_.Infer(error_[0], d_error) * kp_ / 2.f;
    const f32 delta_ki = ki_fuzzy_.Infer(error_[0], d_error) * (enable_dynamic_ki_ ? dynamic_ki_ : calc_ki) / 2.f;
    const f32 delta_kd = kd_fuzzy_.Infer(error_[0], d_error) * kd_ / 2.f;
    calc_kp += delta_kp;
    calc_ki += delta_ki;
    calc_kd += delta_kd;
  }

  // p
  p_out_ = calc_kp * error_[0];

  // i
  trapezoid_ = (error_[0] + error_[1]) / 2 * dt_;     // 梯形积分
  dynamic_ki_ = calc_ki / (1 + std::abs(error_[0]));  // 变速积分
  i_out_ += calc_ki * trapezoid_;
  i_out_ = Clamp(i_out_, -max_iout_, max_iout_);

  // d
  d_out_[1] = d_out_[0];
  if (enable_diff_first_) {
    d_out_[0] = calc_kd * (ref_[0] - ref_[1]) / dt;  // 微分先行
  } else {
    d_out_[0] = calc_kd * (error_[0] - error_[1]) / dt;
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
  if (enable_circular_) {
    error_[0] = Wrap(set - ref, -circular_cycle_ / 2, circular_cycle_ / 2);  // 处理过零
  } else {
    error_[0] = set - ref;
  }

  f32 calc_kp{kp_}, calc_ki{(enable_dynamic_ki_ ? dynamic_ki_ : calc_ki)}, calc_kd{kd_};
  if (enable_fuzzy_) {
    const f32 d_error = (error_[0] - error_[1]) / dt_;
    const f32 delta_kp = kp_fuzzy_.Infer(error_[0], d_error) * kp_ / 2.f;
    const f32 delta_ki = ki_fuzzy_.Infer(error_[0], d_error) * (enable_dynamic_ki_ ? dynamic_ki_ : calc_ki) / 2.f;
    const f32 delta_kd = kd_fuzzy_.Infer(error_[0], d_error) * kd_ / 2.f;
    calc_kp += delta_kp;
    calc_ki += delta_ki;
    calc_kd += delta_kd;
  }

  // p
  p_out_ = calc_kp * error_[0];

  // i
  trapezoid_ = (error_[0] + error_[1]) / 2 * dt_;     // 梯形积分
  dynamic_ki_ = calc_ki / (1 + std::abs(error_[0]));  // 变速积分
  i_out_ += calc_ki * trapezoid_;
  i_out_ = Clamp(i_out_, -max_iout_, max_iout_);

  // d
  d_out_[1] = d_out_[0];
  d_out_[0] = calc_kd * external_diff;
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
PID &PID::SetCircular(bool enable) {
  enable_circular_ = enable;
  return *this;
}
PID &PID::SetCircularCycle(f32 cycle) {
  circular_cycle_ = cycle;
  return *this;
}
PID &PID::SetFuzzy(bool enable) {
  enable_fuzzy_ = enable;
  return *this;
}
PID &PID::SetFuzzyErrorScale(f32 value) {
  kp_fuzzy_.SetErrorScale(value);
  ki_fuzzy_.SetErrorScale(value);
  kd_fuzzy_.SetErrorScale(value);
  return *this;
}
PID &PID::SetFuzzyDErrorScale(f32 value) {
  kp_fuzzy_.SetDErrorScale(value);
  ki_fuzzy_.SetDErrorScale(value);
  kd_fuzzy_.SetDErrorScale(value);
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
bool PID::enable_circular() const { return enable_circular_; }
f32 PID::circular_cycle() const { return circular_cycle_; }
bool PID::enable_fuzzy() const { return enable_fuzzy_; }

PID::FuzzyInfer::FuzzyInfer(RuleTable rule_table) : rule_table_{rule_table} {}

f32 PID::FuzzyInfer::Infer(f32 error, f32 d_error) {
  // 输入归一化到[-3, 3]
  f32 e = error_scale_ == 0 ? 0 : error / error_scale_ * 3.f;
  f32 ec = d_error_scale_ == 0 ? 0 : d_error / d_error_scale_ * 3.f;

  // 限制输入范围
  e = std::fabs(e) > 3.0f ? (e > 0.0f ? 3.0f : -3.0f) : e;
  ec = std::fabs(ec) > 3.0f ? (ec > 0.0f ? 3.0f : -3.0f) : ec;

  // 计算隶属度
  std::array<f32, 7> e_degrees, ec_degrees;
  for (int i = 0; i < 7; ++i) {
    e_degrees[i] = CalcMembership(e, i);
    ec_degrees[i] = CalcMembership(ec, i);
  }

  // 规则推理
  f32 fuzzy_rule = 0.f, weight_sum = 0.f;
  for (int i = 0; i < 7; ++i) {
    for (int j = 0; j < 7; ++j) {
      // 计算规则激活强度
      const f32 w = e_degrees[i] * ec_degrees[j];
      if (w <= 0.0f) {
        continue;
      }
      // 更新输出隶属度
      fuzzy_rule += w * rule_table_[i][j];
      weight_sum += w;
    }
  }

  // 防止除零
  if (weight_sum == 0.0f) {
    weight_sum = 1e-6f;
  }
  // 计算模糊推理结果的加权平均值
  fuzzy_rule /= weight_sum;

  return fuzzy_rule / 3.f;
}

PID::FuzzyInfer &PID::FuzzyInfer::SetErrorScale(f32 value) {
  error_scale_ = value;
  return *this;
}

PID::FuzzyInfer &PID::FuzzyInfer::SetDErrorScale(f32 value) {
  d_error_scale_ = value;
  return *this;
}

const PID::FuzzyInfer::RuleTable &PID::FuzzyInfer::rule_table() const { return rule_table_; }
f32 PID::FuzzyInfer::error_scale() const { return error_scale_; }
f32 PID::FuzzyInfer::d_error_scale() const { return d_error_scale_; }

f32 PID::FuzzyInfer::CalcMembership(f32 x, int set) {
  const auto &params = kMembershipParams[set];

  // 三角形隶属度函数
  if (x <= params[0] || x >= params[2]) {
    return 0.0f;
  }
  if (x <= params[1]) {
    return (x - params[0]) / (params[1] - params[0]);
  }
  return (params[2] - x) / (params[2] - params[1]);
}

PID::FuzzyInfer &PID::kp_fuzzy() { return kp_fuzzy_; }
PID::FuzzyInfer &PID::ki_fuzzy() { return ki_fuzzy_; }
PID::FuzzyInfer &PID::kd_fuzzy() { return kd_fuzzy_; }

}  // namespace rm::modules
