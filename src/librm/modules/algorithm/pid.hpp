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
 * @file  librm/modules/algorithm/pid.hpp
 * @brief PID控制器
 */

#ifndef LIBRM_MODULES_ALGORITHM_PID_HPP
#define LIBRM_MODULES_ALGORITHM_PID_HPP

#include <cstring>
#include <limits>

#include "librm/core/typedefs.h"
#include "librm/modules/algorithm/utils.hpp"

namespace rm::modules::algorithm {

/**
 * @brief PID控制器类型
 */
enum class PIDType {
  kPosition,  ///< 位置式PID
  kDelta,     ///< 增量式PID
};

/**
 * @brief   PID控制器
 * @tparam  type PID控制器类型
 */
template <PIDType type>
class PID {
 public:
  PID() = default;
  virtual ~PID() = default;

  PID(f32 kp, f32 ki, f32 kd, f32 max_out, f32 max_iout)
      : kp_(kp), ki_(ki), kd_(kd), max_out_(max_out), max_iout_(max_iout) {}
  virtual void Update(f32 error);
  virtual void UpdateExtDiff(f32 error, f32 external_diff);
  void Update(f32 set, f32 ref) { Update(set - ref); }
  void UpdateExtDiff(f32 set, f32 ref, f32 external_diff) { Update(set - ref, external_diff); }
  void Clear() {
    set_ = 0;
    fdb_ = 0;
    out_ = 0;
    p_out_ = 0;
    i_out_ = 0;
    d_out_ = 0;
    std::memset(d_buf_, 0, sizeof(d_buf_));
    std::memset(error_, 0, sizeof(error_));
  }
  [[nodiscard]] f32 value() const { return utils::absConstrain(p_out_ + i_out_ + d_out_, max_out_); }
  [[nodiscard]] f32 last_error() const { return error_[0]; }
  [[nodiscard]] f32 p_out() const { return p_out_; }
  [[nodiscard]] f32 i_out() const { return i_out_; }
  [[nodiscard]] f32 d_out() const { return d_out_; }

 public:
  f32 kp_{};
  f32 ki_{};
  f32 kd_{};

  f32 max_out_{};
  f32 max_iout_{};

 protected:
  f32 set_{};
  f32 fdb_{};

  f32 out_{};
  f32 p_out_{};
  f32 i_out_{};
  f32 d_out_{};
  f32 d_buf_[3]{};  // 0: 这次, 1: 上次, 2: 上上次
  f32 error_[3]{};  // 0: 这次, 1: 上次, 2: 上上次
};

template <>
inline void PID<PIDType::kPosition>::Update(f32 error) {
  error_[2] = error_[1];
  error_[1] = error_[0];
  error_[0] = error;

  p_out_ = kp_ * error_[0];
  i_out_ += ki_ * error_[0];

  // update derivative term
  d_buf_[2] = d_buf_[1];
  d_buf_[1] = d_buf_[0];
  d_buf_[0] = error_[0] - error_[1];

  d_out_ = kd_ * d_buf_[0];
  i_out_ = utils::absConstrain(i_out_, max_iout_);
}

template <>
inline void PID<PIDType::kDelta>::Update(f32 error) {
  error_[2] = error_[1];
  error_[1] = error_[0];
  error_[0] = error;

  p_out_ = kp_ * (error_[0] - error_[1]);
  i_out_ = ki_ * error_[0];

  d_buf_[2] = d_buf_[1];
  d_buf_[1] = d_buf_[0];
  d_buf_[0] = error_[0] - 2.0f * error_[1] + error_[2];

  d_out_ = kd_ * d_buf_[0];
}

template <>
inline void PID<PIDType::kPosition>::UpdateExtDiff(f32 error, f32 external_diff) {
  error_[2] = error_[1];
  error_[1] = error_[0];
  error_[0] = error;

  p_out_ = kp_ * error_[0];
  i_out_ += ki_ * error_[0];

  // update derivative term
  d_buf_[2] = d_buf_[1];
  d_buf_[1] = d_buf_[0];
  d_buf_[0] = external_diff;

  d_out_ = kd_ * d_buf_[0];
  i_out_ = utils::absConstrain(i_out_, max_iout_);
}

template <>
inline void PID<PIDType::kDelta>::UpdateExtDiff(f32 error, f32 external_diff) {
  error_[2] = error_[1];
  error_[1] = error_[0];
  error_[0] = error;

  p_out_ = kp_ * (error_[0] - error_[1]);
  i_out_ = ki_ * error_[0];

  d_buf_[2] = d_buf_[1];
  d_buf_[1] = d_buf_[0];
  d_buf_[0] = external_diff;

  d_out_ = kd_ * d_buf_[0];
}

/**
 * @brief 带过零点处理的PID控制器，可以用于电机位置闭环控制等情况
 */
template <PIDType type>
class RingPID : public PID<type> {
 public:
  using PID<type>::Update;
  using PID<type>::UpdateExtDiff;

  RingPID() : cycle_(std::numeric_limits<f32>::max()) {}
  virtual ~RingPID() override = default;

  RingPID(f32 kp, f32 ki, f32 kd, f32 max_out, f32 max_iout, f32 cycle)
      : PID<type>(kp, ki, kd, max_out, max_iout), cycle_(cycle) {}
  void Update(f32 error) override;
  void UpdateExtDiff(f32 error, f32 external_diff) override;

 protected:
  f32 cycle_;
};

template <>
inline void RingPID<PIDType::kPosition>::Update(f32 error) {
  error_[2] = error_[1];
  error_[1] = error_[0];
  error_[0] = error;

  error_[0] = utils::LoopConstrain(error_[0], -cycle_ / 2, cycle_ / 2);

  p_out_ = kp_ * error_[0];
  i_out_ += ki_ * error_[0];

  d_buf_[2] = d_buf_[1];
  d_buf_[1] = d_buf_[0];
  d_buf_[0] = error_[0] - error_[1];

  d_out_ = kd_ * d_buf_[0];
  i_out_ = utils::absConstrain(i_out_, max_iout_);
}

template <>
inline void RingPID<PIDType::kDelta>::Update(f32 error) {
  error_[2] = error_[1];
  error_[1] = error_[0];
  error_[0] = error;

  error_[0] = utils::LoopConstrain(error_[0], -cycle_ / 2, cycle_ / 2);

  p_out_ = kp_ * (error_[0] - error_[1]);
  i_out_ = ki_ * error_[0];

  d_buf_[2] = d_buf_[1];
  d_buf_[1] = d_buf_[0];
  d_buf_[0] = error_[0] - 2.0f * error_[1] + error_[2];

  d_out_ = kd_ * d_buf_[0];
}

template <>
inline void RingPID<PIDType::kPosition>::UpdateExtDiff(f32 error, f32 external_diff) {
  error_[2] = error_[1];
  error_[1] = error_[0];
  error_[0] = error;

  error_[0] = utils::LoopConstrain(error_[0], -cycle_ / 2, cycle_ / 2);

  p_out_ = kp_ * error_[0];
  i_out_ += ki_ * error_[0];

  // update derivative term
  d_buf_[2] = d_buf_[1];
  d_buf_[1] = d_buf_[0];
  d_buf_[0] = external_diff;

  d_out_ = kd_ * d_buf_[0];
  i_out_ = utils::absConstrain(i_out_, max_iout_);
}

template <>
inline void RingPID<PIDType::kDelta>::UpdateExtDiff(f32 error, f32 external_diff) {
  error_[2] = error_[1];
  error_[1] = error_[0];
  error_[0] = error;

  error_[0] = utils::LoopConstrain(error_[0], -cycle_ / 2, cycle_ / 2);

  p_out_ = kp_ * (error_[0] - error_[1]);
  i_out_ = ki_ * error_[0];

  d_buf_[2] = d_buf_[1];
  d_buf_[1] = d_buf_[0];
  d_buf_[0] = external_diff;

  d_out_ = kd_ * d_buf_[0];
}

}  // namespace rm::modules::algorithm

#endif  // LIBRM_MODULES_ALGORITHM_PID_HPP
