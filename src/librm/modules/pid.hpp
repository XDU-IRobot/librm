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
 * @file  librm/modules/pid.hpp
 * @brief PID控制器
 */

#ifndef LIBRM_MODULES_PID_HPP
#define LIBRM_MODULES_PID_HPP

#include "librm/core/typedefs.hpp"

namespace rm::modules {

/**
 * @brief   位置式PID控制器
 */
class PID {
 public:
  PID();
  virtual ~PID();
  PID(f32 kp, f32 ki, f32 kd, f32 max_out, f32 max_iout);
  virtual void Update(f32 set, f32 ref, f32 dt = 1.f);
  virtual void UpdateExtDiff(f32 set, f32 ref, f32 external_diff, f32 dt = 1.f);
  void Clear();

 protected:
  f32 kp_{};
  f32 ki_{};
  f32 kd_{};
  f32 max_out_{};
  f32 max_iout_{};
  f32 set_{};
  f32 ref_[2]{};  ///< 0: 这次, 1: 上次
  f32 out_{};
  f32 p_out_{};
  f32 i_out_{};
  f32 d_out_[2]{};  ///< 0: 这次, 1: 上次
  f32 error_[2]{};  ///< 0: 这次, 1: 上次
  f32 dt_{};
  f32 trapezoid_{};  ///< 梯形积分计算结果
  f32 diff_lpf_alpha_{1.f};  ///< 微分项低通滤波系数，取值范围(0, 1]，越小滤波效果越强，设置为1时不滤波
  bool enable_diff_first_{false};  ///< 是否微分先行
  bool enable_dynamic_ki_{false};  ///< 是否变速积分
  f32 dynamic_ki_{};               ///< 变速积分计算的ki

 public:
  // setters
  PID &SetKp(f32 value);
  PID &SetKi(f32 value);
  PID &SetKd(f32 value);
  PID &SetMaxOut(f32 value);
  PID &SetMaxIout(f32 value);
  PID &SetDiffLpfAlpha(f32 value);
  PID &SetDiffFirst(bool enable);
  PID &SetDynamicKi(bool enable);

  // getters
  f32 kp() const;
  f32 ki() const;
  f32 kd() const;
  f32 max_out() const;
  f32 max_iout() const;
  f32 set() const;
  const f32 *ref() const;
  f32 out() const;
  f32 p_out() const;
  f32 i_out() const;
  const f32 *d_out() const;
  const f32 *error() const;
  f32 dt() const;
  f32 trapezoid() const;
  f32 diff_lpf_alpha() const;
  bool enable_diff_first() const;
  bool enable_dynamic_ki() const;
  f32 dynamic_ki() const;
};

/**
 * @brief 带过零点处理的PID控制器，用于角度闭环控制
 */
class RingPID : public PID {
 public:
  RingPID();
  ~RingPID() override;
  RingPID(f32 kp, f32 ki, f32 kd, f32 max_out, f32 max_iout, f32 cycle);
  void Update(f32 set, f32 ref, f32 dt = 1.f) override;
  void UpdateExtDiff(f32 set, f32 ref, f32 external_diff, f32 dt = 1.f) override;

 protected:
  f32 cycle_;
};

}  // namespace rm::modules

#endif  // LIBRM_MODULES_PID_HPP
