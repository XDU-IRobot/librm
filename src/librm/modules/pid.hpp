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

#include <array>
#include <limits>

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
  void Update(f32 set, f32 ref, f32 dt = 1.f);
  void UpdateExtDiff(f32 set, f32 ref, f32 external_diff, f32 dt = 1.f);
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
  f32 d_out_[2]{};   ///< 0: 这次, 1: 上次
  f32 error_[2]{};   ///< 0: 这次, 1: 上次
  f32 dt_{};         ///< 上次调用Update时输入的dt
  f32 trapezoid_{};  ///< 梯形积分计算结果
  f32 diff_lpf_alpha_{1.f};  ///< 微分项低通滤波系数，取值范围(0, 1]，越小滤波效果越强，设置为1时不滤波
  bool enable_diff_first_{false};                        ///< 是否微分先行
  bool enable_dynamic_ki_{false};                        ///< 是否变速积分
  f32 dynamic_ki_{};                                     ///< 变速积分计算的ki
  bool enable_circular_{false};                          ///< 是否启用过零点处理
  f32 circular_cycle_{std::numeric_limits<f32>::max()};  ///< 过零点处理的周期
  bool enable_fuzzy_{false};                             ///< 是否启用模糊PID

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
  PID &SetCircular(bool enable);
  PID &SetCircularCycle(f32 cycle);
  PID &SetFuzzy(bool enable);
  PID &SetFuzzyErrorScale(f32 value);
  PID &SetFuzzyDErrorScale(f32 value);

  // const getters
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
  bool enable_circular() const;
  f32 circular_cycle() const;
  bool enable_fuzzy() const;

 protected:
  /**
   * @brief 模糊推理引擎
   * @brief source: https://github.com/TongjiSuperPower/sp_middleware/tree/main/tools/fuzzy_pid
   */
  class FuzzyInfer {
   public:
    using RuleTable = std::array<std::array<int, 7>, 7>;

   public:
    FuzzyInfer() = delete;
    explicit FuzzyInfer(RuleTable rule_table);
    f32 Infer(f32 error, f32 d_error);

    // setters
    FuzzyInfer &SetErrorScale(f32 value);
    FuzzyInfer &SetDErrorScale(f32 value);

    // getters
    const RuleTable &rule_table() const;
    f32 error_scale() const;
    f32 d_error_scale() const;

   protected:
    f32 CalcMembership(f32 x, int set);

    const RuleTable rule_table_;
    f32 error_scale_{};    ///< 误差尺度因子
    f32 d_error_scale_{};  ///< 误差变化率尺度因子

    static constexpr f32 kMembershipParams[7][3] = {
        {-3, -3, -2},  ///< NB
        {-2, -2, -1},  ///< NM
        {-1, -1, 0},   ///< NS
        {-1, 0, 1},    ///< ZO
        {0, 1, 1},     ///< PS
        {1, 2, 2},     ///< PM
        {2, 3, 3}      ///< PB
    };
  };

  /**
   * @brief 模糊论域分级常量值，对应语言变量 NB, NM, NS, ZO, PS, PM, PB
   */
  enum FuzzyDomain {
    NB = -3,
    NM,
    NS,
    ZO,
    PS,
    PM,
    PB,
  };
  FuzzyInfer  //
      kp_fuzzy_{{{
          // NB  NM  NS  ZO  PS  PM  PB   (de →)
          {{PB, PB, PM, PM, PS, ZO, ZO}},  // NB (e ↓)
          {{PB, PB, PM, PS, PS, ZO, NS}},  // NM
          {{PM, PM, PM, PS, ZO, NS, NS}},  // NS
          {{PM, PM, PS, ZO, NS, NM, NM}},  // ZO
          {{PS, PS, ZO, NS, NS, NM, NM}},  // PS
          {{PS, ZO, NS, NM, NM, NM, NB}},  // PM
          {{ZO, ZO, NM, NM, NM, NB, NB}}   // PB
      }}},
      ki_fuzzy_{{{
          // NB  NM  NS  ZO  PS  PM  PB   (de →)
          {{NB, NB, NM, NM, NS, ZO, ZO}},  // NB (e ↓)
          {{NB, NB, NM, NS, NS, ZO, ZO}},  // NM
          {{NB, NM, NS, NS, ZO, PS, PS}},  // NS
          {{NM, NM, NS, ZO, PS, PM, PM}},  // ZO
          {{NM, NS, ZO, PS, PS, PM, PB}},  // PS
          {{ZO, ZO, PS, PS, PM, PB, PB}},  // PM
          {{ZO, ZO, PS, PM, PM, PB, PB}}   // PB
      }}},
      kd_fuzzy_{{{
          // NB  NM  NS  ZO  PS  PM  PB   (de →)
          {{PS, NS, NB, NB, NB, NM, PS}},  // NB (e ↓)
          {{PS, NS, NB, NM, NM, NS, ZO}},  // NM
          {{ZO, NS, NM, NM, NS, NS, ZO}},  // NS
          {{ZO, NS, NS, NS, NS, NS, ZO}},  // ZO
          {{ZO, ZO, ZO, ZO, ZO, ZO, ZO}},  // PS
          {{PB, NS, PS, PS, PS, PS, PB}},  // PM
          {{PB, PM, PM, PM, PS, PS, PB}}   // PB
      }}};

  // FuzzyInfer object getters
 public:
  FuzzyInfer &kp_fuzzy();
  FuzzyInfer &ki_fuzzy();
  FuzzyInfer &kd_fuzzy();
};

}  // namespace rm::modules

#endif  // LIBRM_MODULES_PID_HPP
