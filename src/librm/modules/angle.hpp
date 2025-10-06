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
 * @file  librm/modules/angle.hpp
 * @brief 角度值、弧度制的转换、计算和表示工具类
 */

#ifndef LIBRM_MODULES_ANGLE_HPP
#define LIBRM_MODULES_ANGLE_HPP

#include <cmath>

#include "librm/core/typedefs.hpp"

namespace rm::modules {

/**
 * @brief 角度值、弧度制的转换、计算和表示工具类
 */
class Angle {
 public:
  explicit constexpr Angle(f32 rad = 0.f) : value_rad_(rad) {}

  constexpr f32 deg() const { return value_rad_ * 180.f / M_PIf; }
  constexpr f32 rad() const { return value_rad_; }

  constexpr Angle operator+(const Angle &other) const { return Angle{value_rad_ + other.value_rad_}; }
  constexpr Angle operator-(const Angle &other) const { return Angle{value_rad_ - other.value_rad_}; }
  constexpr Angle operator*(f32 scalar) const { return Angle{value_rad_ * scalar}; }
  constexpr Angle operator/(f32 scalar) const { return Angle{value_rad_ / scalar}; }

  static Angle FromDeg(f32 deg) { return Angle{deg * M_PIf / 180.f}; }
  static Angle FromRad(f32 rad) { return Angle{rad}; }

 private:
  f32 value_rad_;  ///< 内部统一用弧度表示
};

namespace angle_literals {

inline constexpr Angle operator"" _deg(long double deg) { return Angle{static_cast<f32>(deg * M_PIf / 180.f)}; }

inline constexpr Angle operator"" _rad(long double rad) { return Angle{static_cast<f32>(rad)}; }

}  // namespace angle_literals

}  // namespace rm::modules

#endif  // LIBRM_MODULES_ANGLE_H