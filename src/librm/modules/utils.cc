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
 * @file  librm/modules/utils.cc
 * @brief 通用工具函数
 */

#include "utils.hpp"

#include <cmath>

#if defined(LIBRM_PLATFORM_STM32)
#include "librm/hal/stm32/hal.hpp"

#if !defined(__FPU_PRESENT)
#define __FPU_PRESENT 1  // fallback to use sqrtf if we can't determine if FPU is present
#endif

#endif

namespace rm::modules {

f32 InvSqrt(f32 x) {
#if defined(LIBRM_PLATFORM_STM32) && __FPU_PRESENT == 0
  // Fast inverse square-root
  // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
  // 对于没有FPU的STM32芯片，使用快速倒数平方根算法来计算1/sqrt(x)，这样可以提高性能
  f32 halfx = 0.5f * x;
  f32 y = x;
  long i = *reinterpret_cast<long *>(&y);
  i = 0x5f3759df - (i >> 1);
  y = *reinterpret_cast<f32 *>(&i);
  y = y * (1.5f - (halfx * y * y));
  return y;
#else
  return 1.f / sqrtf(x);
#endif
}

f32 Deadline(f32 value, f32 min_value, f32 max_value) {
  if (value < min_value || value > max_value) {
    return 0;
  } else {
    return value;
  }
}

f32 Wrap(f32 input, f32 min_value, f32 max_value) {
  f32 cycle = max_value - min_value;
  if (cycle <= 0) return input;
  return fmodf(fmodf(input - min_value, cycle) + cycle, cycle) + min_value;
}

void QuatToEuler(const f32 q[4], f32 euler[3]) {
  euler[0] = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
  euler[1] = asinf(2 * (q[0] * q[2] - q[3] * q[1]));
  euler[2] = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}

f32 Map(f32 value, f32 from_min, f32 from_max, f32 to_min, f32 to_max) {
  return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}

f32 IntToFloat(int x_int, f32 x_min, f32 x_max, int bits) {
  f32 span = x_max - x_min;
  f32 offset = x_min;
  return ((f32)x_int) * span / ((f32)((1 << bits) - 1)) + offset;
}

int FloatToInt(f32 x, f32 x_min, f32 x_max, int bits) {
  f32 span = x_max - x_min;
  f32 offset = x_min;
  return (int)((x - offset) * ((f32)((1 << bits) - 1)) / span);
}

f32 SafeDiv(f32 dividend, f32 divisor) {
  if (divisor == 0.f) {
    divisor = 1e-6f;
  }
  return dividend / divisor;
}

bool IsNear(f32 value, f32 target, f32 threshold) { return std::fabs(value - target) <= threshold; }

}  // namespace rm::modules