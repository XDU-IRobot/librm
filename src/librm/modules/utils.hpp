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
 * @file  librm/modules/utils.hpp
 * @brief 通用工具函数
 */

#ifndef LIBRM_MODULES_UTILS_HPP
#define LIBRM_MODULES_UTILS_HPP

#include <cmath>

#include "librm/core/typedefs.hpp"

namespace rm::modules {

/**
 * @brief  符号函数
 * @tparam T 输入值类型
 * @param  value 输入值
 * @return 正数返回1，负数返回-1，0返回0
 */
template <typename T>
int sign(const T value) {
  if (value > 0) {
    return 1;
  } else if (value < 0) {
    return -1;
  } else {
    return 0;
  }
}

/**
 * @brief  deadband函数
 * @param  value 输入值
 * @param  min_value 下限
 * @param  max_value 上限
 * @return 若输入值在规定的范围内，就返回输入值，否则返回0
 */
f32 Deadline(f32 value, f32 min_value, f32 max_value);

/**
 * @brief  限幅函数
 * @param  input 输入值
 * @param  min_value 下限
 * @param  max_value 上限
 * @return 若输入值超出规定的范围，就返回最近的边界值，否则返回原值
 */
f32 Constrain(f32 input, f32 min_value, f32 max_value);

/**
 * @brief   把输入值限制在一个规定的周期内
 * @note    例如输入值为370，周期为360，则输出是10
 * @param   input       输入值
 * @param   min_value   周期开始
 * @param   max_value   周期结束
 * @return              限制到一个周期内的值
 */
f32 LoopConstrain(f32 input, f32 min_value, f32 max_value);

/**
 * @brief   限制输入值的绝对值
 * @tparam  T           输入值的类型
 * @param   input       输入值
 * @param   max_value   限制的最大绝对值
 * @return              限制后的值
 */
template <typename T>
T absConstrain(T input, T max_value) {
  if (input > max_value) {
    return max_value;
  } else if (input < -max_value) {
    return -max_value;
  } else {
    return input;
  }
}

/**
 * @brief 角度转弧度
 * @param deg   角度
 * @return      弧度
 */
f32 DegToRad(f32 deg);

/**
 * @brief 四元数转欧拉角
 * @param q         四元数
 * @param euler     欧拉角
 */
void QuatToEuler(const f32 q[4], f32 euler[3]);

/**
 * @brief 映射函数
 * @param value     输入值
 * @param from_min  输入值的最小值
 * @param from_max  输入值的最大值
 * @param to_min    输出值的最小值
 * @param to_max    输出值的最大值
 * @return          映射后的值
 */
f32 Map(f32 value, f32 from_min, f32 from_max, f32 to_min, f32 to_max);

/**
 * @brief 把给定位数的整数映射到浮点数的某个范围内
 * @param x_int 输入的整数
 * @param x_min 目标浮点数范围的下限
 * @param x_max 目标浮点数范围的上限
 * @param bits  输入的整数的位数
 * @return      映射后的浮点数
 */
f32 IntToFloat(int x_int, f32 x_min, f32 x_max, int bits);

/**
 * @brief 把某个范围内的浮点数映射到给定位数的整数的整个范围内
 * @param x      输入的浮点数
 * @param x_min  目标浮点数范围的下限
 * @param x_max  目标浮点数范围的上限
 * @param bits   输出的整数的位数
 * @return       映射后的整数
 */
int FloatToInt(f32 x, f32 x_min, f32 x_max, int bits);

}  // namespace rm::modules

#endif  // LIBRM_MODULES_UTILS_HPP
