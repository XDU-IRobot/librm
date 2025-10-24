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
 * @file  librm/core/exception.hpp
 * @brief 异常处理
 */

#ifndef LIBRM_CORE_EXCEPTIONS_HPP
#define LIBRM_CORE_EXCEPTIONS_HPP

#include <stdexcept>

#if defined(LIBRM_PLATFORM_STM32)
#include "librm/hal/stm32/exception.hpp"
#endif

namespace rm {

/**
 * @brief 抛出异常
 * @param e 异常类型
 */
inline void Throw(__attribute__((__unused__)) const std::exception& e) {
#if defined(LIBRM_PLATFORM_LINUX)
  throw e;
#elif defined(LIBRM_PLATFORM_STM32)
  /**
   * @note 如果进到了这里，调试查看函数调用栈，找到抛出异常的地方，然后把异常处理掉。
   */
  while (true) {
    // stm32编译时一般会带-fno-exceptions参数，不能抛异常。所以不管是什么异常，都不处理，直接死机。
  }
#endif
}

}  // namespace rm

#endif
