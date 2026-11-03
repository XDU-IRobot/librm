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
 * @file  librm/core/exception.hpp
 * @brief 异常处理
 */

#ifndef LIBRM_CORE_EXCEPTIONS_HPP
#define LIBRM_CORE_EXCEPTIONS_HPP

#include <cstring>
#include <type_traits>

#include <etl/string.h>

#if defined(LIBRM_PLATFORM_STM32)
#include "librm/hal/stm32/exception.hpp"
#endif

namespace rm {

/// @brief rm::runtime_error 默认的消息缓冲区容量（字符数，不含结尾的 '\0'）
inline constexpr size_t kExceptionMessageCapacity = 128;

/**
 * @brief 无堆分配的运行时错误异常，用于替代 std::runtime_error
 *
 * std::runtime_error 内部用 std::string 保存消息，一旦超过小字符串优化阈值就会从堆上分配。
 * 本类型用 etl::string 的定长缓冲区代替，构造与析构都不会触碰堆，因此在中断上下文、
 * 初始化早期以及 -fno-exceptions 的 STM32 构建里都可以安全使用。
 *
 * 消息超过 Capacity 时会被静默截断，不会断言，也不会在错误处理路径上再生出新的错误。
 *
 * @tparam Capacity 消息缓冲区容量（字符数，不含结尾的 '\0'）
 */
template <size_t Capacity = kExceptionMessageCapacity>
class basic_runtime_error : public std::exception {
 public:
  explicit basic_runtime_error(const char *message) { SetMessage(message); }

  explicit basic_runtime_error(const etl::istring &message) { SetMessage(message.c_str()); }

  const char *what() const noexcept override { return message_.c_str(); }

 private:
  void SetMessage(const char *message) {
    if (message == nullptr) {
      message_.clear();
      return;
    }
    // 先自行限制长度，保证 assign 不会走到截断分支（截断检查可能被配置为断言）
    const size_t length = std::strlen(message);
    message_.assign(message, (length < Capacity) ? length : Capacity);
  }

  etl::string<Capacity> message_;
};

/// @brief 默认容量的无堆分配运行时错误，可直接替换 std::runtime_error
using runtime_error = basic_runtime_error<>;

/**
 * @brief 抛出异常
 * @tparam T 异常类型（必须派生自 std::exception）
 * @param e 异常对象
 */
template <typename T>
[[noreturn]] void Throw(__attribute__((__unused__)) const T &e) {
  static_assert(std::is_base_of<std::exception, T>::value, "Thrown type must be derived from std::exception");
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
