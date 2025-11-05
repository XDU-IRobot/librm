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
 * @file  librm/modules/sparse_value_watcher.hpp
 * @brief 离散值监视器
 */

#ifndef LIBRM_MODULES_SPARSE_VALUE_WATCHER_HPP
#define LIBRM_MODULES_SPARSE_VALUE_WATCHER_HPP

#include <etl/delegate.h>

namespace rm::modules {
/**
 * @brief 监视一个离散值的变化，当值发生变化时触发回调函数
 * @tparam T 要监视的值的类型
 */
template <typename T>
class SparseValueWatcher {
 public:
  /**
   * @brief 回调函数类型
   * @param old_value 变化前的值
   * @param new_value 变化后的值
   */
  using Callback = etl::delegate<void(const T &old_value, const T &new_value)>;

  SparseValueWatcher() = default;

  explicit SparseValueWatcher(T initial_value) : value_(std::move(initial_value)) {}

  /**
   * @brief 构造函数
   * @param initial_value 初始值
   * @param callback 回调函数
   */
  SparseValueWatcher(T initial_value, Callback callback) : value_(std::move(initial_value)), callback_(callback) {}

  /**
   * @brief 设置值变化时的回调函数
   * @param callback
   */
  void OnValueChange(Callback callback) { callback_ = callback; }

  /**
   * @brief 更新值并检查是否发生变化
   * @param new_value 新的值
   */
  void Update(const T &new_value) {
    if (new_value != value_) {
      T old_value = value_;
      value_ = new_value;
      if (callback_) {
        callback_(old_value, value_);
      }
    }
  }

  /**
   * @brief 获取当前值
   * @return 当前值
   */
  const T &value() const { return value_; }

 private:
  T value_;
  Callback callback_;
};

}  // namespace rm::modules

#endif