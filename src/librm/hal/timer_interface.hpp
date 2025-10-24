
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
 * @file    librm/hal/timer_interface.hpp
 * @brief   定时器的接口类
 */

#ifndef LIBRM_HAL_TIMER_INTERFACE_HPP
#define LIBRM_HAL_TIMER_INTERFACE_HPP

#include <chrono>

#include <etl/delegate.h>

#include "librm/core/typedefs.hpp"

namespace rm::hal {

/**
 * @brief 定时器接口类，定义了定时器的基本操作。
 *
 * 该接口用于抽象定时器的启动、停止、运行状态查询及回调绑定。
 */
class TimerInterface {
 public:
  /**
   * @brief 虚析构函数，保证派生类正确析构。
   */
  virtual ~TimerInterface() = default;

  /**
   * @brief 启动定时器。
   * @param timeout 定时器超时时间。
   */
  virtual void Start(std::chrono::duration<f32> timeout) = 0;

  /**
   * @brief 停止定时器。
   */
  virtual void Stop() = 0;

  /**
   * @brief 查询定时器是否正在运行。
   * @return 若定时器正在运行返回true，否则返回false。
   */
  virtual bool running() const = 0;

  /**
   * @brief 绑定定时器超时回调函数。
   * @param callback 超时后调用的回调函数。
   */
  virtual void AttachCallback(etl::delegate<void()> callback) = 0;
};

}  // namespace rm::hal

#endif  // LIBRM_HAL_TIMER_INTERFACE_HPP
