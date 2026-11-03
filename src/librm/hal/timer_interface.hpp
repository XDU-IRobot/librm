
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
 * @file    librm/hal/timer_interface.hpp
 * @brief   定时器的接口类
 */

#ifndef LIBRM_HAL_TIMER_INTERFACE_HPP
#define LIBRM_HAL_TIMER_INTERFACE_HPP

#include <chrono>

#include <etl/delegate.h>

#include "librm/core/typedefs.hpp"

namespace rm::hal {

/// @brief 定时器类型：循环或单次
enum class TimerType : u8 {
  kPeriodic,
  kOneShot,
};

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
   * @brief   启动定时器。
   * @return  是否启动成功
   * @note    启动定时器前，需要先调用SetTimeout方法设置超时，否则总是会启动失败，返回false。
   */
  virtual bool Start() = 0;

  /**
   * @brief   停止定时器。
   * @return  是否停止成功
   */
  virtual bool Stop() = 0;

  /**
   * @brief   查询定时器是否正在运行。
   * @return  若定时器正在运行返回true，否则返回false。
   */
  virtual bool running() const = 0;

  /**
   * @brief   设置定时器超时时间。
   * @return  是否设置成功
   * @param   timeout 定时器超时时间。
   * @note    此函数只能在定时器没有启动时调用。
   *          若在定时器运行时调用此函数，则设置将会失败，并且返回false。
   */
  virtual bool SetTimeout(std::chrono::duration<f32> timeout) = 0;

  /**
   * @brief   查询定时器的超时时间。
   * @return  当前超时时间；从未成功设置过超时时间时返回 0。
   * @note    返回值是定时器实际生效的周期，而不是 SetTimeout() 传入的原始值：
   *          后端可能会把超时时间量化到自己的时间刻度（例如 STM32 的 TIM 会量化到
   *          PSC/ARR 计数刻度，PollingTimer 会量化到整毫秒），此时返回量化后的结果。
   *          需要根据定时器周期推算频率时（如 base_frequency = 1 / GetTimeout()），
   *          应当使用本函数而不是缓存 SetTimeout() 的参数。
   */
  virtual std::chrono::duration<f32> GetTimeout() const = 0;

  /**
   * @brief   绑定定时器超时回调函数。
   * @return  是否绑定成功
   * @param   callback 超时后调用的回调函数。
   * @note    此函数只能在定时器没有启动时调用。
   *          若在定时器运行时调用此函数，则设置将会失败，并且返回false。
   */
  virtual bool AttachCallback(etl::delegate<void()> callback) = 0;

  /**
   * @brief   设置定时器类型。
   * @return  是否设置成功
   * @param   type 定时器类型，循环或单次。
   * @note    定时器对象创建之后默认处于循环模式。要使用单次模式，则需要在启动定时器之前调用此函数设置。
   *          此函数只能在定时器没有启动时调用。
   *          若在定时器运行时调用此函数，则设置将会失败，并且返回false。
   */
  virtual bool SetTimerType(TimerType type) = 0;
};

}  // namespace rm::hal

#endif  // LIBRM_HAL_TIMER_INTERFACE_HPP
