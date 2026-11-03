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
 * @file    librm/hal/stm32/timer.hpp
 * @brief   基于STM32 HAL库TIM外设的TimerInterface实现
 */

#ifndef LIBRM_HAL_STM32_TIMER_HPP
#define LIBRM_HAL_STM32_TIMER_HPP

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_TIM_MODULE_ENABLED)

#include <chrono>

#include <etl/delegate.h>

#include "librm/core/traits.hpp"
#include "librm/core/typedefs.hpp"
#include "librm/hal/timer_interface.hpp"

namespace rm::hal::stm32 {

/**
 * @brief 基于TIM更新中断实现的定时器。
 *
 * @note  由于SetTimeout()方法会根据传入的超时时间计算并设置PSC和ARR值，
 *        故实际使用时，为了避免隐蔽的相互干涉，一个TIM只能交给一个Timer对象独占使用，不要再把同一个TIM用于其他用途。
 *
 * @note  使用前需要满足以下条件：
 *        1. CubeMX 中启用了该TIM的 Register Callback 功能（宏 USE_HAL_TIM_REGISTER_CALLBACKS）；
 *        2. 调用过 MX_TIMx_Init()，使传入的 TIM_HandleTypeDef 处于 HAL_TIM_STATE_READY。
 *
 * @note  超时精度取决于该TIM的时钟频率，短超时下可达到约1个计数周期的精度（亚微秒级）。
 *        超时时间会被钳位在 1 微秒 到 65536 * 65536 / TIM时钟频率 秒之间。
 */
class Timer : public TimerInterface, rm::detail::NonCopyable {
 public:
  /**
   * @param htim 已由 CubeMX 初始化过的 TIM 句柄。
   */
  explicit Timer(TIM_HandleTypeDef &htim);

  ~Timer() override;

  /**
   * @brief 启动定时器。
   * @return 是否启动成功，未设置过超时时间时返回false。
   *
   * @note 定时器默认工作在循环模式，每次更新中断都会调用回调；若设置为单次模式，则触发一次后自动停止。
   */
  bool Start() override;

  /**
   * @brief 停止定时器。
   * @return 是否停止成功，定时器本来就没有在运行时返回false。
   */
  bool Stop() override;

  /**
   * @brief 查询定时器是否正在运行。
   * @return 若定时器正在运行返回true，否则返回false。
   */
  bool running() const override { return running_; }

  /**
   * @brief   设置定时器超时时间。
   * @return  是否设置成功，定时器正在运行、或超时时间不为正数时返回false。
   * @param   timeout 定时器超时时间，内部按微秒换算为计数值。
   */
  bool SetTimeout(std::chrono::duration<f32> timeout) override;

  /**
   * @brief   查询定时器的超时时间。
   * @return  当前超时时间；从未成功设置过超时时间时返回 0。
   * @note    返回的是由 PSC/ARR 决定的实际周期，可能因量化而与 SetTimeout() 传入的值
   *          略有差异（误差不超过一个计数周期），据此可以精确还原定时器的实际中断频率。
   */
  std::chrono::duration<f32> GetTimeout() const override;

  /**
   * @brief 绑定定时器超时回调函数。
   * @param callback 超时后调用的回调函数。
   * @return 是否绑定成功，定时器正在运行时绑定失败并返回false。
   *
   * @warning 回调在中断上下文中执行，不要在回调里做耗时操作。
   */
  bool AttachCallback(etl::delegate<void()> callback) override {
    if (running_) {
      return false;
    }
    callback_ = callback;
    return true;
  }

  /**
   * @brief   设置定时器类型。
   * @return  是否设置成功，定时器正在运行时设置失败并返回false。
   * @param   type 定时器类型，循环或单次。
   * @note    定时器对象创建之后默认处于循环模式。
   */
  bool SetTimerType(TimerType type) override {
    if (running_) {
      return false;
    }
    type_ = type;
    return true;
  }

 private:
  /**
   * @brief 注册给HAL库的更新中断回调入口，通过句柄找到对应的Timer对象。
   */
  static void PeriodElapsedThunk(TIM_HandleTypeDef *htim);

  /**
   * @brief 实际的更新中断处理。
   */
  void PeriodElapsedCallback();

  TIM_HandleTypeDef *htim_;         ///< HAL库的TIM句柄
  bool running_;                    ///< 定时器运行状态
  etl::delegate<void()> callback_;  ///< 超时回调
  TimerType type_;                  ///< 定时器类型（循环/单次）
  u32 prescaler_;                   ///< SetTimeout()算出的预分频值
  u32 autoreload_;                  ///< SetTimeout()算出的自动重装载值
  bool timeout_set_;                ///< 是否已经设置过超时时间
};

}  // namespace rm::hal::stm32

#endif  // HAL_TIM_MODULE_ENABLED

#endif  // LIBRM_HAL_STM32_TIMER_HPP
