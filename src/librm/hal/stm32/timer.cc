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
 * @file    librm/hal/stm32/timer.cc
 * @brief   基于STM32 HAL库TIM外设的TimerInterface实现
 */

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_TIM_MODULE_ENABLED)

#include "timer.hpp"

#include <array>
#include <chrono>
#include <stdexcept>

#include "librm/hal/stm32/helper_macro.hpp"

namespace rm::hal::stm32 {

namespace {

constexpr usize kMaxTimerInstances = 8;
constexpr u32 kMaxAutoReload = 0xFFFFu;  ///< 按16位定时器处理，保证在任意型号上都够用

struct TimerEntry {
  TIM_HandleTypeDef *handle{nullptr};
  Timer *obj{nullptr};
};

/// HAL的回调只带一个句柄参数，所以这里用一个无堆的注册表把句柄映射回对象
std::array<TimerEntry, kMaxTimerInstances> g_timer_registry{};

void TimerRegister(TIM_HandleTypeDef *handle, Timer *obj) {
  for (auto &entry : g_timer_registry) {
    if (entry.handle == handle || entry.handle == nullptr) {
      entry = {handle, obj};
      return;
    }
  }
  Throw(std::runtime_error("timer_register: registry full"));
}

void TimerUnregister(TIM_HandleTypeDef *handle) {
  for (auto &entry : g_timer_registry) {
    if (entry.handle == handle) {
      entry = {nullptr, nullptr};
      return;
    }
  }
}

Timer *TimerFind(TIM_HandleTypeDef *handle) {
  for (auto &entry : g_timer_registry) {
    if (entry.handle == handle) {
      return entry.obj;
    }
  }
  return nullptr;
}

/**
 * @brief 判断TIM实例挂在APB2上还是APB1上。
 * @note  在librm支持的STM32系列里(F4、H7、L4)，都是 TIM1/TIM8/TIM9~TIM11/TIM15~TIM22 挂在APB2，
 *        其余挂在APB1。
 */
bool IsApb2Timer(const TIM_TypeDef *instance) {
  bool is_apb2 = false;
#if defined(TIM1_BASE)
  is_apb2 = is_apb2 || instance == TIM1;
#endif
#if defined(TIM8_BASE)
  is_apb2 = is_apb2 || instance == TIM8;
#endif
#if defined(TIM9_BASE)
  is_apb2 = is_apb2 || instance == TIM9;
#endif
#if defined(TIM10_BASE)
  is_apb2 = is_apb2 || instance == TIM10;
#endif
#if defined(TIM11_BASE)
  is_apb2 = is_apb2 || instance == TIM11;
#endif
#if defined(TIM15_BASE)
  is_apb2 = is_apb2 || instance == TIM15;
#endif
#if defined(TIM16_BASE)
  is_apb2 = is_apb2 || instance == TIM16;
#endif
#if defined(TIM17_BASE)
  is_apb2 = is_apb2 || instance == TIM17;
#endif
#if defined(TIM19_BASE)
  is_apb2 = is_apb2 || instance == TIM19;
#endif
#if defined(TIM20_BASE)
  is_apb2 = is_apb2 || instance == TIM20;
#endif
#if defined(TIM21_BASE)
  is_apb2 = is_apb2 || instance == TIM21;
#endif
#if defined(TIM22_BASE)
  is_apb2 = is_apb2 || instance == TIM22;
#endif
  return is_apb2;
}

/**
 * @brief 获取TIM的计数时钟频率（Hz）。
 *
 * STM32上定时器时钟与APB时钟的关系为：APB预分频系数为1时二者相等，否则定时器时钟是APB时钟的2倍
 * （TIMxPRE 位置1时为4倍，默认不置位）。这里通过比较 HCLK 与 PCLK 来判断APB预分频系数是否为1，
 * 从而避免直接访问各系列不同的RCC寄存器。
 */
u32 TimerClockFreq(const TIM_TypeDef *instance) {
  const u32 hclk = HAL_RCC_GetHCLKFreq();
  u32 multiplier = 2u;
#if defined(RCC_CFGR_TIMPRE)
  if ((RCC->CFGR & RCC_CFGR_TIMPRE) != 0u) {
    multiplier = 4u;
  }
#elif defined(RCC_DCKCFGR_TIMPRE)
  if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) != 0u) {
    multiplier = 4u;
  }
#endif
  const u32 pclk = IsApb2Timer(instance) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  return (pclk == hclk) ? pclk : pclk * multiplier;
}

}  // namespace

/**
 * @param htim 已由 CubeMX 初始化过的 TIM 句柄
 */
Timer::Timer(TIM_HandleTypeDef &htim)
    : htim_(&htim),
      running_(false),
      callback_{},
      type_(TimerType::kPeriodic),
      prescaler_(0),
      autoreload_(0),
      timeout_set_(false) {
  TimerRegister(htim_, this);
}

Timer::~Timer() {
  Stop();
  HAL_TIM_UnRegisterCallback(htim_, HAL_TIM_PERIOD_ELAPSED_CB_ID);
  TimerUnregister(htim_);
}

bool Timer::SetTimeout(std::chrono::duration<f32> timeout) {
  if (running_) {
    return false;
  }
  if (timeout <= std::chrono::duration<f32>::zero()) {
    return false;
  }

  const u32 timer_clk = TimerClockFreq(htim_->Instance);
  if (timer_clk == 0u) {
    Throw(std::runtime_error("timer: invalid timer clock"));
  }

  i64 us = std::chrono::duration_cast<std::chrono::microseconds>(timeout).count();
  if (us < 1) {
    us = 1;
  }

  // 先按不分频估算所需计数值，若超出16位则增大预分频，保证 ARR 装得下
  constexpr u64 kTicksPerFullRange = 1000000ull * (kMaxAutoReload + 1ull);
  const u64 need = static_cast<u64>(timer_clk) * static_cast<u64>(us);
  u64 prescaler = (need + kTicksPerFullRange - 1ull) / kTicksPerFullRange;
  if (prescaler > 0ull) {
    prescaler -= 1ull;
  }
  if (prescaler > kMaxAutoReload) {
    prescaler = kMaxAutoReload;
  }

  const u32 counter_clk = timer_clk / static_cast<u32>(prescaler + 1ull);
  u64 ticks = static_cast<u64>(counter_clk) * static_cast<u64>(us) / 1000000ull;
  if (ticks < 1ull) {
    ticks = 1ull;
  }
  if (ticks > kMaxAutoReload + 1ull) {
    ticks = kMaxAutoReload + 1ull;
  }

  prescaler_ = static_cast<u32>(prescaler);
  autoreload_ = static_cast<u32>(ticks - 1ull);
  timeout_set_ = true;
  return true;
}

std::chrono::duration<f32> Timer::GetTimeout() const {
  if (!timeout_set_) {
    return std::chrono::duration<f32>::zero();
  }

  const u32 timer_clk = TimerClockFreq(htim_->Instance);
  if (timer_clk == 0u) {
    return std::chrono::duration<f32>::zero();
  }

  // 周期只由 PSC 和 ARR 决定，这两个值在 SetTimeout() 里已经算好，
  // 因此定时器未 Start() 时也能查到与启动后一致的周期。
  const f32 ticks = static_cast<f32>(prescaler_ + 1u) * static_cast<f32>(autoreload_ + 1u);
  return std::chrono::duration<f32>(ticks / static_cast<f32>(timer_clk));
}

bool Timer::Start() {
  if (!timeout_set_) {
    return false;  // 还没有成功设置过超时时间，启动失败
  }
  Stop();

  __HAL_TIM_SET_PRESCALER(htim_, prescaler_);
  __HAL_TIM_SET_AUTORELOAD(htim_, autoreload_);
  __HAL_TIM_SET_COUNTER(htim_, 0);
  // 产生一次更新事件，让 PSC/ARR 立即载入影子寄存器，并清掉由此产生的更新标志
  htim_->Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(htim_, TIM_FLAG_UPDATE);

  LIBRM_STM32_HAL_ASSERT(HAL_TIM_RegisterCallback(htim_, HAL_TIM_PERIOD_ELAPSED_CB_ID, &Timer::PeriodElapsedThunk));
  // 先置位运行状态再开中断，避免中断在两者之间触发导致状态判断出错
  running_ = true;
  // 不能在宏参数里用status这个名字，会和LIBRM_STM32_HAL_ASSERT内部声明的变量重名
  const HAL_StatusTypeDef hal_status = HAL_TIM_Base_Start_IT(htim_);
  if (hal_status != HAL_OK) {
    running_ = false;
    LIBRM_STM32_HAL_ASSERT(hal_status);
  }
  return running_;
}

bool Timer::Stop() {
  if (!running_) {
    return false;
  }
  __HAL_TIM_CLEAR_FLAG(htim_, TIM_FLAG_UPDATE);
  LIBRM_STM32_HAL_ASSERT(HAL_TIM_Base_Stop_IT(htim_));
  running_ = false;
  return true;
}

void Timer::PeriodElapsedThunk(TIM_HandleTypeDef *htim) {
  if (Timer *timer = TimerFind(htim); timer != nullptr) {
    timer->PeriodElapsedCallback();
  }
}

void Timer::PeriodElapsedCallback() {
  if (type_ == TimerType::kOneShot) {
    // 单次定时：触发一次后立刻停止，再通知用户
    running_ = false;
    LIBRM_STM32_HAL_ASSERT(HAL_TIM_Base_Stop_IT(htim_));
    __HAL_TIM_CLEAR_FLAG(htim_, TIM_FLAG_UPDATE);
  }

  if (callback_) {
    callback_();
  }
}

}  // namespace rm::hal::stm32

#endif  // HAL_TIM_MODULE_ENABLED
