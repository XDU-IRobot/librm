/*
  Copyright (c) 2024 XDU-IRobot

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
 * @file  librm/core/time.hpp
 * @brief 时间模块，用来包装不同平台的时间相关功能
 */

#ifndef LIBRM_CORE_TIME_HPP
#define LIBRM_CORE_TIME_HPP

#include <chrono>
#include <thread>

#if defined(LIBRM_PLATFORM_STM32)
#include "librm/core/cmsis_rtos.h"
#include "librm/hal/stm32/hal.h"
#endif
#include "librm/core/typedefs.h"

namespace rm::core::time {

// STM32平台的延时比较复杂，所以专门实现两个函数
#if defined(LIBRM_PLATFORM_STM32)
/**
 * @brief 给STM32平台使用的延时函数
 * @param ms 延时时间，单位为毫秒
 */
inline void SleepMs(u32 ms) {
#ifdef LIBRM_USE_CMSIS_RTOS
  if (__get_IPSR()) {  // 检测当前是否在中断里，如果在中断里则调用HAL_Delay，否则调用osDelay
    HAL_Delay(ms);
  } else {
    osDelay(ms);
  }
#else
  HAL_Delay(ms);
#endif
}

/**
 * @brief 给STM32平台使用的延时函数
 * @param us 延时时间，单位为微秒
 */
inline void SleepUs(u32 us) {
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }
  if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  u32 start = DWT->CYCCNT;
  u32 ticks = us * (HAL_RCC_GetSysClockFreq() / 1000000);

  while ((DWT->CYCCNT - start) < ticks) {
    ;
  }
}
#endif

/**
 * @param  duration 延时时间
 */
template <typename Rep, typename Period>
void Sleep(const std::chrono::duration<Rep, Period> &duration) {
#if defined(LIBRM_PLATFORM_STM32)
  if (duration < std::chrono::milliseconds(1)) {
    SleepUs(std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
  } else {
    SleepMs(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());
  }
#else
  std::this_thread::sleep_for(duration);
#endif
}
}  // namespace rm::core::time

#endif  // LIBRM_CORE_TIME_HPP
