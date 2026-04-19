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
 * @file  librm/hal/stm32/posix_complement.cc
 * @brief 补全一些stm32平台下缺失的POSIX函数
 */

#include "librm/hal/stm32/hal.hpp"

#include <sys/_timeval.h>
#include <unistd.h>

extern "C" {

/**
 * @note 关于 gettimeofday 的时间精度
 *
 * HAL_GetTick() 只有毫秒级分辨率。为获取亚毫秒精度，需要读取计数器寄存器来
 * 获取当前毫秒内的微秒余量。具体使用哪个计数器取决于 CubeMX 中配置的 HAL Tick 时基来源：
 *
 *   - 时基为 SysTick（CubeMX 默认）：自动探测后读取 SysTick->VAL，精度 ~1 µs
 *   - 时基为外部 TIMx：读取 TIMx->CNT，精度 ~1 µs，前提：该 TIM 必须配置为递增计数、计数频率 1 MHz
 *     当某个 TIM 被选为 HAL Tick 时基时，CubeMX 会自动将其预分频器设置为使计数频率为 1 MHz，并将 ARR 设置为 999（每
 * 1000 counts = 1 ms），因此无需用户额外配置。
 *
 * 若未定义 LIBRM_STM32_TIMEBASE_SOURCE，则在运行时通过检查 SysTick->LOAD 是否为 0 自动判断：
 *   - SysTick->LOAD != 0：SysTick 已被 HAL 初始化，使用 SysTick 路径
 *   - SysTick->LOAD == 0：SysTick 未初始化（时基为外部 TIM），退化为1ms精度，此时一些时间相关功能会表现不正常
 *
 * 若时基为外部 TIMx，通过编译期宏指定具体外设可恢复亚毫秒精度：
 *   -DLIBRM_STM32_TIMEBASE_SOURCE=TIM7（TIM6、TIM2 等其他 TIM 同理）
 */
__attribute__((weak)) int gettimeofday(struct timeval *tv, struct timezone *tz) {
  (void)tz;
  uint64_t total_us;

#ifdef LIBRM_STM32_TIMEBASE_SOURCE
  // 编译期指定：外部 TIMx 时基（如 TIM7）
  // 前提：该 TIM 为递增计数、1 MHz 计数频率、ARR = 999（HAL_InitTick 自动生成的默认配置）
  {
    const uint32_t arr = LIBRM_STM32_TIMEBASE_SOURCE->ARR;
    uint32_t ms;
    uint32_t cnt;
    do {
      ms = HAL_GetTick();
      cnt = LIBRM_STM32_TIMEBASE_SOURCE->CNT;
    } while (ms != HAL_GetTick());
    // us_in_ms = CNT * 1000 / (ARR + 1)，ARR=999 时退化为 cnt * 1
    const uint32_t us_in_ms = cnt * 1000u / (arr + 1u);
    total_us = ((uint64_t)ms * 1000u) + us_in_ms;
  }
#else
  // 未指定：运行时自动探测
  // SysTick->LOAD == 0 说明 SysTick 未被 HAL 初始化（时基由外部 TIM 驱动），退化为纯毫秒精度
  {
    auto *systick_regs = reinterpret_cast<volatile uint32_t *>(SysTick_BASE);  // NOLINT(*-no-int-to-ptr)
    const uint32_t load = systick_regs[1];                                     // offset 4 = LOAD 寄存器
    if (load == 0u) {
      total_us = (uint64_t)HAL_GetTick() * 1000u;
    } else {
      uint32_t ms;
      uint32_t val;
      do {
        ms = HAL_GetTick();
        val = systick_regs[2];  // offset 8 = VAL 寄存器
      } while (ms != HAL_GetTick());
      const uint32_t us_in_ms = (load - val) * 1000u / (load + 1u);
      total_us = ((uint64_t)ms * 1000u) + us_in_ms;
    }
  }
#endif

  tv->tv_sec = (time_t)(total_us / 1000000u);
  tv->tv_usec = (suseconds_t)(total_us % 1000000u);
  return 0;
}

__attribute__((weak)) unsigned int sleep(unsigned int seconds) {
  (void)seconds;
  // TODO
  return 1;
}

__attribute__((weak)) int usleep(useconds_t useconds) {
  (void)useconds;
  // TODO
  return 1;
}
}