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
__attribute__((weak)) int gettimeofday(struct timeval *tv, struct timezone *tz) {
  const uint32_t load = SysTick->LOAD;
  uint32_t ms, systick_val;
  // 避免在读取 HAL_GetTick() 和 SysTick->VAL 之间发生 tick 中断导致不一致
  do {
    ms = HAL_GetTick();
    systick_val = SysTick->VAL;
  } while (ms != HAL_GetTick());

  // SysTick 从 LOAD 递减到 0，周期 = LOAD + 1 个 tick
  // 当前毫秒内已经过的微秒数 = (LOAD - VAL) * 1000 / (LOAD + 1)
  uint32_t us_fraction = (load - systick_val) * 1000u / (load + 1u);

  uint64_t total_us = (uint64_t)ms * 1000u + us_fraction;
  tv->tv_sec = (time_t)(total_us / 1000000u);
  tv->tv_usec = (suseconds_t)(total_us % 1000000u);
  return 0;
}

__attribute__((weak)) unsigned int sleep(unsigned int seconds) {
  // TODO
  return 1;
}

__attribute__((weak)) int usleep(useconds_t __useconds) {
  // TODO
  return 1;
}
}