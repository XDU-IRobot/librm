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
 * @file  librm/hal/stm32/posix_complement.cc
 * @brief 补全一些stm32平台下缺失的POSIX函数
 */

#include "librm/hal/stm32/hal.hpp"

#include <sys/_timeval.h>
#include <unistd.h>

extern "C" {
__attribute__((weak)) int gettimeofday(struct timeval *tv, struct timezone *tz) {
  const auto millis = HAL_GetTick();
  tv->tv_sec = millis / 1000;
  tv->tv_usec = (millis % 1000) * 1000;
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