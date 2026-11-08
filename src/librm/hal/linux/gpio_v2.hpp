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
 * @file    librm/hal/linux/gpio_v2.hpp
 * @brief   基于 libgpiod v2 API 的 GPIO 引脚封装
 */

#ifndef LIBRM_HAL_LINUX_GPIO_V2_HPP
#define LIBRM_HAL_LINUX_GPIO_V2_HPP

#if defined(LIBRM_PLATFORM_LINUX) && defined(LIBRM_LIBGPIOD_API_V2)

#include <gpiod.h>

#include "librm/hal/gpio_interface.hpp"

namespace rm::hal::linux_ {

/**
 * @brief 基于 libgpiod v2 API 的 GPIO 引脚
 */
class PinV2 final : public PinInterface {
 public:
  /**
   * @param offset    引脚在 GPIO 控制器内的偏移
   * @param direction 引脚方向，请求后不可更改
   * @param chip      GPIO 控制器名称或设备路径
   */
  PinV2(unsigned int offset, PinDirection direction, const char *chip = "gpiochip0");
  PinV2() = delete;
  ~PinV2() override;

  PinV2(const PinV2 &) = delete;
  PinV2 &operator=(const PinV2 &) = delete;

  void Write(bool state) override;
  [[nodiscard]] bool Read() const override;

 private:
  gpiod_chip *chip_;
  gpiod_line_request *request_;
  unsigned int offset_;
  bool is_output_;
  bool state_{};
};

using Pin = PinV2;

}  // namespace rm::hal::linux_

#endif  // LIBRM_PLATFORM_LINUX && LIBRM_LIBGPIOD_API_V2

#endif  // LIBRM_HAL_LINUX_GPIO_V2_HPP
