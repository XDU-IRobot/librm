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
 * @file    librm/hal/raspi/gpio.hpp
 * @brief   wiringPi的gpio封装
 */

#ifndef LIBRM_HAL_RASPI_GPIO_HPP
#define LIBRM_HAL_RASPI_GPIO_HPP

#if defined(LIBRM_PLATFORM_LINUX_RASPI)

#include <wiringPi.h>

#include "librm/core/typedefs.hpp"
#include "librm/hal/gpio_interface.hpp"
#include "librm/hal/raspi/wiringpi_init.hpp"

namespace rm::hal::raspi {

/**
 * @brief   GPIO引脚类
 * @tparam  mode INPUT/OUTPUT, other modes todo
 */
template <int mode>
class Pin final : public PinInterface {
 public:
  Pin(usize pin) : pin_{pin} {
    static_assert(mode == OUTPUT || mode == INPUT, "unsupported pin mode!");
    InitWiringPi();
    pinMode(pin, mode);
  }

  Pin() = delete;
  ~Pin() override = default;

  void Write(bool state) override {
    if constexpr (mode == OUTPUT) {
      digitalWrite(pin_, state);
      state_ = state;
    }
  }

  [[nodiscard]] bool Read() const override {
    if constexpr (mode == OUTPUT) {
      return state_;
    } else if constexpr (mode == INPUT) {
      return digitalRead(pin_);
    }
  }

 protected:
  usize pin_;
  bool state_{};
};

}  // namespace rm::hal::raspi
#endif

#endif  // LIBRM_HAL_RASPI_GPIO_HPP
