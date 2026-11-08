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
 * @file    librm/hal/linux/gpio_v1.cc
 * @brief   基于 libgpiod v1 API 的 GPIO 引脚封装
 */

#include "gpio_v1.hpp"

#if defined(LIBRM_PLATFORM_LINUX) && !defined(LIBRM_LIBGPIOD_API_V2)

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace rm::hal::linux_ {

namespace {

[[noreturn]] void ThrowErrno(const char *what) {
  throw std::runtime_error(std::string{what} + ": " + std::strerror(errno));
}

std::string ChipPath(const char *chip) { return chip[0] == '/' ? chip : std::string{"/dev/"} + chip; }

}  // namespace

PinV1::PinV1(unsigned int offset, PinDirection direction, const char *chip)
    : chip_{nullptr}, line_{nullptr}, is_output_{direction == PinDirection::kOutput} {
  chip_ = gpiod_chip_open(ChipPath(chip).c_str());
  if (chip_ == nullptr) {
    ThrowErrno("gpiod_chip_open");
  }

  line_ = gpiod_chip_get_line(chip_, offset);
  if (line_ == nullptr) {
    const int err = errno;
    gpiod_chip_close(chip_);
    errno = err;
    ThrowErrno("gpiod_chip_get_line");
  }

  const int ret = is_output_ ? gpiod_line_request_output(line_, "librm", 0) : gpiod_line_request_input(line_, "librm");
  if (ret < 0) {
    const int err = errno;
    gpiod_chip_close(chip_);
    errno = err;
    ThrowErrno("gpiod_line_request");
  }
}

PinV1::~PinV1() {
  if (line_ != nullptr) {
    gpiod_line_release(line_);
  }
  if (chip_ != nullptr) {
    gpiod_chip_close(chip_);
  }
}

void PinV1::Write(bool state) {
  if (!is_output_) {
    return;
  }
  if (gpiod_line_set_value(line_, state ? 1 : 0) < 0) {
    ThrowErrno("gpiod_line_set_value");
  }
  state_ = state;
}

bool PinV1::Read() const {
  if (is_output_) {
    return state_;
  }
  const int value = gpiod_line_get_value(line_);
  if (value < 0) {
    ThrowErrno("gpiod_line_get_value");
  }
  return value == 1;
}

}  // namespace rm::hal::linux_

#endif  // LIBRM_PLATFORM_LINUX && !LIBRM_LIBGPIOD_API_V2
