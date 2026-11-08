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
 * @file    librm/hal/linux/gpio_v2.cc
 * @brief   基于 libgpiod v2 API 的 GPIO 引脚封装
 */

#include "gpio_v2.hpp"

#if defined(LIBRM_PLATFORM_LINUX) && defined(LIBRM_LIBGPIOD_API_V2)

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

PinV2::PinV2(unsigned int offset, PinDirection direction, const char *chip)
    : chip_{nullptr}, request_{nullptr}, offset_{offset}, is_output_{direction == PinDirection::kOutput} {
  chip_ = gpiod_chip_open(ChipPath(chip).c_str());
  if (chip_ == nullptr) {
    ThrowErrno("gpiod_chip_open");
  }

  gpiod_line_settings *settings = gpiod_line_settings_new();
  gpiod_line_config *line_config = gpiod_line_config_new();
  gpiod_request_config *request_config = gpiod_request_config_new();
  if (settings == nullptr || line_config == nullptr || request_config == nullptr) {
    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_config);
    gpiod_request_config_free(request_config);
    gpiod_chip_close(chip_);
    ThrowErrno("gpiod request config alloc");
  }

  gpiod_line_settings_set_direction(settings, is_output_ ? GPIOD_LINE_DIRECTION_OUTPUT : GPIOD_LINE_DIRECTION_INPUT);
  if (is_output_) {
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
  }
  gpiod_line_config_add_line_settings(line_config, &offset_, 1, settings);
  gpiod_request_config_set_consumer(request_config, "librm");
  request_ = gpiod_chip_request_lines(chip_, request_config, line_config);

  gpiod_request_config_free(request_config);
  gpiod_line_config_free(line_config);
  gpiod_line_settings_free(settings);

  if (request_ == nullptr) {
    const int err = errno;
    gpiod_chip_close(chip_);
    errno = err;
    ThrowErrno("gpiod_chip_request_lines");
  }
}

PinV2::~PinV2() {
  if (request_ != nullptr) {
    gpiod_line_request_release(request_);
  }
  if (chip_ != nullptr) {
    gpiod_chip_close(chip_);
  }
}

void PinV2::Write(bool state) {
  if (!is_output_) {
    return;
  }
  if (gpiod_line_request_set_value(request_, offset_, state ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE) <
      0) {
    ThrowErrno("gpiod_line_request_set_value");
  }
  state_ = state;
}

bool PinV2::Read() const {
  if (is_output_) {
    return state_;
  }
  const int value = gpiod_line_request_get_value(request_, offset_);
  if (value < 0) {
    ThrowErrno("gpiod_line_request_get_value");
  }
  return value == GPIOD_LINE_VALUE_ACTIVE;
}

}  // namespace rm::hal::linux_

#endif  // LIBRM_PLATFORM_LINUX && LIBRM_LIBGPIOD_API_V2
