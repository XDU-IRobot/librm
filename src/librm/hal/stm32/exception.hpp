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
 * @file  librm/hal/stm32/exception.h
 * @brief 对STM32 HAL库里异常枚举的封装
 */

#ifndef LIBRM_HAL_STM32_EXCEPTION_HPP
#define LIBRM_HAL_STM32_EXCEPTION_HPP

#include <stdexcept>

#include "librm/hal/stm32/hal.hpp"

namespace rm {

class hal_error : public std::exception {
 public:
  hal_error(HAL_StatusTypeDef status) {
    switch (status) {
      case HAL_OK:
        msg_ = "HAL_OK";
        break;
      case HAL_ERROR:
        msg_ = "HAL_ERROR";
        break;
      case HAL_BUSY:
        msg_ = "HAL_BUSY";
        break;
      case HAL_TIMEOUT:
        msg_ = "HAL_TIMEOUT";
        break;
      default:
        msg_ = "Unknown HAL error";
        break;
    }
  }
  const char* what() const noexcept override { return msg_; }

 private:
  const char* msg_;
};

}  // namespace rm

#endif  // LIBRM_HAL_STM32_EXCEPTION_HPP
