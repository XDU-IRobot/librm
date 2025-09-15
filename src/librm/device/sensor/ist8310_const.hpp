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
 * @file    librm/device/sensor/ist8310_const.hpp
 * @brief   IST8310有关常量
 */

#ifndef LIBRM_DEVICE_SENSOR_IST8310_CONST_H
#define LIBRM_DEVICE_SENSOR_IST8310_CONST_H

#include <chrono>

#include "librm/core/typedefs.h"

using namespace std::chrono_literals;

constexpr u8 IST8310_I2C_ADDRESS = 0x0E;
constexpr u8 IST8310_WHO_AM_I = 0x00;        // ist8310 "who am I"
constexpr u8 IST8310_WHO_AM_I_VALUE = 0x10;  // device ID

constexpr auto IST8310_COMM_WAIT_TIME_SHORT = 150us;
constexpr auto IST8310_COMM_WAIT_TIME_LONG = 50ms;

constexpr u8 IST8310_DATA_READY_BIT = 2;

constexpr u8 IST8310_DATA_OUT_X_L = 0x03;

constexpr f32 IST8310_SENSITIVITY = 0.3f;  // 原始数据到单位ut的换算系数

#endif  // LIBRM_DEVICE_SENSOR_IST8310_CONST_H
