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
 * @file  librm/device/actuator/dji_motor.cc
 * @brief 大疆电机类库
 */

#include "dji_motor.hpp"

namespace rm::device {

/**
 * @brief 三种型号电机各自的发送缓冲区
 */
std::unordered_map<hal::CanInterface *, std::array<u8, 18>> DjiMotorProperties<DjiMotorType::GM6020>::tx_buf_{};
std::unordered_map<hal::CanInterface *, std::array<u8, 18>> DjiMotorProperties<DjiMotorType::M3508>::tx_buf_{};
std::unordered_map<hal::CanInterface *, std::array<u8, 18>> DjiMotorProperties<DjiMotorType::M2006>::tx_buf_{};

}  // namespace rm::device
