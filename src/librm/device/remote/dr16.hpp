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
 * @file  librm/device/remote/dr16.hpp
 * @brief DR16接收机
 */

#ifndef LIBRM_DEVICE_REMOTE_DR16_HPP
#define LIBRM_DEVICE_REMOTE_DR16_HPP

#include <vector>

#include "librm/core/typedefs.hpp"
#include "librm/hal/serial.hpp"

namespace rm::device {

/**
 * @brief 遥控器拨杆位置
 */
enum class RcSwitchState : usize {
  kUnknown = 0u,
  kUp,
  kDown,
  kMid,
};

/**
 * @brief 键盘按键
 */
enum class RcKey : u16 {
  kW = 1u << 0,
  kS = 1u << 1,
  kA = 1u << 2,
  kD = 1u << 3,
  kShift = 1u << 4,
  kCtrl = 1u << 5,
  kQ = 1u << 6,
  kE = 1u << 7,
  kR = 1u << 8,
  kF = 1u << 9,
  kG = 1u << 10,
  kZ = 1u << 11,
  kX = 1u << 12,
  kC = 1u << 13,
  kV = 1u << 14,
  kB = 1u << 15,
};

/**
 * @brief DR16接收机
 */
class DR16 {
 public:
  DR16() = delete;
  explicit DR16(hal::SerialInterface &serial);

  void Begin();
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

  [[nodiscard]] i16 left_x() const;
  [[nodiscard]] i16 left_y() const;
  [[nodiscard]] i16 right_x() const;
  [[nodiscard]] i16 right_y() const;
  [[nodiscard]] i16 dial() const;
  [[nodiscard]] RcSwitchState switch_l() const;
  [[nodiscard]] RcSwitchState switch_r() const;
  [[nodiscard]] i16 mouse_x() const;
  [[nodiscard]] i16 mouse_y() const;
  [[nodiscard]] i16 mouse_z() const;
  [[nodiscard]] bool mouse_button_left() const;
  [[nodiscard]] bool mouse_button_right() const;
  [[nodiscard]] bool key(RcKey key) const;

 private:
  hal::SerialInterface *serial_;

  i16 axes_[5]{0};   // [0]: right_x, [1]: right_y, [2]: left_x, [3]: left_y, [4]: dial; 取值范围:-660~660;
  i16 mouse_[3]{0};  // [0]: x, [1]: y, [2]: z; 取值范围:-32768~32767;
  bool mouse_button_[2]{false};                         // [0]: left, [1]: right
  RcSwitchState switches_[2]{RcSwitchState::kUnknown};  // [0]: right, [1]: left
  u16 keyboard_key_;                                    // 每一位代表一个键，0为未按下，1为按下
};

}  // namespace rm::device

#endif
