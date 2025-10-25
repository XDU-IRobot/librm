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
 * @file  librm/device/remote/sbus.hpp
 * @brief 通用SBUS接收机
 */

#ifndef LIBRM_DEVICE_REMOTE_SBUS_HPP
#define LIBRM_DEVICE_REMOTE_SBUS_HPP

#include <vector>

#include "librm/core/typedefs.hpp"
#include "librm/device/device.hpp"
#include "librm/hal/serial.hpp"

namespace rm::device {

/**
 * @brief 通用SBUS接收机
 */
class Sbus : public Device {
 public:
  Sbus() = delete;
  explicit Sbus(hal::SerialInterface &serial);

  void Begin();
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

  [[nodiscard]] i16 channel(int channel_num) const;
  [[nodiscard]] bool digital_channel_1() const;
  [[nodiscard]] bool digital_channel_2() const;
  [[nodiscard]] bool failsafe() const;
  [[nodiscard]] bool frame_lost() const;

 private:
  static constexpr u8 kSbusStartByte = 0x0F;
  static constexpr u8 kSbusEndByte = 0x00;
  static constexpr u16 kSbusFrameSize = 25;
  static constexpr int kNumChannels = 16;

  hal::SerialInterface *serial_;

  i16 channels_[kNumChannels]{0};
  bool digital_channel_1_{false};
  bool digital_channel_2_{false};
  bool failsafe_{false};
  bool frame_lost_{false};
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_REMOTE_SBUS_HPP
