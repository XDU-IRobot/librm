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
 * @file  librm/device/remote/sbus.cc
 * @brief 通用SBUS接收机
 */

#include "sbus.hpp"

#include "librm/device/actuator/directdrive_motor.hpp"

namespace rm::device {

/**
 * @param serial 串口对象
 */
Sbus::Sbus(hal::SerialInterface &serial) : serial_(&serial) {
  static hal::SerialRxCallbackFunction rx_callback =
      std::bind(&Sbus::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  this->serial_->AttachRxCallback(rx_callback);
}

/**
 * @brief 开始接收遥控器数据
 */
void Sbus::Begin() { this->serial_->Begin(); }

void Sbus::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  if (rx_len != kSbusFrameSize || data[0] != kSbusStartByte || data[24] != kSbusEndByte) {
    return;
  }

  // 16 channels of 11-bit data
  channels_[0] = (data[1] | data[2] << 8) & 0x07FF;
  channels_[1] = (data[2] >> 3 | data[3] << 5) & 0x07FF;
  channels_[2] = (data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF;
  channels_[3] = (data[5] >> 1 | data[6] << 7) & 0x07FF;
  channels_[4] = (data[6] >> 4 | data[7] << 4) & 0x07FF;
  channels_[5] = (data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF;
  channels_[6] = (data[9] >> 2 | data[10] << 6) & 0x07FF;
  channels_[7] = (data[10] >> 5 | data[11] << 3) & 0x07FF;
  channels_[8] = (data[12] | data[13] << 8) & 0x07FF;
  channels_[9] = (data[13] >> 3 | data[14] << 5) & 0x07FF;
  channels_[10] = (data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF;
  channels_[11] = (data[16] >> 1 | data[17] << 7) & 0x07FF;
  channels_[12] = (data[17] >> 4 | data[18] << 4) & 0x07FF;
  channels_[13] = (data[18] >> 7 | data[19] << 1 | data[20] << 9) & 0x07FF;
  channels_[14] = (data[20] >> 2 | data[21] << 6) & 0x07FF;
  channels_[15] = (data[21] >> 5 | data[22] << 3) & 0x07FF;

  // flags
  digital_channel_1_ = (data[23] & 0x01);
  digital_channel_2_ = (data[23] & 0x02);
  frame_lost_ = (data[23] & 0x04);
  failsafe_ = (data[23] & 0x08);

  if (failsafe_) {
    ReportStatus(kFault);
  } else {
    ReportStatus(kOk);
  }
}

i16 Sbus::channel(int channel_num) const {
  if (channel_num >= 0 && channel_num < kNumChannels) {
    return channels_[channel_num];
  }
  return 0;
}

bool Sbus::digital_channel_1() const { return digital_channel_1_; }
bool Sbus::digital_channel_2() const { return digital_channel_2_; }
bool Sbus::failsafe() const { return failsafe_; }
bool Sbus::frame_lost() const { return frame_lost_; }

}  // namespace rm::device
