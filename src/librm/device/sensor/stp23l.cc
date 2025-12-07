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
 * @file  librm/device/sensor/stp23l.cc
 * @brief STP23L激光测距模块
 */

#include "stp23l.hpp"

namespace rm::device {

/**
 * @param serial 串口对象
 */
STP23L::STP23L(hal::SerialInterface &serial) : serial_(&serial) {
  static hal::SerialRxCallbackFunction rx_callback =
      std::bind(&STP23L::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  this->serial_->AttachRxCallback(rx_callback);
}

/**
 * @brief 开始接收数据
 */
void STP23L::Begin() const { this->serial_->Begin(); }

void STP23L::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  for (u16 i = 0; i < rx_len; i++) {
    ProcessByte(data[i]);
  }
}

/**
 * @brief 逐字节处理接收到的数据，用状态机在连续字节流上实现滑动窗口，找到4个连续的0xAA帧头后开始解析
 * @param byte 接收到的字节
 */
void STP23L::ProcessByte(u8 byte) {
  switch (this->parse_state_) {
    case kWaitHeader:
      // 等待4个连续的0xAA作为帧头
      if (byte == kHeader) {
        this->header_count_++;
        if (this->header_count_ >= 4) {
          this->parse_state_ = kDeviceAddr;
          this->crc_ = 0;
          this->header_count_ = 0;
        }
      } else {
        this->header_count_ = 0;
      }
      break;

    case kDeviceAddr:
      if (byte == kDeviceAddress) {
        this->crc_ += byte;
        this->parse_state_ = kPacketType;
      } else {
        this->parse_state_ = kWaitHeader;
        this->header_count_ = 0;
      }
      break;

    case kPacketType:
      // 目前只处理获取距离数据包
      if (byte == static_cast<u8>(STP23LPacketType::kGetDistance)) {
        this->crc_ += byte;
        this->parse_state_ = kChunkOffset1;
      } else {
        // 其他类型的包暂不处理，重新等待帧头
        this->parse_state_ = kWaitHeader;
        this->header_count_ = 0;
        this->crc_ = 0;
      }
      break;

    case kChunkOffset1:
      if (byte == kChunkOffset) {
        this->crc_ += byte;
        this->parse_state_ = kChunkOffset2;
      } else {
        this->parse_state_ = kWaitHeader;
        this->header_count_ = 0;
        this->crc_ = 0;
      }
      break;

    case kChunkOffset2:
      if (byte == kChunkOffset) {
        this->crc_ += byte;
        this->parse_state_ = kDataLenLow;
      } else {
        this->parse_state_ = kWaitHeader;
        this->header_count_ = 0;
        this->crc_ = 0;
      }
      break;

    case kDataLenLow:
      this->data_len_ = static_cast<u16>(byte);
      this->crc_ += byte;
      this->parse_state_ = kDataLenHigh;
      break;

    case kDataLenHigh:
      this->data_len_ |= (static_cast<u16>(byte) << 8);
      this->crc_ += byte;

      // 验证数据长度是否合理（12点*15字节+4字节时间戳=184字节）
      if (this->data_len_ == 184) {
        this->parse_state_ = kData;
        this->data_idx_ = 0;
      } else {
        // 数据长度异常，重新等待帧头
        this->parse_state_ = kWaitHeader;
        this->header_count_ = 0;
        this->crc_ = 0;
      }
      break;

    case kData:
      this->crc_ += byte;
      this->data_buffer_[this->data_idx_++] = byte;

      // 数据段接收完成
      if (this->data_idx_ >= this->data_len_) {
        this->parse_state_ = kChecksum;
      }
      break;

    case kChecksum:
      // 校验成功
      if (byte == this->crc_) {
        this->ParseDataBuffer();
        this->ProcessData();
        this->frame_count_++;
        ReportStatus(kOk);
      }
      // 无论校验成功与否，都重新开始等待下一帧
      this->parse_state_ = kWaitHeader;
      this->header_count_ = 0;
      this->crc_ = 0;
      break;

    default:
      this->parse_state_ = kWaitHeader;
      this->header_count_ = 0;
      this->crc_ = 0;
      break;
  }
}

/**
 * @brief 从数据缓冲区解析出12个点的数据和时间戳
 */
void STP23L::ParseDataBuffer() {
  usize idx = 0;

  // 解析12个点的数据，每个点15字节
  for (usize i = 0; i < kPointsPerFrame; i++) {
    auto &point = this->raw_points_[i];

    // distance (2 bytes, little-endian)
    point.distance = static_cast<i16>(this->data_buffer_[idx]) | (static_cast<i16>(this->data_buffer_[idx + 1]) << 8);
    idx += 2;

    // noise (2 bytes, little-endian)
    point.noise = static_cast<u16>(this->data_buffer_[idx]) | (static_cast<u16>(this->data_buffer_[idx + 1]) << 8);
    idx += 2;

    // peak (4 bytes, little-endian)
    point.peak = static_cast<u32>(this->data_buffer_[idx]) | (static_cast<u32>(this->data_buffer_[idx + 1]) << 8) |
                 (static_cast<u32>(this->data_buffer_[idx + 2]) << 16) |
                 (static_cast<u32>(this->data_buffer_[idx + 3]) << 24);
    idx += 4;

    // confidence (1 byte)
    point.confidence = this->data_buffer_[idx];
    idx += 1;

    // intg (4 bytes, little-endian)
    point.intg = static_cast<u32>(this->data_buffer_[idx]) | (static_cast<u32>(this->data_buffer_[idx + 1]) << 8) |
                 (static_cast<u32>(this->data_buffer_[idx + 2]) << 16) |
                 (static_cast<u32>(this->data_buffer_[idx + 3]) << 24);
    idx += 4;

    // reftof (2 bytes, little-endian)
    point.reftof = static_cast<i16>(this->data_buffer_[idx]) | (static_cast<i16>(this->data_buffer_[idx + 1]) << 8);
    idx += 2;
  }

  // 解析时间戳 (4 bytes, little-endian) - 目前不使用，但需要跳过
  // u32 timestamp = static_cast<u32>(this->data_buffer_[idx]) |
  //                 (static_cast<u32>(this->data_buffer_[idx + 1]) << 8) |
  //                 (static_cast<u32>(this->data_buffer_[idx + 2]) << 16) |
  //                 (static_cast<u32>(this->data_buffer_[idx + 3]) << 24);
}

/**
 * @brief 数据处理，对12个点进行平均
 */
void STP23L::ProcessData() {
  // 清空累加变量
  i32 sum_distance = 0;
  u32 sum_noise = 0;
  u64 sum_peak = 0;
  u32 sum_confidence = 0;
  u64 sum_intg = 0;
  i32 sum_reftof = 0;
  u8 valid_count = 0;

  // 累加所有有效点（距离不为0）
  for (usize i = 0; i < kPointsPerFrame; i++) {
    if (this->raw_points_[i].distance != 0) {
      sum_distance += this->raw_points_[i].distance;
      sum_noise += this->raw_points_[i].noise;
      sum_peak += this->raw_points_[i].peak;
      sum_confidence += this->raw_points_[i].confidence;
      sum_intg += this->raw_points_[i].intg;
      sum_reftof += this->raw_points_[i].reftof;
      valid_count++;
    }
  }

  // 计算平均值
  if (valid_count > 0) {
    this->averaged_data_.distance = static_cast<i16>(sum_distance / valid_count);
    this->averaged_data_.noise = static_cast<u16>(sum_noise / valid_count);
    this->averaged_data_.peak = static_cast<u32>(sum_peak / valid_count);
    this->averaged_data_.confidence = static_cast<u8>(sum_confidence / valid_count);
    this->averaged_data_.intg = static_cast<u32>(sum_intg / valid_count);
    this->averaged_data_.reftof = static_cast<i16>(sum_reftof / valid_count);
  }
}

i16 STP23L::distance() const { return this->averaged_data_.distance; }
u16 STP23L::noise() const { return this->averaged_data_.noise; }
u32 STP23L::peak() const { return this->averaged_data_.peak; }
u8 STP23L::confidence() const { return this->averaged_data_.confidence; }
u32 STP23L::intg() const { return this->averaged_data_.intg; }
i16 STP23L::reftof() const { return this->averaged_data_.reftof; }
u16 STP23L::frame_count() const { return this->frame_count_; }

const STP23L::PointData &STP23L::GetRawPoint(u8 index) const {
  if (index >= kPointsPerFrame) {
    index = kPointsPerFrame - 1;
  }
  return this->raw_points_[index];
}

}  // namespace rm::device
