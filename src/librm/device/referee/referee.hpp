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
 * @file  librm/device/referee/referee.hpp
 * @brief 裁判系统
 */

#ifndef LIBRM_DEVICE_REFEREE_REFEREE_HPP
#define LIBRM_DEVICE_REFEREE_REFEREE_HPP

#include "protocol_v164.hpp"
#include "protocol_v170.hpp"
// implement and add more revisions here

#include <array>

#include "librm/modules/algorithm/crc.h"

namespace rm::device {

/**
 * @brief 裁判系统
 */
template <RefereeRevision revision>
class Referee {
 private:
  enum class DeserializeFsmState {
    kSof,
    kLenLsb,
    kLenMsb,
    kSeq,
    kCrc8,
    kCrc16,
  } deserialize_fsm_state_{DeserializeFsmState::kSof};

 public:
  Referee() = default;

  void operator<<(u8 data) {
    switch (deserialize_fsm_state_) {
      case DeserializeFsmState::kSof: {
        if (data == kRefProtocolHeaderSof) {
          deserialize_fsm_state_ = DeserializeFsmState::kLenLsb;
          valid_data_so_far_[valid_data_so_far_idx_++] = data;
        } else {
          valid_data_so_far_idx_ = 0;
        }
        break;
      }

      case DeserializeFsmState::kLenLsb: {
        data_len_this_time_ = data;
        valid_data_so_far_[valid_data_so_far_idx_++] = data;
        deserialize_fsm_state_ = DeserializeFsmState::kLenMsb;
        break;
      }

      case DeserializeFsmState::kLenMsb: {
        data_len_this_time_ |= (data << 8);
        valid_data_so_far_[valid_data_so_far_idx_++] = data;

        if (data_len_this_time_ < (kRefProtocolFrameMaxLen - kRefProtocolAllMetadataLen)) {
          deserialize_fsm_state_ = DeserializeFsmState::kSeq;
        } else {
          deserialize_fsm_state_ = DeserializeFsmState::kSof;
          valid_data_so_far_idx_ = 0;
        }
        break;
      }

      case DeserializeFsmState::kSeq: {
        valid_data_so_far_[valid_data_so_far_idx_++] = data;
        deserialize_fsm_state_ = DeserializeFsmState::kCrc8;
        break;
      }

      case DeserializeFsmState::kCrc8: {
        valid_data_so_far_[valid_data_so_far_idx_++] = data;

        if (valid_data_so_far_idx_ == kRefProtocolHeaderLen) {
          if (modules::algorithm::Crc8(valid_data_so_far_.data(), kRefProtocolHeaderLen - 1,
                                       modules::algorithm::CRC8_INIT) == valid_data_so_far_[4]) {
            deserialize_fsm_state_ = DeserializeFsmState::kCrc16;
          } else {
            deserialize_fsm_state_ = DeserializeFsmState::kSof;
            valid_data_so_far_idx_ = 0;
          }
        }
        break;
      }

      case DeserializeFsmState::kCrc16: {
        if (valid_data_so_far_idx_ < (kRefProtocolAllMetadataLen + data_len_this_time_)) {
          valid_data_so_far_[valid_data_so_far_idx_++] = data;
        }
        if (valid_data_so_far_idx_ >= (kRefProtocolAllMetadataLen + data_len_this_time_)) {
          deserialize_fsm_state_ = DeserializeFsmState::kSof;
          valid_data_so_far_idx_ = 0;
          crc16_this_time_ = (valid_data_so_far_[kRefProtocolAllMetadataLen + data_len_this_time_ - 1] << 8) |
                             valid_data_so_far_[kRefProtocolAllMetadataLen + data_len_this_time_ - 2];

          if (modules::algorithm::Crc16(valid_data_so_far_.data(), kRefProtocolAllMetadataLen + data_len_this_time_ - 2,
                                        modules::algorithm::CRC16_INIT) == crc16_this_time_) {
            cmdid_this_time_ = (valid_data_so_far_[6] << 8) | valid_data_so_far_[5];

            // 整包接收完+校验通过，把数据拷贝到反序列化缓冲区对应的结构体中
            memcpy((u8*)(&deserialize_buffer_) + referee_protocol_memory_map<revision>.at(cmdid_this_time_),
                   valid_data_so_far_.data() + kRefProtocolHeaderLen + kRefProtocolCmdIdLen, data_len_this_time_);
          }
        }
        break;
      }

      default: {
        deserialize_fsm_state_ = DeserializeFsmState::kSof;
        valid_data_so_far_idx_ = 0;
        break;
      }
    }
  }

  const RefereeProtocol<revision>& data() const { return deserialize_buffer_; }

 private:
  RefereeProtocol<revision> deserialize_buffer_;
  std::array<u8, kRefProtocolFrameMaxLen> valid_data_so_far_;
  usize valid_data_so_far_idx_{0};
  usize data_len_this_time_;
  usize cmdid_this_time_;
  u16 crc16_this_time_;
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_REFEREE_REFEREE_HPP
