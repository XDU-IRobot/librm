/*
  Copyright (c) 2024 XDU-IRobot

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
 * @file  librm/device/remote/vt03.hpp
 * @brief VT03图传发送端的遥控器数据解析器
 */

#ifndef LIBRM_DEVICE_REMOTE_VT03_HPP
#define LIBRM_DEVICE_REMOTE_VT03_HPP

#include <cstring>
#include <array>

#include "librm/modules/algorithm/utils.hpp"
#include "librm/modules/algorithm/crc.h"

namespace rm::device {

/**
 * @brief VT03图传发送端的遥控器数据解析器
 */
class VT03 {
 private:
  enum class DeserializeFsmState {
    kSof0,
    kSof1,
    kCrc16,
  } deserialize_fsm_state_{DeserializeFsmState::kSof0};

  static constexpr u8 kSof[2] = {0xa9, 0x53};
  static constexpr usize kFrameLength = 21;  // bytes

 public:
  /**
   * @brief 档位切换开关位置
   */
  enum class SwitchPosition : usize {
    C = 0,
    N = 1,
    S = 2,
  };

  /**
   * @brief 键盘按键
   */
  enum class KeyboardKey : u16 {
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

 public:
  VT03() = default;

  void operator<<(u8 data) {
    switch (deserialize_fsm_state_) {
      case DeserializeFsmState::kSof0: {
        if (data == kSof[0]) {
          deserialize_fsm_state_ = DeserializeFsmState::kSof1;
          valid_data_so_far_[valid_data_so_far_idx_++] = data;
        } else {
          valid_data_so_far_idx_ = 0;
        }
        break;
      }

      case DeserializeFsmState::kSof1: {
        if (data == kSof[1]) {
          deserialize_fsm_state_ = DeserializeFsmState::kCrc16;
          valid_data_so_far_[valid_data_so_far_idx_++] = data;
        } else {
          valid_data_so_far_idx_ = 0;
        }
        break;
      }

      case DeserializeFsmState::kCrc16: {
        if (valid_data_so_far_idx_ < kFrameLength) {
          valid_data_so_far_[valid_data_so_far_idx_++] = data;
          break;
        }
        deserialize_fsm_state_ = DeserializeFsmState::kSof0;
        valid_data_so_far_idx_ = 0;
        crc16_this_time_ = (valid_data_so_far_[kFrameLength - 1] << 8) | valid_data_so_far_[kFrameLength - 2];

        if (modules::algorithm::Crc16(valid_data_so_far_.data(), kFrameLength - 2, modules::algorithm::CRC16_INIT) ==
            crc16_this_time_) {
          // 整包接收完+校验通过，开始解析数据
          using modules::algorithm::utils::Map;
          std::memcpy(&raw_payload_data_, valid_data_so_far_.data(), sizeof(raw_payload_data_));
          data_.right_x = Map(raw_payload_data_.ch_0, 364, 1684, -1.0f, 1.0f);
          data_.right_y = Map(raw_payload_data_.ch_1, 364, 1684, -1.0f, 1.0f);
          data_.left_x = Map(raw_payload_data_.ch_2, 364, 1684, -1.0f, 1.0f);
          data_.left_y = Map(raw_payload_data_.ch_3, 364, 1684, -1.0f, 1.0f);
          data_.switch_position = static_cast<SwitchPosition>(raw_payload_data_.mode_sw);
          data_.pause_button = raw_payload_data_.pause;
          data_.left_button = raw_payload_data_.fn_1;
          data_.right_button = raw_payload_data_.fn_2;
          data_.dial = Map(raw_payload_data_.wheel, 364, 1684, -1.0f, 1.0f);
          data_.trigger = raw_payload_data_.trigger;
          data_.mouse_x = raw_payload_data_.mouse_x;
          data_.mouse_y = raw_payload_data_.mouse_y;
          data_.mouse_z = raw_payload_data_.mouse_z;
          data_.mouse_button_left = raw_payload_data_.mouse_left;
          data_.mouse_button_right = raw_payload_data_.mouse_right;
          data_.mouse_button_middle = raw_payload_data_.mouse_middle;
          data_.keyboard_key = raw_payload_data_.key;
        }  // else drop this packet
        break;
      }

      default: {
        deserialize_fsm_state_ = DeserializeFsmState::kSof0;
        valid_data_so_far_idx_ = 0;
        break;
      }
    }
  }

  const auto &data() const { return data_; }

 private:
  /**
   * @brief VT03遥控器数据包的原始数据结构，从示例代码里抄的：
   * @note  https://rm-static.djicdn.com/tem/17348/Example_Code_for_Data_Frame_and_Validation.c
   */
  struct __attribute__((__packed__)) {
    u8 sof_1;
    u8 sof_2;
    u64 ch_0 : 11;
    u64 ch_1 : 11;
    u64 ch_2 : 11;
    u64 ch_3 : 11;
    u64 mode_sw : 2;
    u64 pause : 1;
    u64 fn_1 : 1;
    u64 fn_2 : 1;
    u64 wheel : 11;
    u64 trigger : 1;

    i16 mouse_x;
    i16 mouse_y;
    i16 mouse_z;
    u8 mouse_left : 2;
    u8 mouse_right : 2;
    u8 mouse_middle : 2;
    u16 key;
    u16 crc16;
  } raw_payload_data_;
  struct {
    f32 right_x;                     ///< 右摇杆水平方向位置，归一化到[-1, 1]，左负右正
    f32 right_y;                     ///< 右摇杆垂直方向位置，归一化到[-1, 1]，左负右正
    f32 left_x;                      ///< 左摇杆水平方向位置，归一化到[-1, 1]，左负右正
    f32 left_y;                      //< 左摇杆垂直方向位置，归一化到[-1, 1]，左负右正
    SwitchPosition switch_position;  ///< 档位切换开关位置
    bool pause_button;               ///< 暂停键
    bool left_button;                ///< 左边的按钮
    bool right_button;               ///< 右边的按钮
    f32 dial;                        ///< 拨轮位置，归一化到[-1, 1]，左负右正
    bool trigger;                    ///< 扳机键
    i16 mouse_x;                     ///< 鼠标左右移动速度，范围[-32768, 32767]，左负右正
    i16 mouse_y;                     ///< 鼠标前后移动速度，范围[-32768, 32767]，左负右正
    i16 mouse_z;                     ///< 鼠标滚轮滚动速度，范围[-32768, 32767]，左负右正
    bool mouse_button_left;          ///< 鼠标左键
    bool mouse_button_right;         ///< 鼠标右键
    bool mouse_button_middle;        ///< 鼠标中键
    u16 keyboard_key;  ///< 键盘按键，每一位代表一个键，0为未按下，1为按下，bit定义见VT03::KeyboardKey
  } data_{};
  std::array<u8, kFrameLength * 2> valid_data_so_far_;
  usize valid_data_so_far_idx_{0};
  u16 crc16_this_time_;
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_REMOTE_VT03_HPP
