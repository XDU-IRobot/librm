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
 * @file    librm/device/sensor/jy_me02_can.hpp
 * @brief   维特智能JY-ME02-CAN角度编码器，CAN协议版本
 */

#ifndef LIBRM_DEVICE_SENSOR_JY_ME02_CAN_HPP
#define LIBRM_DEVICE_SENSOR_JY_ME02_CAN_HPP

namespace rm::device {

/**
 * @brief 维特智能JY-ME02-CAN角度编码器，CAN协议版本
 */
class JyMe02Can final : public CanDevice {
 public:
  ~JyMe02Can() override = default;
  JyMe02Can(JyMe02Can &&) noexcept = default;

  /**
   * @param can             所属CAN总线
   * @param rx_id           接收报文ID
   * @param sample_time_s   角速度采样时间，在上位机中设置
   */
  JyMe02Can(hal::CanInterface &can, u16 rx_id, f32 sample_time_s = 0.2f)
      : CanDevice(can, rx_id), sample_time_s_(sample_time_s) {}

  void RxCallback(const hal::CanFrame *msg) override {
    /**
     * 数据格式示例: 55 55 aa bb cc dd ee ff
     * - 角度寄存器 = (0xbb << 8) | 0xaa
     * - 角速度寄存器 = (0xdd << 8) | 0xcc
     * - 转数寄存器 = (0xff << 8) | 0xee
     *
     * 角度(°) = reg * 360 / 32768
     * 角速度(°/s) = reg * 360 / 32768 / sample_time_s (默认 sample_time_s = 0.1s)
     * */
    ReportStatus(kOk);
    if (msg->data[0] == 0x55 && msg->data[1] == 0x55) {
      // 按照给定字节序，msg->data: [0]=0x55 [1]=0x55 [2]=aa [3]=bb [4]=cc [5]=dd [6]=ee [7]=ff
      const u16 angle_raw = static_cast<u16>((msg->data[3] << 8) | msg->data[2]);
      const u16 speed_raw = static_cast<u16>((msg->data[5] << 8) | msg->data[4]);
      rotations_ = static_cast<u16>((msg->data[7] << 8) | msg->data[6]);

      // 转换原始值为实际单位
      angle_deg_ = static_cast<f32>(angle_raw) * 360.f / 32768.f;
      angular_speed_dps_ = static_cast<f32>(speed_raw) * 360.f / 32768.f / sample_time_s_;
    } else {
    }
  }

  [[nodiscard]] f32 angle_deg() const { return angle_deg_; }
  [[nodiscard]] f32 angular_speed_dps() const { return angular_speed_dps_; }

  [[nodiscard]] u16 rotations() const { return rotations_; }

 private:
  f32 angle_deg_{};          ///< 角度(度)
  f32 angular_speed_dps_{};  ///< 角速度(°/s)
  u16 rotations_{};          ///< 圈数
  f32 sample_time_s_{};      ///< 角速度采样时间(秒)
};

}  // namespace rm::device

#endif