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
 * @file  librm/device/sensor/stp23l.hpp
 * @brief STP23L激光测距模块
 */

#ifndef LIBRM_DEVICE_SENSOR_STP23L_HPP
#define LIBRM_DEVICE_SENSOR_STP23L_HPP

#include "librm/core/typedefs.hpp"
#include "librm/hal/serial.hpp"
#include "librm/device/device.hpp"

namespace rm::device {

/**
 * @brief STP23L激光测距传感器
 * @note  串口参数：230400, 8N1
 */
class STP23L : public Device {
 public:
  /**
   * @brief STP23L单点测量数据
   */
  struct PointData {
    i16 distance;   ///< 测量目标距离单位mm
    u16 noise;      ///< 当前测量环境下的外部环境噪声，越大说明噪声越大
    u32 peak;       ///< 测量目标反射回的光强度
    u8 confidence;  ///< 由环境噪声和接收强度信息融合后的测量点的可信度
    u32 intg;       ///< 当前传感器测量的积分次数
    i16 reftof;     ///< 测量芯片内部温度变化表征值，只是一个温度变化量无法与真实温度对应
  };

  STP23L() = delete;
  explicit STP23L(hal::SerialInterface &serial);

  void Begin() const;

  [[nodiscard]] i16 distance() const;     ///< 获取平均距离值，单位mm
  [[nodiscard]] u16 noise() const;        ///< 获取平均噪声量
  [[nodiscard]] u32 peak() const;         ///< 获取平均反射强度
  [[nodiscard]] u8 confidence() const;    ///< 获取平均置信度
  [[nodiscard]] u32 intg() const;         ///< 获取平均积分次数
  [[nodiscard]] i16 reftof() const;       ///< 获取平均温度补偿值
  [[nodiscard]] u16 frame_count() const;  ///< 获取成功接收的帧数

  /**
   * @brief 获取原始测量点数据
   * @param index 点的索引，范围0-11（一帧包含12个点）
   * @return 原始测量点数据
   */
  [[nodiscard]] const PointData &GetRawPoint(u8 index) const;

 private:
  /**
   * @brief 串口接收回调函数
   * @param data   接收到的数据
   * @param rx_len 接收到的数据长度
   */
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

  /**
   * @brief 逐字节处理接收到的数据
   * @param byte 接收到的字节
   */
  void ProcessByte(u8 byte);

  /**
   * @brief 从数据缓冲区解析出12个点的数据和时间戳
   */
  void ParseDataBuffer();

  /**
   * @brief 数据处理，对12个点进行平均
   */
  void ProcessData();

  hal::SerialInterface *serial_;

  // 协议常量
  static constexpr u8 kHeader = 0xAA;           ///< 帧头
  static constexpr u8 kDeviceAddress = 0x00;    ///< 设备地址
  static constexpr u8 kChunkOffset = 0x00;      ///< 偏移地址偏移
  static constexpr usize kPointsPerFrame = 12;  ///< 每帧点数

  /**
   * @brief STP23L数据包类型
   */
  enum class STP23LPacketType : u8 {
    kGetDistance = 0x02,  ///< 获取测量数据包
    kResetSystem = 0x0D,  ///< 复位包
    kStop = 0x0F,         ///< 停止测量数据流包
    kAck = 0x10,          ///< 应答数据包
    kVersion = 0x14,      ///< 获取版本信息包
  };

  /**
   * @brief 字节流解析状态机的状态列表
   */
  enum ParseState {
    kWaitHeader,    ///< 等待帧头（4个0xAA）
    kDeviceAddr,    ///< 设备地址
    kPacketType,    ///< 数据包类型
    kChunkOffset1,  ///< 偏移地址低位
    kChunkOffset2,  ///< 偏移地址高位
    kDataLenLow,    ///< 数据长度低位
    kDataLenHigh,   ///< 数据长度高位
    kData,          ///< 数据段（包括点数据和时间戳）
    kChecksum,      ///< 校验和
  };

  ParseState parse_state_{kWaitHeader};
  u8 header_count_{0};                 ///< 帧头计数（需要4个连续的0xAA）
  u8 crc_{0};                          ///< 校验和累加值
  u16 data_len_{0};                    ///< 数据段长度
  usize data_idx_{0};                  ///< 数据段当前索引
  std::array<u8, 196> data_buffer_{};  ///< 数据缓冲区（12点*15字节+4字节时间戳=184字节）

  PointData raw_points_[kPointsPerFrame]{};  ///< 原始12个点的数据
  PointData averaged_data_{};                ///< 平均后的数据
  u16 frame_count_{0};                       ///< 成功接收的帧数
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_SENSOR_STP23L_HPP
