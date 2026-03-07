/*
  Copyright (c) 2026 XDU-IRobot

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
 * @file  librm/device/referee/protocol.hpp
 * @brief 裁判系统串口协议
 */

#ifndef LIBRM_DEVICE_REFEREE_PROTOCOL_HPP
#define LIBRM_DEVICE_REFEREE_PROTOCOL_HPP

#include "librm/core/typedefs.hpp"

namespace rm::device {

constexpr u8 kRefProtocolHeaderSof = 0xa5;
constexpr int kRefProtocolFrameMaxLen = 128;
constexpr int kRefProtocolHeaderLen = 5;
constexpr int kRefProtocolCmdIdLen = 2;
constexpr int kRefProtocolCrc16Len = 2;
constexpr int kRefProtocolAllMetadataLen = kRefProtocolHeaderLen + kRefProtocolCmdIdLen + kRefProtocolCrc16Len;

/**
 * @brief 裁判系统协议版本
 */
enum class RefereeRevision {
  kV164,     ///< V1.6.4, 2024-7-15
  kV170,     ///< V1.7.0, 2024-12-25
  kNewV110,  ///< 新通信协议V1.1.0, 2025-12-17
  kNewV120,  ///< 新通信协议V1.2.0, 2026-2-9
};

/**
 * @brief 裁判系统协议命令码
 */
template <RefereeRevision revision>
struct RefereeCmdId {};

/**
 * @brief 裁判系统协议数据结构
 */
template <RefereeRevision revision>
struct RefereeProtocol {};

/**
 * @brief 裁判系统协议内存映射，记录了每个命令码对应的数据结构在 RefereeProtocol 中的偏移量
 */
template <RefereeRevision revision>
struct RefereeProtocolMemoryMap {};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_REFEREE_PROTOCOL_HPP
