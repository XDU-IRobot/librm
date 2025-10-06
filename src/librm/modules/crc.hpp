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
 * @file  librm/modules/crc.hpp
 * @brief crc8/crc16
 */

#ifndef LIBRM_MODULES_CRC_HPP
#define LIBRM_MODULES_CRC_HPP

#include <string>

#include "librm/core/typedefs.hpp"

namespace rm::modules {

constexpr u8 CRC8_INIT = 0xff;
constexpr u16 CRC16_INIT = 0xffff;
constexpr u32 CRC32_INIT = 0xffffffff;

u8 Crc8(const u8 *input, usize len, u8 init);
u8 Crc8(std::string_view input, u8 init);
u8 Crc8(const std::string &input, u8 init);

u16 Crc16(const u8 *input, usize len, u16 init);
u16 Crc16(std::string_view input, u16 init);
u16 Crc16(const std::string &input, u16 init);

u32 Crc32(const u32 *input, usize len, u32 init);
u32 Crc32(std::string_view input, u32 init);
u32 Crc32(const std::string &input, u32 init);

u16 CrcCcitt(const u8 *input, usize len, u16 init);
u16 CrcCcitt(std::string_view input, u16 init);
u16 CrcCcitt(const std::string &input, u16 init);

}  // namespace rm::modules

#endif  // LIBRM_MODULES_CRC_H