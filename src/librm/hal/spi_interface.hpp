/*
  Copyright (c) 2025 XDU-IRobot

  Permission is hereby granted, free of u8ge, to any person obtaining a copy
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
 * @file    librm/hal/spi_interface.hpp
 * @brief   SPI的接口类
 */

#ifndef LIBRM_HAL_STM32_SPI_INTERFACE_HPP
#define LIBRM_HAL_STM32_SPI_INTERFACE_HPP

#include "librm/core/typedefs.hpp"

namespace rm::hal {

/**
 * @brief SPI接口类
 */
class SpiInterface {
 public:
  virtual ~SpiInterface() = default;

  /**
   * @brief 初始化SPI
   */
  virtual void Begin() = 0;

  /**
   * @brief 发送数据
   * @param data 发送数据指针
   * @param size 发送数据长度
   */
  virtual void Write(const u8 *data, usize size) = 0;

  /**
   * @brief 读取数据
   * @param dst  读取缓冲区数组指针
   * @param size 读取长度
   */
  virtual void Read(u8 *dst, usize size) = 0;

  /**
   * @brief 全双工发送和接收一定长度的数据
   * @param tx_data   发送数据指针
   * @param rx_buffer 读取缓冲区数组指针
   * @param size      发送/接收数据长度
   */
  virtual void ReadWrite(const u8 *tx_data, u8 *rx_buffer, usize size) = 0;
};

}  // namespace rm::hal

#endif  // LIBRM_HAL_STM32_SPI_INTERFACE_HPP
