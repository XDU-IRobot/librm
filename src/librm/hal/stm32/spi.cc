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
 * @file  librm/hal/stm32/spi.h
 * @brief SPI类库
 */

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_SPI_MODULE_ENABLED)

#include "spi.hpp"

namespace rm::hal::stm32 {

Spi::Spi(SPI_HandleTypeDef &hspi, usize timeout_ms) : hspi_(&hspi), transmission_timeout_(timeout_ms) {}

/**
 * @brief 初始化SPI
 * @note CubeMX+HAL库已经初始化好SPI，所以这里什么都不用做
 */
void Spi::Begin() {}

/**
 * @brief 发送数据
 * @param data 发送数据指针
 * @param size 发送数据长度
 */
void Spi::Write(const u8 *data, usize size) {
  HAL_SPI_Transmit(this->hspi_, (uint8_t *)data, size, this->transmission_timeout_);
}

/**
 * @brief 读取数据
 * @param dst  读取缓冲区数组指针
 * @param size 读取长度
 */
void Spi::Read(u8 *dst, usize size) { HAL_SPI_Receive(this->hspi_, dst, size, this->transmission_timeout_); }

/**
 * @brief 全双工发送和接收一定长度的数据
 * @param tx_data   发送数据指针
 * @param rx_buffer 读取缓冲区数组指针
 * @param size      发送/接收数据长度
 */
void Spi::ReadWrite(const u8 *tx_data, u8 *rx_buffer, usize size) {
  HAL_SPI_TransmitReceive(this->hspi_, (uint8_t *)tx_data, rx_buffer, size, this->transmission_timeout_);
}

}  // namespace rm::hal::stm32

#endif