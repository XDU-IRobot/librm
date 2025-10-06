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
 * @file  librm/hal/stm32/spi_device.cc
 * @brief SPI设备抽象
 * @todo  实现不够通用，有改进空间
 */

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_SPI_MODULE_ENABLED) && defined(HAL_GPIO_MODULE_ENABLED)

#include "spi_device.hpp"

namespace rm::hal::stm32 {
/**
 * @param hspi          SPI外设句柄
 * @param cs_gpio_port  片选引脚所在GPIO端口
 * @param cs_pin        片选引脚编号
 */
SpiDevice::SpiDevice(SPI_HandleTypeDef &hspi, GPIO_TypeDef *cs_gpio_port, u16 cs_pin)
    : hspi_(&hspi), cs_gpio_port_(cs_gpio_port), cs_pin_(cs_pin) {
  HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_SET);
}

/**
 * @brief 读写一个字节
 * @param tx_data 待发送的数据
 */
void SpiDevice::ReadWriteByte(u8 tx_data) {
  HAL_SPI_TransmitReceive(this->hspi_, &tx_data, &this->single_byte_buffer_, 1, 1000);
}

/**
 * @brief 读写多个字节
 * @param reg 寄存器地址
 * @param len 待读取的字节数
 */
void SpiDevice::ReadWriteBytes(const u8 reg, const u8 len) {
  this->ReadWriteByte(reg | 0x80);

  for (int i = 0; i < len; i++) {
    this->ReadWriteByte(0x55);
    this->buffer_[i] = this->single_byte_buffer_;
  }
}

/**
 * @brief 写一个字节
 * @param reg   寄存器地址
 * @param data  待写入的数据
 */
void SpiDevice::WriteByte(u8 reg, u8 data) {
  HAL_GPIO_WritePin(this->cs_gpio_port_, this->cs_pin_, GPIO_PIN_RESET);
  this->ReadWriteByte(reg);
  this->ReadWriteByte(data);
  HAL_GPIO_WritePin(this->cs_gpio_port_, this->cs_pin_, GPIO_PIN_SET);
}

/**
 * @brief   获取单字节缓冲区的值
 * @return  单字节缓冲区的值
 */
u8 SpiDevice::single_byte_buffer() const { return this->single_byte_buffer_; }

/**
 * @brief   获取缓冲区指针
 * @return  缓冲区指针
 */
const u8 *SpiDevice::buffer() const { return this->buffer_; }

/**
 * @brief 读一个字节
 * @param reg 寄存器地址
 */
void SpiDevice::ReadByte(u8 reg) {
  HAL_GPIO_WritePin(this->cs_gpio_port_, this->cs_pin_, GPIO_PIN_RESET);
  this->ReadWriteByte(reg | 0x80);
  this->ReadWriteByte(0x55);
  HAL_GPIO_WritePin(this->cs_gpio_port_, this->cs_pin_, GPIO_PIN_SET);
}

/**
 * @brief 读多个字节
 * @param reg 寄存器地址
 * @param len 待读取的字节数
 */
void SpiDevice::ReadBytes(u8 reg, u8 len) {
  HAL_GPIO_WritePin(this->cs_gpio_port_, this->cs_pin_, GPIO_PIN_RESET);
  this->ReadWriteBytes(reg, len);
  HAL_GPIO_WritePin(this->cs_gpio_port_, this->cs_pin_, GPIO_PIN_SET);
}
}  // namespace rm::hal::stm32

#endif