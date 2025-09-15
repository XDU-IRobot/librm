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
 * @file    librm/device/sensor/ist8310.cc
 * @brief   IST8310磁力计类
 */

#include "librm/hal/stm32/hal.h"
#if defined(HAL_I2C_MODULE_ENABLED)

#include "ist8310.h"

#include <vector>

#include "librm/core/time.hpp"

namespace rm::device {
#include "ist8310_const.hpp"

// 初始化序列
// 第一列:寄存器地址
// 第二列:需要写入的寄存器值
const static std::vector<std::vector<u8>> ist8310_init_sequence = {{0x0B, 0x08},  // 开启中断，并且设置低电平
                                                                   {0x41, 0x09},   // 平均采样两次
                                                                   {0x42, 0xC0},   // 必须是0xC0
                                                                   {0x0A, 0x0B}};  // 200Hz输出频率

/**
 * @param hi2c I2C外设句柄
 */
IST8310::IST8310(I2C_HandleTypeDef &hi2c, GPIO_TypeDef *rst_port, u16 rst_pin)
    : rm::hal::stm32::I2cDevice(hi2c, IST8310_I2C_ADDRESS), rst_port_(rst_port), rst_pin_(rst_pin) {
  // 开机复位
  this->Reset();

  // 检查"who am i"寄存器的值是否正确
  this->Read(IST8310_WHO_AM_I, 1);
  if (this->buffer_[0] != IST8310_WHO_AM_I_VALUE) {
    this->status_ = IST8310Status::NO_SENSOR;
  }

  // 发送初始化序列，有错误则设置status为对应错误码
  for (const auto &operation : ist8310_init_sequence) {
    this->Write(operation[0], const_cast<u8 *>(&operation[1]), 1);
    core::time::Sleep(IST8310_COMM_WAIT_TIME_SHORT);
    this->Read(operation[0], 1);
    core::time::Sleep(IST8310_COMM_WAIT_TIME_SHORT);
    if (this->buffer_[0] != operation[1]) {
      this->status_ = IST8310Status::SENSOR_ERROR;
    }
  }
  this->status_ = IST8310Status::NO_ERROR;
}

/**
 * @brief 重置传感器
 */
void IST8310::Reset() {
  HAL_GPIO_WritePin(this->rst_port_, this->rst_pin_, GPIO_PIN_RESET);
  core::time::Sleep(IST8310_COMM_WAIT_TIME_LONG);
  HAL_GPIO_WritePin(this->rst_port_, this->rst_pin_, GPIO_PIN_SET);
  core::time::Sleep(IST8310_COMM_WAIT_TIME_LONG);
}

/**
 * @brief 更新传感器数据
 */
void IST8310::Update() {
  this->Read(IST8310_DATA_OUT_X_L, 6);
  this->mag_[0] =
      static_cast<f32>(static_cast<int16_t>(this->buffer_[1] << 8 | this->buffer_[0])) * IST8310_SENSITIVITY;
  this->mag_[1] =
      static_cast<f32>(static_cast<int16_t>(this->buffer_[3] << 8 | this->buffer_[2])) * IST8310_SENSITIVITY;
  this->mag_[2] =
      static_cast<f32>(static_cast<int16_t>(this->buffer_[5] << 8 | this->buffer_[4])) * IST8310_SENSITIVITY;
}

/**
 * @return 传感器状态
 */
IST8310Status IST8310::status() const { return this->status_; }

/**
 * @return 磁力计X轴数据(uT)
 */
f32 IST8310::mag_x() const { return this->mag_[0]; }

/**
 * @return 磁力计Y轴数据(uT)
 */
f32 IST8310::mag_y() const { return this->mag_[1]; }

/**
 * @return 磁力计Z轴数据(uT)
 */
f32 IST8310::mag_z() const { return this->mag_[2]; }

}  // namespace rm::device

#endif
