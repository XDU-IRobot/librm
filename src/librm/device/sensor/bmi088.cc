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
 * @file    librm/device/sensor/bmi088.cc
 * @brief   BMI088传感器类
 * @todo    DMA传输+硬件片选(maybe)
 */

#include "librm/hal/stm32/hal.h"
#if defined(HAL_SPI_MODULE_ENABLED)

#include "bmi088.h"

#include <vector>

#include "librm/core/time.hpp"

namespace rm::device {
#include "bmi088_const.hpp"

/**
 * @brief BMI088加速度计初始化序列
 * @note  顺序：寄存器地址，写入值，错误码
 */
const static std::vector<std::vector<u8>> BMI088_ACCEL_INIT_SEQUENCE = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, static_cast<u8>(BMI088Status::ACC_PWR_CTRL_ERROR)},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, static_cast<u8>(BMI088Status::ACC_PWR_CONF_ERROR)},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set,
     static_cast<u8>(BMI088Status::ACC_CONF_ERROR)},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, static_cast<u8>(BMI088Status::ACC_RANGE_ERROR)},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW,
     static_cast<u8>(BMI088Status::INT1_IO_CTRL_ERROR)},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, static_cast<u8>(BMI088Status::INT_MAP_DATA_ERROR)}};

/**
 * @brief BMI088陀螺仪初始化序列
 * @note  顺序：寄存器地址，写入值，错误码
 */
const static std::vector<std::vector<u8>> BMI088_GYRO_INIT_SEQUENCE = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, static_cast<u8>(BMI088Status::GYRO_RANGE_ERROR)},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_SET,
     static_cast<u8>(BMI088Status::GYRO_BANDWIDTH_ERROR)},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, static_cast<u8>(BMI088Status::GYRO_LPM1_ERROR)},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, static_cast<u8>(BMI088Status::GYRO_CTRL_ERROR)},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
     static_cast<u8>(BMI088Status::GYRO_INT3_INT4_IO_CONF_ERROR)},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3,
     static_cast<u8>(BMI088Status::GYRO_INT3_INT4_IO_MAP_ERROR)}};

/**
 * @param hspi                  SPI外设句柄
 * @param cs1_accel_gpio_port   加速度计片选引脚所在GPIO端口
 * @param cs1_accel_pin         加速度计片选引脚编号
 * @param cs1_gyro_gpio_port    陀螺仪片选引脚所在GPIO端口
 * @param cs1_gyro_pin          陀螺仪片选引脚编号
 */
BMI088::BMI088(SPI_HandleTypeDef &hspi, GPIO_TypeDef *cs1_accel_gpio_port, u16 cs1_accel_pin,
               GPIO_TypeDef *cs1_gyro_gpio_port, u16 cs1_gyro_pin)
    : accel_device_(hspi, cs1_accel_gpio_port, cs1_accel_pin), gyro_device_(hspi, cs1_gyro_gpio_port, cs1_gyro_pin) {
  this->InitGyroscope();
  this->InitAccelerometer();
}

/**
 * @brief 初始化加速度计
 */
void BMI088::InitAccelerometer() {
  // 检查通信是否正常
  this->accel_device_.ReadBytes(BMI088_ACC_CHIP_ID, 2);
  core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);
  this->accel_device_.ReadBytes(BMI088_ACC_CHIP_ID, 2);
  core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);

  // soft reset一次
  this->accel_device_.WriteByte(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
  core::time::Sleep(BMI088_LONG_DELAY_TIME);

  // 再次检查通信是否正常
  this->accel_device_.ReadBytes(BMI088_ACC_CHIP_ID, 2);
  core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);
  this->accel_device_.ReadBytes(BMI088_ACC_CHIP_ID, 2);
  core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);

  // 检查"who am I"寄存器值是否正确
  if (accel_device_.single_byte_buffer() != BMI088_ACC_CHIP_ID_VALUE) {
    this->status_ = BMI088Status::NO_SENSOR;
  }

  // 发送初始化序列，有错误则设置status为对应错误码
  for (const auto &operation : BMI088_ACCEL_INIT_SEQUENCE) {
    this->accel_device_.WriteByte(operation[0], operation[1]);
    core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);
    this->accel_device_.ReadBytes(operation[0], 2);
    core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);

    if (accel_device_.single_byte_buffer() != operation[1]) {
      this->status_ = (BMI088Status)operation[2];
    }
  }
}

/**
 * @brief 初始化陀螺仪
 */
void BMI088::InitGyroscope() {
  // 检查通信是否正常
  this->gyro_device_.ReadByte(BMI088_GYRO_CHIP_ID);
  core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);
  this->gyro_device_.ReadByte(BMI088_GYRO_CHIP_ID);
  core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);

  // soft reset一次
  this->gyro_device_.WriteByte(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
  core::time::Sleep(BMI088_LONG_DELAY_TIME);

  // 再次检查通信是否正常
  this->gyro_device_.ReadByte(BMI088_GYRO_CHIP_ID);
  core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);
  this->gyro_device_.ReadByte(BMI088_GYRO_CHIP_ID);
  core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);

  // 检查"who am I"寄存器值是否正确
  if (gyro_device_.single_byte_buffer() != BMI088_GYRO_CHIP_ID_VALUE) {
    this->status_ = BMI088Status::NO_SENSOR;
  }

  // 发送初始化序列，有错误则设置status为对应错误码
  for (const auto &operation : BMI088_GYRO_INIT_SEQUENCE) {
    this->gyro_device_.WriteByte(operation[0], operation[1]);
    core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);
    this->gyro_device_.ReadByte(operation[0]);
    core::time::Sleep(BMI088_COM_WAIT_SENSOR_TIME);

    if (gyro_device_.single_byte_buffer() != operation[1]) {
      this->status_ = (BMI088Status)operation[2];
    }
  }
}

/**
 * @brief 更新传感器数据
 */
void BMI088::Update() {
  const u8 *accel_buf = this->accel_device_.buffer();
  const u8 *gyro_buf = this->gyro_device_.buffer();

  this->accel_device_.ReadBytes(BMI088_ACCEL_XOUT_L, 7);
  this->accel_[0] = ((i16)(accel_buf[2] << 8) | accel_buf[1]) * BMI088_ACCEL_SENSITIVITY[(u8)this->accel_range_];
  this->accel_[1] = ((i16)(accel_buf[4] << 8) | accel_buf[3]) * BMI088_ACCEL_SENSITIVITY[(u8)this->accel_range_];
  this->accel_[2] = ((i16)(accel_buf[6] << 8) | accel_buf[5]) * BMI088_ACCEL_SENSITIVITY[(u8)this->accel_range_];

  this->gyro_device_.ReadBytes(BMI088_GYRO_X_L, 6);
  this->gyro_[0] = ((i16)(gyro_buf[1] << 8) | gyro_buf[0]) * BMI088_GYRO_SENSITIVITY[(u8)this->gyro_range_];
  this->gyro_[1] = ((i16)(gyro_buf[3] << 8) | gyro_buf[2]) * BMI088_GYRO_SENSITIVITY[(u8)this->gyro_range_];
  this->gyro_[2] = ((i16)(gyro_buf[5] << 8) | gyro_buf[4]) * BMI088_GYRO_SENSITIVITY[(u8)this->gyro_range_];

  this->accel_device_.ReadBytes(BMI088_TEMP_M, 3);

  auto bmi088_raw_temp = (i16)(accel_buf[1] << 3 | accel_buf[2] >> 5);
  if (bmi088_raw_temp > 1023) {
    bmi088_raw_temp -= 2048;
  }
  this->temperature_ = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/**
 * @brief   获取传感器状态
 * @return  传感器状态
 */
BMI088Status BMI088::status() const { return this->status_; }

/**
 * @brief   获取传感器温度
 * @return  传感器温度
 */
f32 BMI088::temperature() const { return this->temperature_; }

/**
 * @return  陀螺仪X轴数据(rad/s)
 */
f32 BMI088::gyro_x() const { return this->gyro_[0]; }

/**
 * @return  陀螺仪Y轴数据(rad/s)
 */
f32 BMI088::gyro_y() const { return this->gyro_[1]; }

/**
 * @return  陀螺仪Z轴数据(rad/s)
 */
f32 BMI088::gyro_z() const { return this->gyro_[2]; }

/**
 * @return  加速度计X轴数据(m/s^2)
 */
f32 BMI088::accel_x() const { return this->accel_[0]; }

/**
 * @return  加速度计Y轴数据(m/s^2)
 */
f32 BMI088::accel_y() const { return this->accel_[1]; }

/**
 * @return  加速度计Z轴数据(m/s^2)
 */
f32 BMI088::accel_z() const { return this->accel_[2]; }

}  // namespace rm::device

#endif