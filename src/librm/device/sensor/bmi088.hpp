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
 * @file    librm/device/sensor/bmi088.hpp
 * @brief   BMI088 IMU
 */

#ifndef LIBRM_DEVICE_SENSOR_BMI088_HPP
#define LIBRM_DEVICE_SENSOR_BMI088_HPP

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_SPI_MODULE_ENABLED)

#include "librm/core/typedefs.hpp"
#include "librm/hal/stm32/spi_device.hpp"

namespace rm::device {

/**
 * @brief BMI088传感器状态
 */
enum class BMI088Status : u8 {
  NO_ERROR = 0x00,
  ACC_PWR_CTRL_ERROR = 0x01,
  ACC_PWR_CONF_ERROR = 0x02,
  ACC_CONF_ERROR = 0x03,
  ACC_SELF_TEST_ERROR = 0x04,
  ACC_RANGE_ERROR = 0x05,
  INT1_IO_CTRL_ERROR = 0x06,
  INT_MAP_DATA_ERROR = 0x07,
  GYRO_RANGE_ERROR = 0x08,
  GYRO_BANDWIDTH_ERROR = 0x09,
  GYRO_LPM1_ERROR = 0x0A,
  GYRO_CTRL_ERROR = 0x0B,
  GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
  GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

  SELF_TEST_ACCEL_ERROR = 0x80,
  SELF_TEST_GYRO_ERROR = 0x40,
  NO_SENSOR = 0xFF,
};

/**
 * @brief BMI088陀螺仪量程
 */
enum class BMI088GyroRange : u8 {
  GYRO_2000 = 0u,
  GYRO_1000 = 1u,
  GYRO_500 = 2u,
  GYRO_250 = 3u,
  GYRO_125 = 4u,
};

/**
 * @brief BMI088加速度计量程
 */
enum class BMI088AccelRange : u8 {
  ACC_RANGE_3G = 0u,
  ACC_RANGE_6G = 1u,
  ACC_RANGE_12G = 2u,
  ACC_RANGE_24G = 3u,
};

/**
 * @brief BMI088 IMU
 */
class BMI088 {
 public:
  BMI088() = delete;
  ~BMI088() = default;
  BMI088(SPI_HandleTypeDef &hspi, GPIO_TypeDef *cs1_accel_gpio_port, u16 cs1_accel_pin,
         GPIO_TypeDef *cs1_gyro_gpio_port, u16 cs1_gyro_pin);

  // no copy
  BMI088(const BMI088 &) = delete;
  BMI088 &operator=(const BMI088 &) = delete;

  void Update();
  [[nodiscard]] f32 temperature() const;
  [[nodiscard]] BMI088Status status() const;
  [[nodiscard]] f32 gyro_x() const;
  [[nodiscard]] f32 gyro_y() const;
  [[nodiscard]] f32 gyro_z() const;
  [[nodiscard]] f32 accel_x() const;
  [[nodiscard]] f32 accel_y() const;
  [[nodiscard]] f32 accel_z() const;

 private:
  void InitAccelerometer();
  void InitGyroscope();

  hal::stm32::SpiDevice accel_device_;
  hal::stm32::SpiDevice gyro_device_;

  BMI088Status status_{BMI088Status::NO_SENSOR};
  BMI088GyroRange gyro_range_{BMI088GyroRange::GYRO_2000};
  BMI088AccelRange accel_range_{BMI088AccelRange::ACC_RANGE_3G};

  f32 gyro_[3]{0};
  f32 accel_[3]{0};
  f32 temperature_{};
};
}  // namespace rm::device

#endif  // HAL_SPI_MODULE_ENABLED

#endif  // LIBRM_DEVICE_SENSOR_BMI088_H