/**
 * @file  hal_wrapper/spi_device.h
 * @brief SPI设备抽象
 * @todo  实现不够通用，有改进空间
 */

#ifndef EC_LIB_HAL_WRAPPER_HAL_SPI_H
#define EC_LIB_HAL_WRAPPER_HAL_SPI_H

#include "hal.h"
#if defined(HAL_SPI_MODULE_ENABLED) && defined(HAL_GPIO_MODULE_ENABLED)

namespace irobot_ec::hal {
/**
 * @brief SPI设备基类
 */
class SpiDevice {
 public:
  SpiDevice(SPI_HandleTypeDef &hspi, GPIO_TypeDef *cs_gpio_port, uint16_t cs_pin);
  SpiDevice() = delete;
  ~SpiDevice() = default;

  void ReadByte(uint8_t reg);
  void ReadBytes(uint8_t reg, uint8_t len);
  void WriteByte(uint8_t reg, uint8_t data);

  [[nodiscard]] uint8_t single_byte_buffer() const;
  [[nodiscard]] const uint8_t *buffer() const;

 protected:
  void ReadWriteByte(uint8_t tx_data);
  void ReadWriteBytes(uint8_t reg, uint8_t len);

  SPI_HandleTypeDef *hspi_;
  GPIO_TypeDef *cs_gpio_port_;
  uint16_t cs_pin_;
  uint8_t single_byte_buffer_{};
  uint8_t buffer_[8]{0};
};
}  // namespace irobot_ec::hal

#endif

#endif  // EC_LIB_HAL_WRAPPER_HAL_SPI_H