/**
 * @file    bsp/interface/bsp_can_interface.h
 * @brief   CAN外设接口
 */

#ifndef EC_LIB_BSP_INTERFACE_BSP_CAN_INTERFACE_H_
#define EC_LIB_BSP_INTERFACE_BSP_CAN_INTERFACE_H_

#include "hal_wrapper/hal.h"
#if defined(HAL_CAN_MODULE_ENABLED)

#include "can.h"

#include "modules/typedefs.h"

namespace irobot_ec::bsp {

struct CanRxMsg {
  uint8_t data[8];
  CAN_RxHeaderTypeDef header;
};

class CanInterface {
 public:
  virtual ~CanInterface() = default;

  virtual void InitCanFilter(CAN_HandleTypeDef *hcan, uint16_t id = 0, uint16_t mask = 0) = 0;
};

}  // namespace irobot_ec::bsp

#endif

#endif  // EC_LIB_BSP_INTERFACE_BSP_CAN_INTERFACE_H_
