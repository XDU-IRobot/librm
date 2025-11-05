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
 * @file  librm/hal/stm32/fdcan.cc
 * @brief fdCAN类库
 */

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_FDCAN_MODULE_ENABLED)
#include "librm/hal/stm32/check_register_callbacks.hpp"

#include "fdcan.hpp"

#include <functional>
#include <algorithm>

#include <etl/unordered_map.h>

#include "librm/device/can_device.hpp"
#include "librm/core/exception.hpp"

/**
 * @brief 存储各个hfdcan句柄对应的接收回调函数指针的map
 */
static etl::unordered_map<FDCAN_HandleTypeDef *, std::function<void()>,
                          5>  // 一般来说不会用到超过5条CAN总线的STM32，这里设为5应该够了
    fn_cb_map;

/**
 * @brief  把std::function转换为函数指针
 * @param  fn   要转换的函数
 * @param  hfdcan  HAL库的FDCAN_HandleTypeDef
 * @return      转换后的函数指针
 * @note
 * 背景：因为要用面向对象的方式对外设进行封装，所以回调函数必须存在于类内。但是存在于类内就意味着这个回调函数多了一个this参数，
 * 而HAL库要求的回调函数并没有这个this参数。通过std::bind，可以生成一个参数列表里没有this指针的std::function对象，而std::function
 * 并不能直接强转成函数指针。借助这个函数，可以把std::function对象转换成函数指针。然后就可以把这个类内的回调函数传给HAL库了。
 */
static pFDCAN_RxFifo0CallbackTypeDef StdFunctionToCallbackFunctionPtr(std::function<void()> fn,
                                                                      FDCAN_HandleTypeDef *hfdcan) {
  fn_cb_map[hfdcan] = std::move(fn);
  return [](FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (fn_cb_map.find(hfdcan) != fn_cb_map.end()) {
      fn_cb_map[hfdcan]();
    }
  };
}

namespace rm::hal::stm32 {

/**
 * @param hfdcan HAL库的CAN_HandleTypeDef
 */
FdCan::FdCan(FDCAN_HandleTypeDef &hfdcan)
    : hfdcan_(&hfdcan),
      // 判断这个FDCAN句柄对应的外设是否被设置为FD模式
      fd_mode_{hfdcan.Init.FrameFormat == FDCAN_FRAME_FD_NO_BRS ||  //
               hfdcan.Init.FrameFormat == FDCAN_FRAME_FD_BRS} {}

/**
 * @brief 设置过滤器
 * @param id
 * @param mask
 */
void FdCan::SetFilter(u16 id, u16 mask) {
  FDCAN_FilterTypeDef can_filter_st;
  can_filter_st.IdType = FDCAN_STANDARD_ID;
  can_filter_st.FilterType = FDCAN_FILTER_MASK;
  can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  can_filter_st.FilterID1 = id;
  can_filter_st.FilterID2 = mask;
  can_filter_st.RxBufferIndex = 0;
  can_filter_st.IsCalibrationMsg = 0;
  if (reinterpret_cast<u32>(hfdcan_->Instance) == FDCAN1_BASE) {
    can_filter_st.FilterIndex = 0;
  }
  if (reinterpret_cast<u32>(hfdcan_->Instance) == FDCAN2_BASE) {
    can_filter_st.FilterIndex = 1;
  }
  if (reinterpret_cast<u32>(hfdcan_->Instance) == FDCAN3_BASE) {
    can_filter_st.FilterIndex = 2;
  }

  HAL_StatusTypeDef hal_status;
  hal_status = HAL_FDCAN_ConfigGlobalFilter(hfdcan_, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, DISABLE, DISABLE);
  if (hal_status != HAL_OK) {
    Throw(hal_error(hal_status));
  }
  hal_status = HAL_FDCAN_ConfigFilter(hfdcan_, &can_filter_st);
  if (hal_status != HAL_OK) {
    Throw(hal_error(hal_status));
  }
}

/**
 * @brief 立刻向总线上发送数据
 * @param id    标准帧ID
 * @param data  数据指针
 * @param size  数据长度
 */
void FdCan::Write(u16 id, const u8 *data, usize size) {
  if (size > 64) {
    Throw(std::runtime_error("CAN frame too long!"));
  }
  if (!fd_mode_ && size > 8) {
    Throw(std::runtime_error("Data is too long for a std CAN frame!"));
  }

  if (fd_mode_) {
    // 将字节长度转换为FDCAN DLC码
    uint32_t dlc_code;
    if (size <= 8) {
      dlc_code = size;  // 8字节以下DLC码和数据长度相同
    } else if (size <= 12) {
      dlc_code = FDCAN_DLC_BYTES_12;
    } else if (size <= 16) {
      dlc_code = FDCAN_DLC_BYTES_16;
    } else if (size <= 20) {
      dlc_code = FDCAN_DLC_BYTES_20;
    } else if (size <= 24) {
      dlc_code = FDCAN_DLC_BYTES_24;
    } else if (size <= 32) {
      dlc_code = FDCAN_DLC_BYTES_32;
    } else if (size <= 48) {
      dlc_code = FDCAN_DLC_BYTES_48;
    } else {
      dlc_code = FDCAN_DLC_BYTES_64;
    }
    hal_tx_header_.DataLength = dlc_code;
  } else {
    hal_tx_header_.DataLength = size;
  }

  hal_tx_header_.Identifier = id;

  HAL_StatusTypeDef hal_status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &hal_tx_header_, const_cast<u8 *>(data));
  if (hal_status != HAL_OK) {
    Throw(hal_error(hal_status));
  }
}

/**
 * @brief 启动CAN外设
 */
void FdCan::Begin() {
  // 如果这个FDCAN外设处于FD模式，就把发送头设置为FD格式
  if (fd_mode_) {
    hal_tx_header_.FDFormat = FDCAN_FD_CAN;
  } else {
    hal_tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
  }

  HAL_StatusTypeDef hal_status;
  hal_status = HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  if (hal_status != HAL_OK) {
    Throw(hal_error(hal_status));
  }
  hal_status = HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_BUS_OFF, 0);
  if (hal_status != HAL_OK) {
    Throw(hal_error(hal_status));
  }
  HAL_FDCAN_RegisterRxFifo0Callback(hfdcan_,
                                    StdFunctionToCallbackFunctionPtr([this] { Fifo0MsgPendingCallback(); }, hfdcan_));
  hal_status = HAL_FDCAN_Start(hfdcan_);
  if (hal_status != HAL_OK) {
    Throw(hal_error(hal_status));
  }
}

/**
 * @brief 停止CAN外设
 */
void FdCan::Stop() {
  HAL_StatusTypeDef hal_status = HAL_FDCAN_Stop(hfdcan_);
  if (hal_status != HAL_OK) {
    Throw(hal_error(hal_status));
  }
}

/**
 * @brief 利用Register callbacks机制，用这个函数替代HAL_CAN_RxFifo0MsgPendingCallback
 * @note  这个函数替代了HAL_CAN_RxFifo0MsgPendingCallback，HAL库会调用这个函数，不要手动调用
 */
void FdCan::Fifo0MsgPendingCallback() {
  static FDCAN_RxHeaderTypeDef rx_header;
  HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header, rx_buffer_.data.data());
  auto &device_list_ = GetDeviceListByRxStdid(rx_header.Identifier);
  if (device_list_.empty()) {
    return;
  }
  rx_buffer_.rx_std_id = rx_header.Identifier;
  if (rx_header.FDFormat == FDCAN_FD_CAN) {
    // FD格式需要把DLC码转换为字节长度
    if (rx_header.DataLength <= FDCAN_DLC_BYTES_8) {
      rx_buffer_.dlc = rx_header.DataLength;  // 8字节以下DLC码和数据长度相同
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_12) {
      rx_buffer_.dlc = 12;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_16) {
      rx_buffer_.dlc = 16;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_20) {
      rx_buffer_.dlc = 20;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_24) {
      rx_buffer_.dlc = 24;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_32) {
      rx_buffer_.dlc = 32;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_48) {
      rx_buffer_.dlc = 48;
    } else {
      rx_buffer_.dlc = 64;
    }
  } else {
    rx_buffer_.dlc = rx_header.DataLength;
  }
  for (auto &device : device_list_) {
    device->RxCallback(&rx_buffer_);
  }
}

}  // namespace rm::hal::stm32

#endif