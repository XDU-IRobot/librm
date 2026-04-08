/*
  Copyright (c) 2026 XDU-IRobot

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

#include "librm/device/can_device.hpp"
#include "librm/hal/stm32/helper_macro.hpp"

namespace rm::hal::stm32 {

/**
 * @brief 存储FDCAN1、FDCAN2、FDCAN3对应的FdCan实例指针
 */
static FdCan *fdcan_instances[3] = {nullptr, nullptr, nullptr};

/**
 * @brief  根据FDCAN实例地址获取fdcan_instances数组索引
 * @param  hfdcan  HAL库的FDCAN_HandleTypeDef
 * @return 数组索引（0、1或2），如果不是有效的FDCAN实例则返回-1
 */
__attribute__((always_inline)) static inline int GetFdCanIndex(FDCAN_HandleTypeDef *hfdcan) {
  const auto instance = reinterpret_cast<uintptr_t>(hfdcan->Instance);
  if (instance == FDCAN1_BASE) return 0;
#if defined(FDCAN2_BASE)
  if (instance == FDCAN2_BASE) return 1;
#endif
#if defined(FDCAN3_BASE)
  if (instance == FDCAN3_BASE) return 2;
#endif
  return -1;
}

/**
 * @brief  注册给HAL库的FDCAN接收回调函数
 * @param  hfdcan  HAL库的FDCAN_HandleTypeDef
 * @param  RxFifo0ITs  中断标志
 */
void FdCanRxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan, u32 RxFifo0ITs) {
  const int fdcan_idx = GetFdCanIndex(hfdcan);
  if (fdcan_idx >= 0) {
    FdCan *instance = fdcan_instances[fdcan_idx];
    if (instance != nullptr) {
      instance->Fifo0MsgPendingCallback();
    }
  }
}

/**
 * @brief  Bus-Off错误回调函数，当总线进入Bus-Off状态时自动恢复
 * @param  hfdcan  HAL库的FDCAN_HandleTypeDef
 */
void FdCanErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, u32 ErrorStatusITs) {
  if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
    const int fdcan_idx = GetFdCanIndex(hfdcan);
    if (fdcan_idx >= 0) {
      FdCan *instance = fdcan_instances[fdcan_idx];
      if (instance != nullptr) {
        instance->Restart();
      }
    }
  }
}

/**
 * @param hfdcan HAL库的CAN_HandleTypeDef
 */
FdCan::FdCan(FDCAN_HandleTypeDef &hfdcan)
    : hfdcan_(&hfdcan),
      // 判断这个FDCAN句柄对应的外设是否被设置为FD模式
      fd_mode_{hfdcan.Init.FrameFormat == FDCAN_FRAME_FD_NO_BRS ||  //
               hfdcan.Init.FrameFormat == FDCAN_FRAME_FD_BRS} {
  int idx = GetFdCanIndex(hfdcan_);
  if (idx >= 0) {
    fdcan_instances[idx] = this;
  }
}

FdCan::~FdCan() {
  Stop();
  int idx = GetFdCanIndex(hfdcan_);
  if (idx >= 0) {
    fdcan_instances[idx] = nullptr;
  }
}

FdCan::FdCan(FdCan &&other) noexcept : hfdcan_(other.hfdcan_), fd_mode_(other.fd_mode_) {
  // 把全局数组里的指针指向新的自己
  int idx = GetFdCanIndex(hfdcan_);
  if (idx >= 0) {
    fdcan_instances[idx] = this;
  }
}

FdCan &FdCan::operator=(FdCan &&other) noexcept {
  if (this != &other) {
    hfdcan_ = other.hfdcan_;
    const_cast<bool &>(fd_mode_) = other.fd_mode_;
    // 把全局数组里的指针指向新的自己
    int idx = GetFdCanIndex(hfdcan_);
    if (idx >= 0) {
      fdcan_instances[idx] = this;
    }
  }
  return *this;
}

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

  LIBRM_STM32_HAL_ASSERT(
      HAL_FDCAN_ConfigGlobalFilter(hfdcan_, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, DISABLE, DISABLE));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_ConfigFilter(hfdcan_, &can_filter_st));
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
    u32 dlc_code;
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

  // 检查是否处于Bus-Off状态，如果是则尝试恢复
  FDCAN_ProtocolStatusTypeDef protocol_status;
  HAL_FDCAN_GetProtocolStatus(hfdcan_, &protocol_status);
  if (protocol_status.BusOff) {
    Restart();
    return;  // 这一帧放弃发送
  }

  // 等待TX FIFO有空闲位置并入队，若入队失败则继续等待重试，增加超时保护
  constexpr u32 kTimeoutLimit = 100000;
  constexpr u32 kMaxRetries = 10;
  u32 retry_count = 0;
  while (true) {
    u32 timeout_counter = 0;
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan_) == 0) {
      if (++timeout_counter > kTimeoutLimit) {
        HAL_FDCAN_GetProtocolStatus(hfdcan_, &protocol_status);
        if (protocol_status.BusOff) {
          Restart();
        }
        return;  // 放弃这一帧
      }
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &hal_tx_header_, data) == HAL_OK) {
      break;  // 入队成功
    }
    if (++retry_count >= kMaxRetries) {
      return;  // 超过最大重试次数，放弃这一帧
    }
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

  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_BUS_OFF, 0));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_RegisterRxFifo0Callback(hfdcan_, FdCanRxFifo0MsgPendingCallback));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_RegisterErrorStatusCallback(hfdcan_, FdCanErrorStatusCallback));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_Start(hfdcan_));
}

/**
 * @brief 停止CAN外设
 */
void FdCan::Stop() { LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_Stop(hfdcan_)); }

/**
 * @brief 重启外设，用于从Bus-Off状态恢复
 * @note  停止外设、重新激活中断通知、再启动外设
 */
void FdCan::Restart() const {
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_Stop(hfdcan_));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_BUS_OFF, 0));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_Start(hfdcan_));
}

/**
 * @brief 利用Register callbacks机制，用这个函数替代HAL_CAN_RxFifo0MsgPendingCallback
 * @note  这个函数替代了HAL_CAN_RxFifo0MsgPendingCallback，HAL库会调用这个函数，不要手动调用
 */
void FdCan::Fifo0MsgPendingCallback() const {
  FDCAN_RxHeaderTypeDef rx_header;
  CanFrame rx_frame;
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header, rx_frame.data.data()));
  auto &device_list_ = GetDeviceListByRxStdid(rx_header.Identifier);
  if (device_list_.empty()) {
    return;
  }
  rx_frame.rx_std_id = rx_header.Identifier;
  if (rx_header.FDFormat == FDCAN_FD_CAN) {
    // FD格式需要把DLC码转换为字节长度
    if (rx_header.DataLength <= FDCAN_DLC_BYTES_8) {
      rx_frame.dlc = rx_header.DataLength;  // 8字节以下DLC码和数据长度相同
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_12) {
      rx_frame.dlc = 12;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_16) {
      rx_frame.dlc = 16;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_20) {
      rx_frame.dlc = 20;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_24) {
      rx_frame.dlc = 24;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_32) {
      rx_frame.dlc = 32;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_48) {
      rx_frame.dlc = 48;
    } else {
      rx_frame.dlc = 64;
    }
  } else {
    rx_frame.dlc = rx_header.DataLength;
  }
  for (auto &device : device_list_) {
    device->RxCallback(&rx_frame);
  }
}

}  // namespace rm::hal::stm32

#endif