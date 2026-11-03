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
 * @file  librm/hal/stm32/bxcan.cc
 * @brief bxCAN类库
 */

#include "librm/hal/stm32/hal.hpp"

#if defined(HAL_CAN_MODULE_ENABLED)

#include "bxcan.hpp"

#include "librm/device/can_device.hpp"
#include "librm/hal/stm32/helper_macro.hpp"

namespace rm::hal::stm32 {

/**
 * @brief 存储CAN1和CAN2对应的BxCan实例指针
 */
static BxCan *bxcan_instances[2] = {nullptr, nullptr};

/**
 * @brief  根据CAN实例地址获取bxcan_instances数组索引
 * @param  hcan  HAL库的CAN_HandleTypeDef
 * @return 数组索引（0或1），如果不是有效的CAN实例则返回-1
 */
__attribute__((always_inline)) static inline int GetCanIndex(CAN_HandleTypeDef *hcan) {
  const auto instance = reinterpret_cast<uintptr_t>(hcan->Instance);
  if (instance == CAN1_BASE) return 0;
  if (instance == CAN2_BASE) return 1;
  return -1;
}

/**
 * @brief  注册给HAL库的CAN接收回调函数
 * @param  hcan  HAL库的CAN_HandleTypeDef
 */
void CanRxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  const int can_idx = GetCanIndex(hcan);
  if (can_idx >= 0) {
    BxCan *instance = bxcan_instances[can_idx];
    if (instance != nullptr) {
      instance->Fifo0MsgPendingCallback();
    }
  }
}

/**
 * @param hcan HAL库的CAN_HandleTypeDef
 */
BxCan::BxCan(CAN_HandleTypeDef &hcan) : hcan_(&hcan) {
  int idx = GetCanIndex(hcan_);
  if (idx >= 0) {
    bxcan_instances[idx] = this;
  }
}

BxCan::~BxCan() {
  Stop();
  int idx = GetCanIndex(hcan_);
  if (idx >= 0) {
    bxcan_instances[idx] = nullptr;
  }
}

BxCan::BxCan(BxCan &&other) noexcept : hcan_(other.hcan_) {
  // 把全局数组里的指针指向新的自己
  int idx = GetCanIndex(hcan_);
  if (idx >= 0) {
    bxcan_instances[idx] = this;
  }
}

BxCan &BxCan::operator=(BxCan &&other) noexcept {
  if (this != &other) {
    hcan_ = other.hcan_;
    // 把全局数组里的指针指向新的自己
    int idx = GetCanIndex(hcan_);
    if (idx >= 0) {
      bxcan_instances[idx] = this;
    }
  }
  return *this;
}

/**
 * @brief 设置过滤器
 * @param id
 * @param mask
 */
void BxCan::SetFilter(u16 id, u16 mask) {
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = id >> 8;
  can_filter_st.FilterIdLow = id;
  can_filter_st.FilterMaskIdHigh = mask >> 8;
  can_filter_st.FilterMaskIdLow = mask;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  if (reinterpret_cast<u32>(hcan_->Instance) == CAN1_BASE) {
    // 如果是CAN1
    can_filter_st.FilterBank = 0;
  } else if (reinterpret_cast<u32>(hcan_->Instance) == CAN2_BASE) {
    // 如果是CAN2
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
  }
  LIBRM_STM32_HAL_ASSERT(HAL_CAN_ConfigFilter(hcan_, &can_filter_st));
}

/**
 * @brief 立刻向总线上发送数据
 * @param id    标准帧ID
 * @param data  数据指针
 * @param size  数据长度
 * @note  已废弃，请改用 Write(const CanFrame &)
 */
void BxCan::Write(u16 id, const u8 *data, usize size) { WriteFrame(id, data, static_cast<u8>(size), false); }

/**
 * @brief 立刻向总线上发送一帧
 * @param frame 数据帧，std_id 为发送的目标ID，is_fd_frame 必须为 false
 */
void BxCan::Write(const CanFrame &frame) {
  WriteFrame(frame.std_id, frame.data.data(), static_cast<u8>(frame.dlc), frame.is_fd_frame);
}

/**
 * @brief 发送一帧的公共实现
 * @param id          标准帧ID
 * @param data        数据指针
 * @param dlc         数据长度（字节，0~8）
 * @param is_fd_frame 是否发送FD帧；bxCAN不支持FD，传true会直接停机
 */
void BxCan::WriteFrame(u16 id, const u8 *data, u8 dlc, bool is_fd_frame) {
  if (is_fd_frame) {
    Throw(rm::runtime_error("bxcan: CAN FD frame requested, but bxCAN does not support CAN FD"));
  }
  if (dlc > 8u) {
    Throw(rm::runtime_error("bxcan: data too long for a classic CAN frame"));
  }

  hal_tx_header_.StdId = id;
  hal_tx_header_.DLC = dlc;

  // 等待CAN外设空闲
  while (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0) {
  }

  LIBRM_STM32_HAL_ASSERT(HAL_CAN_AddTxMessage(hcan_, &hal_tx_header_, data, &tx_mailbox_));
}

/**
 * @brief 启动CAN外设
 */
void BxCan::Begin() {
  LIBRM_STM32_HAL_ASSERT(
      HAL_CAN_RegisterCallback(hcan_, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CanRxFifo0MsgPendingCallback));
  LIBRM_STM32_HAL_ASSERT(HAL_CAN_Start(hcan_));
  LIBRM_STM32_HAL_ASSERT(HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING));
}

/**
 * @brief 停止CAN外设
 */
void BxCan::Stop() { LIBRM_STM32_HAL_ASSERT(HAL_CAN_Stop(hcan_)); }

/**
 * @brief 利用Register callbacks机制，用这个函数替代HAL_CAN_RxFifo0MsgPendingCallback
 * @note  这个函数替代了HAL_CAN_RxFifo0MsgPendingCallback，HAL库会调用这个函数，不要手动调用
 */
void BxCan::Fifo0MsgPendingCallback() {
  CAN_RxHeaderTypeDef rx_header;
  LIBRM_STM32_HAL_ASSERT(HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &rx_header, rx_buffer_.data.data()));

  auto &device_list = GetDeviceListByRxStdid(rx_header.StdId);
  if (device_list.empty()) {
    return;
  }

  // 设置rx_buffer_成员（只有在有设备监听时才需要）
  rx_buffer_.std_id = rx_header.StdId;
  rx_buffer_.dlc = static_cast<CanFrame::Dlc>(rx_header.DLC);

  // 遍历并调用所有注册设备的回调
  for (auto &device : device_list) {
    device->RxCallback(&rx_buffer_);
  }
}

}  // namespace rm::hal::stm32

#endif