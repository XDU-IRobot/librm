/*
  Copyright (c) 2024 XDU-IRobot

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
 * @todo  部分完成
 */

#include "librm/hal/stm32/hal.h"
#if defined(HAL_FDCAN_MODULE_ENABLED)
#include "librm/hal/stm32/check_register_callbacks.h"

#include "fdcan.h"

#include <functional>

#include "librm/device/can_device.hpp"
#include "librm/core/exception.h"

/**
 * 用于存储回调函数的map
 * key: HAL库的CAN_HandleTypeDef
 * value: 回调函数
 */
static std::unordered_map<FDCAN_HandleTypeDef *, std::function<void()>> fn_cb_map;

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

using core::exception::Exception;
using core::exception::ThrowException;

/**
 * @param hcan HAL库的CAN_HandleTypeDef
 */
FdCan::FdCan(FDCAN_HandleTypeDef &hfdcan) : hfdcan_(&hfdcan) {}

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
  if (reinterpret_cast<u32>(this->hfdcan_->Instance) == FDCAN1_BASE) {
    can_filter_st.FilterIndex = 0;
  }
  if (reinterpret_cast<u32>(this->hfdcan_->Instance) == FDCAN2_BASE) {
    can_filter_st.FilterIndex = 1;
  }
  if (reinterpret_cast<u32>(this->hfdcan_->Instance) == FDCAN3_BASE) {
    can_filter_st.FilterIndex = 2;
  }
  if (HAL_FDCAN_ConfigGlobalFilter(this->hfdcan_, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, DISABLE, DISABLE) == HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_ConfigFilter error
  }
  if (HAL_FDCAN_ConfigFilter(this->hfdcan_, &can_filter_st) == HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_ConfigFilter error
  }
}

/**
 * @brief 立刻向总线上发送数据
 * @param id    标准帧ID
 * @param data  数据指针
 * @param size  数据长度
 */
void FdCan::Write(u16 id, const u8 *data, usize size) {
  if (size > 8) {
    // todo:发送长度大于8的消息
    ThrowException(Exception::kValueError);  // 数据长度超过8
  }
  this->hal_tx_header_.Identifier = id;
  this->hal_tx_header_.DataLength = size;
  if (HAL_FDCAN_AddMessageToTxFifoQ(this->hfdcan_, &this->hal_tx_header_, const_cast<u8 *>(data)) != HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_AddTxMessage error
  }
}

/**
 * @brief 从消息队列里取出一条消息发送
 */
void FdCan::Write() {
  // 按优先级从高到低遍历所有消息队列
  for (auto &queue : this->tx_queue_) {
    if (queue.second.empty()) {
      continue;  // 如果是空的就换下一个
    }
    // 从队首取出一条消息发送
    this->Write(queue.second.front()->rx_std_id, queue.second.front()->data.data(), queue.second.front()->dlc);
    queue.second.pop_front();
    // 检查消息队列长度是否超过了设定的最大长度，如果超过了就清空
    if (queue.second.size() > kQueueMaxSize) {
      queue.second.clear();
    }
    break;
  }
}

/**
 * @brief 向消息队列里加入一条消息
 * @param id        数据帧ID
 * @param data      数据指针
 * @param size      数据长度
 * @param priority  消息的优先级
 */
void FdCan::Enqueue(u16 id, const u8 *data, usize size, CanTxPriority priority) {
  if (size > 8) {
    // todo:发送长度大于8的消息
    ThrowException(Exception::kValueError);  // 数据长度超过8
  }
  // 检查消息队列长度是否超过了设定的最大长度，如果超过了就清空
  if (this->tx_queue_[priority].size() > kQueueMaxSize) {
    this->tx_queue_[priority].clear();
  }
  auto msg = std::make_shared<CanMsg>(CanMsg{
      .rx_std_id = id,
      .dlc = size,
  });
  std::copy_n(data, size, msg->data.begin());

  this->tx_queue_[priority].push_back(msg);
}

/**
 * @brief 启动CAN外设
 */
void FdCan::Begin() {
  if (HAL_FDCAN_ActivateNotification(this->hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_ActivateNotification error
  }
  if (HAL_FDCAN_ActivateNotification(this->hfdcan_, FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_ActivateNotification error
  }
  HAL_FDCAN_RegisterRxFifo0Callback(
      this->hfdcan_, StdFunctionToCallbackFunctionPtr([this] { Fifo0MsgPendingCallback(); }, this->hfdcan_));
  if (HAL_FDCAN_Start(this->hfdcan_) != HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_Start error
  }
}

/**
 * @brief 停止CAN外设
 */
void FdCan::Stop() {
  if (HAL_FDCAN_Stop(this->hfdcan_) != HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_Stop error
  };
}

/**
 * @brief 利用Register callbacks机制，用这个函数替代HAL_CAN_RxFifo0MsgPendingCallback
 * @note  这个函数替代了HAL_CAN_RxFifo0MsgPendingCallback，HAL库会调用这个函数，不要手动调用
 */
void FdCan::Fifo0MsgPendingCallback() {
  static FDCAN_RxHeaderTypeDef rx_header;
  HAL_FDCAN_GetRxMessage(this->hfdcan_, FDCAN_RX_FIFO0, &rx_header, this->rx_buffer_.data.data());
  if (this->device_list_.find(rx_header.Identifier) == this->device_list_.end()) {
    return;
  }
  this->rx_buffer_.rx_std_id = rx_header.Identifier;
  this->rx_buffer_.dlc = rx_header.DataLength;
  this->device_list_[rx_header.Identifier]->RxCallback(&this->rx_buffer_);
}

/**
 * @brief 注册一个CAN设备
 * @param device    设备对象
 * @param rx_stdid  设备想要接收的的rx消息标准帧id
 */
void FdCan::RegisterDevice(device::CanDevice &device, u32 rx_stdid) {
  if (this->device_list_.find(rx_stdid) != this->device_list_.end()) {
    ThrowException(Exception::kValueError);
  }
  this->device_list_[rx_stdid] = &device;
}

}  // namespace rm::hal::stm32

#endif