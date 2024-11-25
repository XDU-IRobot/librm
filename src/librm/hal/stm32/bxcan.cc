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
 * @file  librm/hal/stm32/bxcan.cc
 * @brief bxCAN类库
 */

#include "librm/hal/stm32/hal.h"
#if defined(HAL_CAN_MODULE_ENABLED)
#include "librm/hal/stm32/check_register_callbacks.h"

#include "bxcan.h"

#include <functional>

#include "librm/device/can_device.hpp"
#include "librm/core/exception.h"

/*
  * 用于存储回调函数的map
  * key: HAL库的CAN_HandleTypeDef
  * value: 回调函数
  */
*/
static std::unordered_map<CAN_HandleTypeDef *, std::function<void()>> fn_cb_map;

/**
 * @brief  把std::function转换为函数指针
 * @param  fn   要转换的函数
 * @return      转换后的函数指针
 * @note
 * 背景：因为要用面向对象的方式对外设进行封装，所以回调函数必须存在于类内。但是存在于类内就意味着这个回调函数多了一个this参数，
 * 而HAL库要求的回调函数并没有这个this参数。通过std::bind，可以生成一个参数列表里没有this指针的std::function对象，而std::function
 * 并不能直接强转成函数指针。借助这个函数，可以把std::function对象转换成函数指针。然后就可以把这个类内的回调函数传给HAL库了。
 */
static pCAN_CallbackTypeDef StdFunctionToCallbackFunctionPtr(std::function<void()> fn, CAN_HandleTypeDef *hcan) {
  fn_cb_map[hcan] = std::move(fn);
  return [](CAN_HandleTypeDef *hcan) {
    if (fn_cb_map.find(hcan) != fn_cb_map.end()) {
      fn_cb_map[hcan]();
    }
  };
}

namespace rm::hal::stm32 {

using core::exception::Exception;
using core::exception::ThrowException;

/**
 * @param hcan HAL库的CAN_HandleTypeDef
 */
BxCan::BxCan(CAN_HandleTypeDef &hcan) : hcan_(&hcan) {}

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
  if (reinterpret_cast<u32>(this->hcan_->Instance) == CAN1_BASE) {
    // 如果是CAN1
    can_filter_st.FilterBank = 0;
  } else if (reinterpret_cast<u32>(this->hcan_->Instance) == CAN2_BASE) {
    // 如果是CAN2
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
  }
  if (HAL_CAN_ConfigFilter(this->hcan_, &can_filter_st) != HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_ConfigFilter error
  }
}

/**
 * @brief 立刻向总线上发送数据
 * @param id    标准帧ID
 * @param data  数据指针
 * @param size  数据长度
 */
void BxCan::Write(u16 id, const u8 *data, usize size) {
  if (size > 8) {
    ThrowException(Exception::kValueError);  // 数据长度超过8
  }
  this->hal_tx_header_.StdId = id;
  this->hal_tx_header_.DLC = size;
  if (HAL_CAN_AddTxMessage(this->hcan_, &this->hal_tx_header_, const_cast<u8 *>(data), &this->tx_mailbox_) != HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_AddTxMessage error
  }
}

/**
 * @brief 从消息队列里取出一条消息发送
 */
void BxCan::Write() {
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
void BxCan::Enqueue(u16 id, const u8 *data, usize size, CanTxPriority priority) {
  if (size > 8) {
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
void BxCan::Begin() {
  if (HAL_CAN_Start(this->hcan_) != HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_Start error
  }
  if (HAL_CAN_ActivateNotification(this->hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_ActivateNotification error
  }
  HAL_CAN_RegisterCallback(this->hcan_, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
                           StdFunctionToCallbackFunctionPtr(std::bind(&BxCan::Fifo0MsgPendingCallback, this), this->hcan_));
}

/**
 * @brief 停止CAN外设
 */
void BxCan::Stop() {
  if (HAL_CAN_Stop(this->hcan_) != HAL_OK) {
    ThrowException(Exception::kHALError);  // HAL_CAN_Stop error
  };
}

/**
 * @brief 利用Register callbacks机制，用这个函数替代HAL_CAN_RxFifo0MsgPendingCallback
 * @note  这个函数替代了HAL_CAN_RxFifo0MsgPendingCallback，HAL库会调用这个函数，不要手动调用
 */
void BxCan::Fifo0MsgPendingCallback() {
  static CAN_RxHeaderTypeDef rx_header;
  HAL_CAN_GetRxMessage(this->hcan_, CAN_RX_FIFO0, &rx_header, this->rx_buffer_.data.data());
  if (this->device_list_.find(rx_header.StdId) == this->device_list_.end()) {
    return;
  }
  this->rx_buffer_.rx_std_id = rx_header.StdId;
  this->rx_buffer_.dlc = rx_header.DLC;
  this->device_list_[rx_header.StdId]->RxCallback(&this->rx_buffer_);
}

/**
 * @brief 注册一个CAN设备
 * @param device    设备对象
 * @param rx_stdid  设备想要接收的的rx消息标准帧id
 */
void BxCan::RegisterDevice(device::CanDevice &device, u32 rx_stdid) {
  if (this->device_list_.find(rx_stdid) != this->device_list_.end()) {
    ThrowException(Exception::kValueError);
  }
  this->device_list_[rx_stdid] = &device;
}

}  // namespace rm::hal::stm32

#endif