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
 * @file  librm/hal/stm32/uart.cc
 * @brief UART类库
 */

#include "librm/hal/stm32/hal.h"
#if defined(HAL_UART_MODULE_ENABLED)
#include "librm/hal/stm32/check_register_callbacks.h"

#include "uart.h"

#include "librm/core/exception.h"

/**
 * @brief 串口接收回调函数键值对
 * @note  用于存储串口接收回调函数
 * @note  key: 串口对象指针
 * @note  value: 回调函数
 * 解释：键是串口对象指针，值是回调函数，用于实现多个串口的回调函数
 */
static std::unordered_map<UART_HandleTypeDef *, std::function<void(u16)>> fn_cb_map;

/**
 * @brief 串口错误回调函数键值对
 * @note  用于存储串口错误回调函数
 * @note  key: 串口对象指针
 * @note  value: 错误回调函数
 * 解释：键是串口对象指针，值是错误回调函数，用于实现多个串口的错误回调函数
 */
static std::unordered_map<UART_HandleTypeDef *, std::function<void(void)>> fn_error_map;

/**
 * @brief  把std::function转换为函数指针
 * @param  fn   要转换的函数
 * @return      转换后的函数指针
 * @note
 * 背景：因为要用面向对象的方式对外设进行封装，所以回调函数必须存在于类内。但是存在于类内就意味着这个回调函数多了一个this参数，
 * 而HAL库要求的回调函数并没有这个this参数。通过std::bind，可以生成一个参数列表里没有this指针的std::function对象，而std::function
 * 并不能直接强转成函数指针。借助这个函数，可以把std::function对象转换成函数指针。然后就可以把这个类内的回调函数传给HAL库了。
 */

static auto StdFunctionToCallbackFunctionPtr(std::function<void(u16)> fn,
                                             UART_HandleTypeDef *huart) -> pUART_RxEventCallbackTypeDef {
  fn_cb_map[huart] = std::move(fn);
  return [](UART_HandleTypeDef *handle, u16 rx_len) {
    if (fn_cb_map.find(handle) != fn_cb_map.end()) {
      fn_cb_map[handle](rx_len);
    }
  };
}

static auto StdFunctionToErrorCallbackFunctionPtr(std::function<void(void)> fn,
                                                  UART_HandleTypeDef *huart) -> pUART_CallbackTypeDef {
  fn_error_map[huart] = std::move(fn);
  return [](UART_HandleTypeDef *handle) {
    if (fn_error_map.find(handle) != fn_error_map.end()) {
      fn_error_map[handle]();
    }
  };
}

namespace rm::hal::stm32 {

using core::exception::Exception;
using core::exception::ThrowException;

/**
 * @param huart            HAL库的UART句柄
 * @param rx_buffer_size   接收缓冲区大小
 * @param tx_mode          tx工作模式（正常、中断、DMA）
 * @param rx_mode          rx工作模式（正常、中断、DMA）
 */
Uart::Uart(UART_HandleTypeDef &huart, usize rx_buffer_size, UartMode tx_mode, UartMode rx_mode)
    : huart_(&huart),
      tx_mode_(tx_mode),
      rx_mode_(rx_mode),
      rx_buf_{std::vector<u8>(rx_buffer_size), std::vector<u8>(rx_buffer_size)} {}

/**
 * @brief 初始化UART
 */
void Uart::Begin() {
  // 检查dma模式下是否已经配置好DMA
  if (this->tx_mode_ == UartMode::kDma && this->huart_->hdmatx == nullptr) {
    ThrowException(Exception::kHALError);
  }
  if (this->rx_mode_ == UartMode::kDma && this->huart_->hdmarx == nullptr) {
    ThrowException(Exception::kHALError);
  }
  // 注册接收完成回调函数
  HAL_UART_RegisterRxEventCallback(
      this->huart_,
      StdFunctionToCallbackFunctionPtr(std::bind(&Uart::HalRxCpltCallback, this, std::placeholders::_1), this->huart_));

  // 注册错误回调函数
  HAL_UART_RegisterCallback(
      this->huart_, HAL_UART_ERROR_CB_ID,
      StdFunctionToErrorCallbackFunctionPtr(std::bind(&Uart::HalErrorCallback, this), this->huart_));

  // 启动接收
  switch (this->rx_mode_) {
    case UartMode::kNormal:
      HAL_UART_Receive(this->huart_, this->rx_buf_[0].data(), this->rx_buf_[this->buffer_selector_].size(),
                       HAL_MAX_DELAY);
      break;
    case UartMode::kInterrupt:
      HAL_UART_Receive_IT(this->huart_, this->rx_buf_[0].data(), this->rx_buf_[this->buffer_selector_].size());
      break;
#if defined(HAL_DMA_MODULE_ENABLED)
    case UartMode::kDma:
      HAL_UARTEx_ReceiveToIdle_DMA(this->huart_, this->rx_buf_[0].data(), this->rx_buf_[this->buffer_selector_].size());
      __HAL_DMA_DISABLE_IT(this->huart_->hdmarx, DMA_IT_HT);  // 关闭DMA半传输中断
      break;
#endif
  }
}

/**
 * @brief 发送数据
 * @param data 数据指针
 * @param size 数据大小
 */
void Uart::Write(const u8 *data, usize size) {
  switch (this->tx_mode_) {
    case UartMode::kNormal:
      HAL_UART_Transmit(this->huart_, const_cast<u8 *>(data), size, HAL_MAX_DELAY);
      break;
    case UartMode::kInterrupt:
      HAL_UART_Transmit_IT(this->huart_, const_cast<u8 *>(data), size);
      break;
#if defined(HAL_DMA_MODULE_ENABLED)
    case UartMode::kDma:
      HAL_UART_Transmit_DMA(this->huart_, const_cast<u8 *>(data), size);
      break;
#endif
  }
}

/**
 * @brief 注册用户定义的接收完成回调函数
 * @param callback 回调函数
 */
void Uart::AttachRxCallback(SerialRxCallbackFunction &callback) { this->rx_callbacks_.push_back(&callback); }

/**
 * @return 接收缓冲区
 */
const std::vector<u8> &Uart::rx_buffer() const { return rx_buf_[buffer_selector_]; }

/**
 * @brief 接收完成回调函数
 * @note  这个回调函数是给HAL库用的，要实现自己的功能就在外部定义一个UartCallbackFunction
 * 类型的回调函数，然后通过AttachRxCallback注册
 */
void Uart::HalRxCpltCallback(u16 rx_len) {
  // 判断rx模式，重新启动接收
  switch (this->rx_mode_) {
    case UartMode::kNormal:
      HAL_UART_Receive(this->huart_, this->rx_buf_[!this->buffer_selector_].data(),
                       this->rx_buf_[!this->buffer_selector_].size(), HAL_MAX_DELAY);
      break;
    case UartMode::kInterrupt:
      HAL_UART_Receive_IT(this->huart_, this->rx_buf_[!this->buffer_selector_].data(),
                          this->rx_buf_[!this->buffer_selector_].size());
      break;
#if defined(HAL_DMA_MODULE_ENABLED)
    case UartMode::kDma:
      HAL_UARTEx_ReceiveToIdle_DMA(this->huart_, this->rx_buf_[!this->buffer_selector_].data(),
                                   this->rx_buf_[!this->buffer_selector_].size());
      __HAL_DMA_DISABLE_IT(this->huart_->hdmarx, DMA_IT_HT);  // 关闭DMA半传输中断
      break;
#endif
  }
  // 调用外部重写的回调函数
  for (auto callback : this->rx_callbacks_) {
    if (callback != nullptr) {
      (*callback)(this->rx_buf_[this->buffer_selector_], rx_len);
    }
  }
  // 切换缓冲区
  this->buffer_selector_ = !this->buffer_selector_;
}

void Uart::HalErrorCallback() {
  // 重启接收
  u16 rx_len = this->rx_buf_[this->buffer_selector_].size();
  switch (this->rx_mode_) {
    case UartMode::kNormal:
      HAL_UART_Receive(this->huart_, this->rx_buf_[!this->buffer_selector_].data(),
                       this->rx_buf_[!this->buffer_selector_].size(), HAL_MAX_DELAY);
      break;
    case UartMode::kInterrupt:
      HAL_UART_Receive_IT(this->huart_, this->rx_buf_[!this->buffer_selector_].data(),
                          this->rx_buf_[!this->buffer_selector_].size());
      break;
#if defined(HAL_DMA_MODULE_ENABLED)
    case UartMode::kDma:
      HAL_UARTEx_ReceiveToIdle_DMA(this->huart_, this->rx_buf_[!this->buffer_selector_].data(),
                                   this->rx_buf_[!this->buffer_selector_].size());
      __HAL_DMA_DISABLE_IT(this->huart_->hdmarx, DMA_IT_HT);  // 关闭DMA半传输中断
      break;
#endif
  }
}

}  // namespace rm::hal::stm32

#endif