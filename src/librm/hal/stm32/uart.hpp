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
 * @file  librm/hal/stm32/uart.hpp
 * @brief UART 类库
 *
 * Uart<RxBufSize> 是一个模板类，RxBufSize 在编译时确定接收缓冲区大小，
 * 双缓冲 (rx_buf_[2]) 使用 std::array 分配在对象所在的存储区（通常为 BSS），
 * 不涉及堆分配。
 *
 * HAL 回调通过无捕获 lambda（可隐式转换为函数指针）+ 内联注册表转发，
 * 注册表本身也使用 std::array，零堆分配。
 */

#ifndef LIBRM_HAL_STM32_UART_HPP
#define LIBRM_HAL_STM32_UART_HPP

#include "librm/hal/stm32/hal.hpp"
#ifdef HAL_UART_MODULE_ENABLED
#include "librm/hal/stm32/check_register_callbacks.hpp"

#include <array>
#include <functional>

#include <etl/span.h>
#include <etl/vector.h>

#include "librm/hal/serial_interface.hpp"
#include "librm/hal/stm32/helper_macro.hpp"
#include "librm/core/exception.hpp"
#include "librm/core/typedefs.hpp"

namespace rm::hal::stm32 {

/**
 * @name UartBase
 * @brief 非模板基类，用于 HAL 回调的多态转发
 * @{
 */
class UartBase {
  template <usize>
  friend class Uart;

  virtual void HalRxCpltCallback(u16 rx_len) = 0;
  virtual void HalTxCpltCallback() = 0;
  virtual void HalErrorCallback() = 0;
  virtual ~UartBase() = default;
};
/** @} */

/**
 * @name 内联注册表
 * @brief 无堆，最多支持 kMaxUartInstances 个 Uart 实例
 * @{
 */
namespace detail {

constexpr usize kMaxUartInstances = 8;

struct UartEntry {
  UART_HandleTypeDef *handle{nullptr};
  UartBase *obj{nullptr};
};

// inline 变量：C++17，所有 TU 共享同一对象，无 ODR 问题
inline std::array<UartEntry, kMaxUartInstances> g_uart_registry{};

inline void UartRegister(UART_HandleTypeDef *h, UartBase *obj) {
  for (auto &e : g_uart_registry) {
    if (e.handle == nullptr) {
      e = {h, obj};
      return;
    }
  }
  Throw(std::runtime_error("uart_register: registry full"));
}

inline void UartUnregister(UART_HandleTypeDef *h) {
  for (auto &e : g_uart_registry) {
    if (e.handle == h) {
      e = {nullptr, nullptr};
      return;
    }
  }
}

inline UartBase *UartFind(UART_HandleTypeDef *h) {
  for (auto &e : g_uart_registry) {
    if (e.handle == h) return e.obj;
  }
  return nullptr;
}

}  // namespace detail
/** @} */

/**
 * @brief UART 类（模板参数 RxBufSize 决定双缓冲每块的字节数）
 *
 * TX 阻塞写与发送中断/DMA 正交。这允许同时调用 Read()、Write()、Start()、WriteAsync()。
 */
template <usize RxBufSize>
class Uart : public SerialInterface, public AsyncWritable, public SyncReadable, UartBase {
 public:
  explicit Uart(UART_HandleTypeDef &huart, bool async_tx_use_dma = false, bool async_rx_use_dma = false)
      : huart_(&huart), async_tx_use_dma_(async_tx_use_dma), async_rx_use_dma_(async_rx_use_dma) {}

  ~Uart() override {
    Uart<RxBufSize>::Stop();
    detail::UartUnregister(huart_);
  }

  // /// @name SyncWritable
  void Write(const u8 *data, usize size, u32 timeout_ms) override {
    LIBRM_STM32_HAL_ASSERT(HAL_UART_Transmit(huart_, const_cast<u8 *>(data), size, timeout_ms));
  }

  // /// @name AsyncWritable
  void WriteAsync(const u8 *data, usize size, std::function<void()> on_done) override {
    tx_done_callback_ = std::move(on_done);
#ifdef HAL_DMA_MODULE_ENABLED
    if (async_tx_use_dma_) {
      LIBRM_STM32_HAL_ASSERT(HAL_UART_Transmit_DMA(huart_, const_cast<u8 *>(data), size));
    } else
#endif
    {
      LIBRM_STM32_HAL_ASSERT(HAL_UART_Transmit_IT(huart_, const_cast<u8 *>(data), size));
    }
  }

  // /// @name SyncReadable
  usize Read(u8 *buf, usize size, u32 timeout_ms) override {
    const HAL_StatusTypeDef s = HAL_UART_Receive(huart_, buf, static_cast<u16>(size), timeout_ms);
    return (s == HAL_OK) ? size : 0u;
  }

  // /// @name AsyncReadable
  void AttachRxCallback(SerialRxCallbackFunction callback) override { rx_callbacks_.emplace_back(std::move(callback)); }

  void Start() override {
    if (is_receiving_) {
      return;
    }
#ifdef HAL_DMA_MODULE_ENABLED
    if (async_rx_use_dma_ && huart_->hdmarx == nullptr) {
      Throw(std::runtime_error("async_rx_use_dma_ selected but hdmarx is nullptr"));
    }
#endif
    detail::UartRegister(huart_, this);

    // 注册 RX 完成回调（无捕获 lambda → 函数指针）
    LIBRM_STM32_HAL_ASSERT(HAL_UART_RegisterRxEventCallback(huart_, [](UART_HandleTypeDef *h, u16 len) {
      if (auto *obj = detail::UartFind(h)) {
        obj->HalRxCpltCallback(len);
      }
    }));

    // 注册错误回调
    LIBRM_STM32_HAL_ASSERT(HAL_UART_RegisterCallback(huart_, HAL_UART_ERROR_CB_ID, [](UART_HandleTypeDef *h) {
      if (auto *obj = detail::UartFind(h)) {
        obj->HalErrorCallback();
      }
    }));

    // 注册 TX 完成回调（供 WriteAsync 使用）
    LIBRM_STM32_HAL_ASSERT(HAL_UART_RegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID, [](UART_HandleTypeDef *h) {
      if (auto *obj = detail::UartFind(h)) {
        obj->HalTxCpltCallback();
      }
    }));

    is_receiving_ = true;
    RestartRx();
  }

  void Stop() override {
    if (!is_receiving_) {
      return;
    }
    is_receiving_ = false;
    HAL_UART_AbortReceive(huart_);
  }

 private:
  void RestartRx() {
    auto &buf = rx_buf_[!buffer_selector_];
#ifdef HAL_DMA_MODULE_ENABLED
    if (async_rx_use_dma_) {
      LIBRM_STM32_HAL_ASSERT(HAL_UARTEx_ReceiveToIdle_DMA(huart_, buf.data(), buf.size()));
      __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
    } else
#endif
    {
      LIBRM_STM32_HAL_ASSERT(HAL_UARTEx_ReceiveToIdle_IT(huart_, buf.data(), buf.size()));
    }
  }

  void HalRxCpltCallback(u16 rx_len) override {
    if (!is_receiving_) {
      return;
    }
    RestartRx();  // 先重启接收，最小化数据丢失窗口
    etl::span<const u8> received{rx_buf_[buffer_selector_].data(), rx_len};
    for (auto &cb : rx_callbacks_) {
      if (cb) {
        cb(received);
      }
    }
    buffer_selector_ = !buffer_selector_;
  }

  void HalTxCpltCallback() override {
    if (!tx_done_callback_) {
      return;
    }
    auto cb = std::move(tx_done_callback_);
    tx_done_callback_ = nullptr;
    cb();
  }

  void HalErrorCallback() override {
    if (!is_receiving_) {
      return;
    }
    is_receiving_ = false;
    RestartRx();
    is_receiving_ = true;
  }

  etl::vector<SerialRxCallbackFunction, 10> rx_callbacks_;
  std::function<void()> tx_done_callback_{nullptr};
  UART_HandleTypeDef *huart_;
  bool async_tx_use_dma_;
  bool async_rx_use_dma_;
  std::array<u8, RxBufSize> rx_buf_[2]{};  ///< 双缓冲，栈/BSS 分配，无堆
  bool buffer_selector_{false};
  bool is_receiving_{false};
};

}  // namespace rm::hal::stm32

#endif  // HAL_UART_MODULE_ENABLED
#endif  // LIBRM_HAL_STM32_UART_HPP
