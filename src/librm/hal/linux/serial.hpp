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
 * @file  librm/hal/linux/serial.hpp
 * @brief 串口类库
 */

#ifndef LIBRM_HAL_LINUX_SERIAL_HPP
#define LIBRM_HAL_LINUX_SERIAL_HPP

#include <atomic>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <etl/vector.h>

#include "librm/hal/serial_interface.hpp"

namespace rm::hal::linux_ {

/**
 * @brief 基于 boost::asio::serial_port 的串口封装
 *
 * 实现 SerialInterface（SyncWritable + AsyncReadable）:
 *   - Write()  : 同步写，blocking
 *   - Start()  : 启动后台接收线程
 *   - Stop()   : 停止后台接收线程
 *   - AttachRxCallback : 支持注册多个回调
 */
class Serial : public hal::SerialInterface {
 public:
  Serial() = delete;
  ~Serial() override;

  // 禁止复制构造
  Serial(const Serial &) = delete;
  Serial &operator=(const Serial &) = delete;

  // 移动构造
  Serial(Serial &&other);
  Serial &operator=(Serial &&other);

  /**
   * @note
   * 必须传进一个serial_port的右值引用（移动构造），以便Serial类完全接管这个对象
   *
   * @param boost_serial_port_object  boost::asio::serial_port对象的右值引用
   * @param rx_buffer_size            接收缓冲区大小
   */
  Serial(boost::asio::serial_port &&boost_serial_port_object, usize rx_buffer_size);

  // SyncWritable
  void Write(const u8 *data, usize size, u32 timeout_ms) override;

  // AsyncReadable
  void AttachRxCallback(SerialRxCallbackFunction callback) override;
  void Start() override;
  void Stop() override;

  /**
   * @brief   获取这个Serial接管的 boost::asio::serial_port 对象
   * @return  boost::asio::serial_port 对象的引用
   */
  auto &boost_serial_port_object() { return serial_port_; }

 private:
  boost::asio::serial_port serial_port_;                ///< 接管的boost::asio::serial_port对象
  std::vector<SerialRxCallbackFunction> rx_callbacks_;  ///< Linux 平台保留 std::vector（非嵌入式）
  std::thread rx_thread_{};                             ///< 接收线程
  std::atomic<bool> rx_thread_running_{
      false};  ///< 控制接收线程是否运行，Serial对象析构时会将其设置为false，从而结束接收线程
  std::vector<u8> rx_buffer_;  ///< 接收缓冲区
};

}  // namespace rm::hal::linux_

#endif