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
 * @file  librm/hal/linux/serial.h
 * @brief 串口类库
 */

#ifndef LIBRM_HAL_LINUX_SERIAL_H
#define LIBRM_HAL_LINUX_SERIAL_H

#include <thread>

#include "boost/asio.hpp"

#include "librm/hal/serial_interface.h"

namespace rm::hal::linux_ {

/**
 * @brief 基于boost::asio::serial_port的串口二次封装
 */
class Serial : public hal::SerialInterface {
 public:
  Serial() = delete;
  ~Serial() override;

  /**
   * @note
   * 必须传进一个serial_port的右值引用（移动构造），以便Serial类完全接管这个对象
   *
   * @param boost_serial_port_object  boost::asio::serial_port对象的右值引用
   * @param rx_buffer_size            接收缓冲区大小
   */
  Serial(boost::asio::serial_port &&boost_serial_port_object, usize rx_buffer_size);

  /**
   * @brief 打开串口，开始接收
   */
  void Begin() override;

  /**
   * @brief 向串口写入数据
   * @param data  数据指针
   * @param size  数据长度
   */
  void Write(const u8 *data, usize size) override;

  /**
   * @brief 注册接收完成回调函数
   * @note  出现缓冲区满、超时未收到数据两种情况时，这个回调函数会被调用
   * @param callback  接收完成回调函数
   */
  void AttachRxCallback(SerialRxCallbackFunction &callback) override;
  [[nodiscard]] const std::vector<u8> &rx_buffer() const override;

  /**
   * @brief   获取这个Serial接管的 boost::asio::serial_port 对象
   * @return  boost::asio::serial_port 对象的引用
   */
  auto &boost_serial_port_object() { return serial_port_; }

 private:
  boost::asio::serial_port serial_port_;           ///< 接管的boost::asio::serial_port对象
  SerialRxCallbackFunction rx_callback_{nullptr};  ///< 接收完成回调函数
  std::thread rx_thread_{};                        ///< 接收线程
  std::atomic<bool> rx_thread_running_{
      false};  ///< 控制接收线程是否运行，Serial对象析构时会将其设置为false，从而结束接收线程
  std::vector<u8> rx_buffer_;  ///< 接收缓冲区
};

}  // namespace rm::hal::linux_

#endif