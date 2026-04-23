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
 * @file  librm/hal/linux/serial.cc
 * @brief 串口类库
 */

#include "serial.hpp"

#include <etl/span.h>

#include "librm/core/exception.hpp"

namespace rm::hal::linux_ {

Serial::Serial(boost::asio::serial_port &&serial_port, usize rx_buffer_size)
    : serial_port_{std::move(serial_port)}, rx_buffer_{std::vector<u8>(rx_buffer_size)} {}

Serial::~Serial() {
  Stop();
  if (serial_port_.is_open()) {
    serial_port_.close();
  }
}

Serial::Serial(Serial &&other)
    : serial_port_{std::move(other.serial_port_)},
      rx_callbacks_{std::move(other.rx_callbacks_)},
      rx_thread_{std::move(other.rx_thread_)},
      rx_thread_running_{other.rx_thread_running_.load()},
      rx_buffer_{std::move(other.rx_buffer_)} {}

Serial &Serial::operator=(Serial &&other) {
  if (this != &other) {
    serial_port_ = std::move(other.serial_port_);
    rx_callbacks_ = std::move(other.rx_callbacks_);
    rx_thread_ = std::move(other.rx_thread_);
    rx_thread_running_.store(other.rx_thread_running_.load());
    rx_buffer_ = std::move(other.rx_buffer_);
  }
  return *this;
}

// SyncWritablevoid Serial::Write(const u8 *data, usize size, u32 timeout_ms) {
// timeout_ms is ignored in posix serial atm, standard boost::asio
// doesn't have an easy way to specify write timeout without async ops.
if (!serial_port_.is_open()) {
  Throw(std::runtime_error("boost::asio::serial_port object is not opened"));
  return;
}
try {
  serial_port_.write_some(boost::asio::buffer(data, size));
} catch (const std::exception &e) {
  Throw(std::runtime_error("Failed to write to serial port: " + std::string(e.what())));
}
}

// AsyncReadablevoid Serial::AttachRxCallback(SerialRxCallbackFunction callback) {
rx_callbacks_.emplace_back(std::move(callback));
}

void Serial::Start() {
  if (!serial_port_.is_open()) {
    Throw(std::runtime_error("boost::asio::serial_port object is not opened"));
    return;
  }
  if (rx_thread_.joinable()) {
    // 已经 Start() 过了
    return;
  }

  rx_thread_running_.store(true);
  rx_thread_ = std::thread{[this] {
    while (serial_port_.is_open() && rx_thread_running_.load()) {
      const usize bytes_read = serial_port_.read_some(boost::asio::buffer(rx_buffer_));
      if (bytes_read == 0) continue;

      etl::span<const u8> received{rx_buffer_.data(), bytes_read};
      for (auto &cb : rx_callbacks_) {
        if (cb) cb(received);
      }
    }
  }};
}

void Serial::Stop() {
  rx_thread_running_.store(false);
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

}  // namespace rm::hal::linux_