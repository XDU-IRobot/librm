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
 * @file  librm/hal/linux/socketcan.hpp
 * @brief SocketCAN类库
 */

#ifndef LIBRM_HAL_LINUX_SOCKETCAN_HPP
#define LIBRM_HAL_LINUX_SOCKETCAN_HPP

#include <string>
#include <deque>
#include <memory>
#include <unordered_map>
#include <thread>
#include <atomic>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "librm/core/traits.hpp"
#include "librm/device/can_device.hpp"
#include "librm/hal/can_interface.hpp"

namespace rm::hal::linux_ {

class SocketCan : public hal::CanInterface, rm::detail::NonCopyable {
 public:
  explicit SocketCan(const char *dev);
  SocketCan() = default;
  ~SocketCan() override;

  void SetFilter(u16 id, u16 mask) override;
  void Write(u16 id, const u8 *data, usize size) override;
  void Begin() override;
  void Stop() override;

 private:
  void RecvThread();

  int socket_fd_{-1};                            ///< 当前使用的 CAN 套接字描述符
  struct ::sockaddr_can addr_ {};                ///< 绑定使用的 CAN 接口地址
  struct ::ifreq interface_request_ {};          ///< ioctl 查询时使用的结构体缓存
  struct ::can_filter filter_ {};                ///< 最近一次配置的过滤器参数
  std::string netdev_;                           ///< CAN 网络设备名（例如 "can0"）
  std::thread recv_thread_{};                    ///< 后台接收工作线程
  std::atomic_bool recv_thread_running_{false};  ///< 指示接收线程是否继续运行
};

}  // namespace rm::hal::linux_

#endif