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
 * @file  librm/hal/linux/socketcan.cc
 * @brief SocketCAN类库
 * @todo  实现tx消息队列
 */

#include "socketcan.hpp"

#include <cstring>

#include "librm/core/exception.hpp"

namespace rm::hal::linux_ {

/**
 * @param dev CAN设备名，使用ifconfig -a查看
 */
SocketCan::SocketCan(const char *dev) : netdev_(dev) {}

SocketCan::~SocketCan() {
  recv_thread_running_.store(false);
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  Stop();
}

/**
 * @brief 初始化SocketCan
 */
void SocketCan::Begin() {
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if (socket_fd_ < 0) {
    rm::Throw(std::runtime_error(netdev_ + " open error"));
  }

  // 获取对应CAN netdev的标志并检查网络是否为UP状态
  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, netdev_.c_str());
  if (::ioctl(socket_fd_, SIOCGIFFLAGS, &ifr) < 0) {
    rm::Throw(std::runtime_error(netdev_ + " ioctl error"));
  }
  if (!(ifr.ifr_flags & IFF_UP)) {
    rm::Throw(std::runtime_error(netdev_ + " is not up"));
  }

  // 配置 Socket CAN 为阻塞IO
  int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  ::fcntl(socket_fd_, F_SETFL, flags | (~O_NONBLOCK));

  // 设置接收超时时间
  struct timeval timeout;
  timeout.tv_sec = 1;  // 超时时间为1s
  timeout.tv_usec = 0;
  if (::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    rm::Throw(std::runtime_error(netdev_ + " setsockopt error"));
  }

  // 指定can设备
  std::strcpy(interface_request_.ifr_name, netdev_.c_str());

  ::ioctl(socket_fd_, SIOCGIFINDEX, &interface_request_);
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = interface_request_.ifr_ifindex;

  // 配置过滤器，接收所有数据帧
  SetFilter(0, 0);

  // 将套接字与can设备绑定
  if (::bind(socket_fd_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
    rm::Throw(std::runtime_error(netdev_ + " bind error"));
  }

  // 启动接收线程
  recv_thread_running_.store(true);
  recv_thread_ = std::thread(&SocketCan::RecvThread, this);
}

/**
 * @param id   过滤器ID
 * @param mask 过滤器掩码
 */
void SocketCan::SetFilter(u16 id, u16 mask) {
  filter_.can_id = id;
  filter_.can_mask = mask;
  ::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter_, sizeof(filter_));
}

/**
 * @brief 立刻向总线上发送数据
 * @param id   数据帧ID
 * @param data 数据指针
 * @param size 数据长度/DLC
 */
void SocketCan::Write(u16 id, const u8 *data, usize size) {
  const auto frame = [id, data, size] {
    struct ::can_frame frame;
    frame.can_id = id;
    frame.can_dlc = size;
    std::copy(data, data + size, frame.data);
    return frame;
  }();
  while (::write(socket_fd_, &frame, sizeof(frame)) == -1) {
    // 如果写入失败，就一直重试，失败超过20次就抛出异常
  }
}

void SocketCan::Write() {
  // TODO: tx消息队列
}

void SocketCan::Enqueue(u16 id, const u8 *data, usize size, CanTxPriority priority) {
  // TODO: tx消息队列
}

/**
 * @brief 停止CAN外设
 */
void SocketCan::Stop() {
  ::close(socket_fd_);  // 关闭套接字
}

/**
 * @brief 接收线程，轮询接收报文并分发给对应ID的设备
 */
void SocketCan::RecvThread() {
  struct ::can_frame frame;
  while (recv_thread_running_.load()) {
    if ((::read(socket_fd_, &frame, sizeof(frame))) <= 0) {
      if (recv_thread_running_.load() == false) {
        break;
      } else {
        continue;
      }
    }
    // 根据报文ID找到对应的设备，如果没有设备订阅这个ID的报文，就丢弃这个报文
    auto receipient_devices = GetDeviceListByRxStdid(frame.can_id);
    if (receipient_devices.empty()) {
      continue;
    }
    // 封包，依次调用订阅者设备的回调函数
    const auto msg_packet = [&frame] {
      CanMsg msg_packet;
      msg_packet.rx_std_id = frame.can_id;
      msg_packet.dlc = frame.can_dlc;
      std::copy(frame.data, frame.data + frame.can_dlc, msg_packet.data.begin());
      return msg_packet;
    }();
    for (auto &device : receipient_devices) {
      device->RxCallback(&msg_packet);
    }
  }
}

}  // namespace rm::hal::linux_