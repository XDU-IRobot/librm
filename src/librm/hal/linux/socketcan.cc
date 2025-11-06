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

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdio>
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
  // 打开目标接口的原始 CAN 套接字
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if (socket_fd_ < 0) {
    rm::Throw(std::runtime_error(netdev_ + " open error"));
  }

  // 请求启用 CAN FD，这样内核才会接收 64 字节数据段
  const int enable_canfd = 1;
  if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
    const auto error_message = netdev_ + " enable CAN FD error";
    Stop();
    rm::Throw(std::runtime_error(error_message));
  }

  // 获取对应CAN netdev的标志并检查网络是否为UP状态
  struct ifreq ifr {};
  // 读取当前链路状态，确认接口处于工作状态
  std::strcpy(ifr.ifr_name, netdev_.c_str());
  if (::ioctl(socket_fd_, SIOCGIFFLAGS, &ifr) < 0) {
    const auto error_message = netdev_ + " ioctl error";
    Stop();
    rm::Throw(std::runtime_error(error_message));
  }
  if (!(ifr.ifr_flags & IFF_UP)) {
    const auto error_message = netdev_ + " is not up";
    Stop();
    rm::Throw(std::runtime_error(error_message));
  }

  // 配置 Socket CAN 为阻塞IO
  int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(socket_fd_, F_SETFL, flags & ~O_NONBLOCK);
  }

  // 设置接收超时时间
  struct timeval timeout {
    1, 0
  };
  if (::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    const auto error_message = netdev_ + " setsockopt error";
    Stop();
    rm::Throw(std::runtime_error(error_message));
  }

  // 指定can设备
  interface_request_ = {};
  // 查询接口索引，供后续绑定使用
  std::strcpy(interface_request_.ifr_name, netdev_.c_str());
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &interface_request_) < 0) {
    const auto error_message = netdev_ + " get ifindex error";
    Stop();
    rm::Throw(std::runtime_error(error_message));
  }
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = interface_request_.ifr_ifindex;

  // 将套接字与can设备绑定
  if (::bind(socket_fd_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
    const auto error_message = netdev_ + " bind error";
    Stop();
    rm::Throw(std::runtime_error(error_message));
  }

  // 应用过滤器设置
  // 在用户设置更严格掩码前，过滤器默认全通
  SetFilter(filter_.can_id, filter_.can_mask);

  // 启动接收线程
  // 后台线程阻塞在 read() 上并将报文转发给设备
  recv_thread_running_.store(true);
  recv_thread_ = std::thread(&SocketCan::RecvThread, this);
}

/**
 * @param id   过滤器ID
 * @param mask 过滤器掩码
 */
void SocketCan::SetFilter(u16 id, u16 mask) {
  // 缓存过滤器参数，便于下次 Begin() 时重新设置
  filter_.can_id = id & CAN_SFF_MASK;
  filter_.can_mask = mask & CAN_SFF_MASK;
  if (socket_fd_ < 0) {
    // 暂存过滤配置，等待 Begin() 打开套接字后再写入内核
    return;
  }
  if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter_, sizeof(filter_)) < 0) {
    rm::Throw(std::runtime_error(netdev_ + " set filter error"));
  }
}

/**
 * @brief 立刻向总线上发送数据
 * @param id   数据帧ID
 * @param data 数据指针
 * @param size 数据长度/DLC
 */
void SocketCan::Write(u16 id, const u8 *data, usize size) {
  if (id > CAN_SFF_MASK) {
    rm::Throw(std::runtime_error("invalid standard id"));
  }

  if (size > CANFD_MAX_DLEN) {
    rm::Throw(std::runtime_error("payload too large"));
  }

  const bool use_fd_frame = size > CAN_MAX_DLEN;
  constexpr int kMaxRetry = 20;

  if (use_fd_frame) {
    struct ::canfd_frame frame {};
    frame.can_id = id & CAN_SFF_MASK;
    frame.len = static_cast<__u8>(size);
    frame.flags = 0;
    std::copy_n(data, size, frame.data);
    for (int attempt = 0; attempt < kMaxRetry; ++attempt) {
      if (::write(socket_fd_, &frame, sizeof(frame)) >= 0) {
        return;
      }
      // 驱动反馈暂时性的阻塞时重试写入
      if (errno != EAGAIN && errno != EINTR) {
        break;
      }
    }
  } else {
    struct ::can_frame frame {};
    frame.can_id = id & CAN_SFF_MASK;
    frame.can_dlc = static_cast<__u8>(size);
    std::copy_n(data, size, frame.data);
    for (int attempt = 0; attempt < kMaxRetry; ++attempt) {
      if (::write(socket_fd_, &frame, sizeof(frame)) >= 0) {
        return;
      }
      if (errno != EAGAIN && errno != EINTR) {
        break;
      }
    }
  }

  // 连续失败后抛出异常，交由上层处理
  rm::Throw(std::runtime_error(netdev_ + " write error"));
}

/**
 * @brief 停止CAN外设
 */
void SocketCan::Stop() {
  if (socket_fd_ >= 0) {
    // 关闭文件描述符可以唤醒 RecvThread 并结束阻塞读取
    ::close(socket_fd_);  // 关闭套接字
    socket_fd_ = -1;
  }
}

/**
 * @brief 接收线程，轮询接收报文并分发给对应ID的设备
 */
void SocketCan::RecvThread() {
  // 预留足够大的缓冲区以容纳完整的 CAN FD 帧
  std::array<std::uint8_t, sizeof(::canfd_frame)> frame_storage{};
  while (recv_thread_running_.load()) {
    // 阻塞读取在有报文或 1 秒超时后被唤醒
    const auto bytes_read = ::read(socket_fd_, frame_storage.data(), frame_storage.size());
    if (bytes_read <= 0) {
      if (!recv_thread_running_.load()) {
        break;
      }
      if (errno == EAGAIN || errno == EINTR) {
        continue;
      }
      if (errno == 0) {
        printf("socket read returned 0 bytes\n");
      } else {
        printf("socket read error: %s\n", std::strerror(errno));
      }
      continue;
    }

    const bool is_fd_frame = bytes_read == static_cast<ssize_t>(CANFD_MTU);
    if (!is_fd_frame && bytes_read != static_cast<ssize_t>(CAN_MTU)) {
      printf("unexpected CAN frame size: %zd\n", bytes_read);
      continue;
    }

    const auto &base_frame = *reinterpret_cast<const ::can_frame *>(frame_storage.data());
    if (base_frame.can_id & CAN_ERR_FLAG) {
      continue;
    }

    if (base_frame.can_id & CAN_EFF_FLAG) {
      continue;  // 当前接口只处理标准ID
    }

    const auto rx_id = static_cast<u16>(base_frame.can_id & CAN_SFF_MASK);
    auto receipient_devices = GetDeviceListByRxStdid(rx_id);
    if (receipient_devices.empty()) {
      continue;
    }

    // 将原始帧转换为 CanFrame 并分发给已注册设备
    const auto msg_packet = [&frame_storage, is_fd_frame, rx_id] {
      CanFrame msg_packet;
      msg_packet.rx_std_id = rx_id;
      msg_packet.is_fd_frame = is_fd_frame;
      if (is_fd_frame) {
        const auto &frame = *reinterpret_cast<const ::canfd_frame *>(frame_storage.data());
        const auto payload_len = std::min(static_cast<usize>(frame.len), static_cast<usize>(CANFD_MAX_DLEN));
        msg_packet.dlc = static_cast<u32>(payload_len);
        std::copy_n(frame.data, payload_len, msg_packet.data.begin());
      } else {
        const auto &frame = *reinterpret_cast<const ::can_frame *>(frame_storage.data());
        const bool is_remote_frame = (frame.can_id & CAN_RTR_FLAG) != 0;
        // 按规范 RTR 帧只声明 DLC 不携带数据
        const auto payload_len =
            is_remote_frame ? 0U : std::min(static_cast<usize>(frame.can_dlc & 0x0F), static_cast<usize>(CAN_MAX_DLEN));
        msg_packet.dlc = static_cast<u32>(payload_len);
        std::copy_n(frame.data, payload_len, msg_packet.data.begin());
      }
      return msg_packet;
    }();

    for (auto &device : receipient_devices) {
      device->RxCallback(&msg_packet);
    }
  }
}

}  // namespace rm::hal::linux_