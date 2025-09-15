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

#include "socketcan.h"

#include <cstring>

#include <iostream>

namespace rm::hal::linux_ {

/**
 * @param dev CAN设备名，使用ifconfig -a查看
 */
SocketCan::SocketCan(const char *dev) : dev_(dev) {
  // 创建线程池
  this->thread_pool_ = std::make_unique<core::ThreadPool>(SocketCan::kMaxThreads);
}

SocketCan::~SocketCan() { this->Stop(); }

/**
 * @brief 初始化SocketCan
 */
void SocketCan::Begin() {
  this->socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if (this->socket_fd_ < 0) {
    throw std::runtime_error(this->dev_ + " open error");
  }

  // 配置 Socket CAN 为阻塞IO
  int flags = fcntl(this->socket_fd_, F_GETFL, 0);
  fcntl(this->socket_fd_, F_SETFL, flags | (~O_NONBLOCK));

  // 指定can设备
  strcpy(this->interface_request_.ifr_name, this->dev_.c_str());

  ioctl(this->socket_fd_, SIOCGIFINDEX, &this->interface_request_);
  this->addr_.can_family = AF_CAN;
  this->addr_.can_ifindex = this->interface_request_.ifr_ifindex;

  // 配置过滤器，接收所有数据帧
  this->SetFilter(0, 0);

  // 将套接字与can设备绑定
  bind(this->socket_fd_, (struct sockaddr *)&this->addr_, sizeof(this->addr_));

  // 启动接收线程
  this->thread_pool_->enqueue([this]() { this->RecvThread(); });
}

/**
 * @param id   过滤器ID
 * @param mask 过滤器掩码
 */
void SocketCan::SetFilter(u16 id, u16 mask) {
  this->filter_.can_id = id;
  this->filter_.can_mask = mask;
  setsockopt(this->socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &this->filter_, sizeof(this->filter_));
}

/**
 * @brief 立刻向总线上发送数据
 * @param id   数据帧ID
 * @param data 数据指针
 * @param size 数据长度/DLC
 */
void SocketCan::Write(u16 id, const u8 *data, usize size) {
  struct ::can_frame frame;
  frame.can_id = id;
  frame.can_dlc = size;
  std::copy(data, data + size, frame.data);
  while (write(this->socket_fd_, &frame, sizeof(frame)) == -1) {
    // 如果写入失败，就一直重试
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
  close(this->socket_fd_);  // 关闭套接字
}

/**
 * @brief 接收线程，轮询接收报文并分发给对应ID的设备
 */
void SocketCan::RecvThread() {
  struct ::can_frame frame;
  for (;;) {
    if (read(this->socket_fd_, &frame, sizeof(frame)) <= 0) {
      continue;
    }
    // 如果接收成功，就异步调用RxCallbackCallWorker处理后续逻辑；之后立刻返回再次接收，防止漏收或延迟
    this->thread_pool_->enqueue(
        [this, frame]() { this->RxCallbackCallWorker(std::move(std::make_unique<struct can_frame>(frame))); });
  }
}

/**
 * @brief 发送线程，循环按优先级发送消息队列里的报文
 */
void SocketCan::SendThread() {
  // TODO: tx消息队列
}

/**
 * @brief worker线程，异步调用设备的Rx回调函数
 * @note  调用者为RecvThread
 */
void SocketCan::RxCallbackCallWorker(std::unique_ptr<struct ::can_frame> msg) {
  // 根据报文ID找到对应的设备，如果找到了的话就调用它的rx回调函数
  auto receipient_device = this->device_list_.find(msg->can_id);
  if (receipient_device != this->device_list_.end()) {
    // 封包
    CanMsg msg_packet;
    msg_packet.rx_std_id = msg->can_id;
    msg_packet.dlc = msg->can_dlc;
    std::copy(msg->data, msg->data + msg->can_dlc, msg_packet.data.begin());

    // 给这个设备加锁，然后调用它的回调函数
    {
      std::lock_guard<std::mutex> lock(receipient_device->second->mutex);
      receipient_device->second->dev->RxCallback(&msg_packet);
    }
  }
}

/**
 * @brief 注册CAN设备
 * @param device 设备对象
 * @param rx_stdid 这个设备的rx消息标准帧id
 */
void SocketCan::RegisterDevice(device::CanDevice &device, u32 rx_stdid) {
  // TODO: 冲突检查，如果已经有这个ID的设备了就抛出异常
  this->device_list_[rx_stdid] = new AsyncCanDevice(device);
}

}  // namespace rm::hal::linux_