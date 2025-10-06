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
 * @file  librm/device/can_device.hpp
 * @brief CAN设备抽象类
 */

#ifndef LIBRM_CAN_DEVICE_HPP
#define LIBRM_CAN_DEVICE_HPP

#include <vector>

#include "librm/hal/can_interface.hpp"

namespace rm::device {

/**
 * @brief CAN设备的基类
 * @note  这个类是抽象基类，用来被CAN设备类继承，不能实例化。
 * @note  使用以下几个步骤，实现一个使用CAN总线通信的设备类：
 * @note  1. public继承CanDevice
 * @note  2. 代理构造，表明自己属于哪一条CAN总线、自己要接收哪些ID的报文
 * @note  3. 实现RxCallback(const hal::CanMsg *msg)函数，编写收到报文时的处理逻辑（实现时注意线程安全）
 * @note  框架对CAN总线和设备的封装使用观察者模式，CAN设备向CAN总线类"注册"自己，并且告知自己要接收哪些ID的报文；
 *        基于具体平台实现，CAN总线类会用轮询或接管中断的方式接收所有报文，每接收到一条报文，它就会寻找有没有注册过想要接收这条报文的设备，
 *        如果有，这个设备的RxCallback()函数就会被CAN总线类调用。
 */
class CanDevice {
 public:
  /**
   * @param can        CAN外设对象
   * @param rx_std_ids 这个设备的rx消息标准帧id列表
   */
  template <typename... IdList>
  explicit CanDevice(hal::CanInterface &can, IdList... rx_std_ids) : can_(&can) {
    (can.RegisterDevice(*this, rx_std_ids), ...);
    (rx_std_ids_.push_back(rx_std_ids), ...);
  }

  /**
   * @brief 析构
   */
  virtual ~CanDevice() {
    // 析构时，把这个设备对象从CAN总线的注册列表中移除
    can_->UnregisterDevice(*this);
  }

  /**
   * @brief 拷贝构造
   */
  inline CanDevice(const CanDevice &old) {
    can_ = old.can_;
    rx_std_ids_ = old.rx_std_ids_;
    // 设备对象复制之后，把新对象注册给CAN总线
    for (const auto id : old.rx_std_ids_) {
      can_->RegisterDevice(*this, id);
    }
  }
  /**
   * @brief 拷贝赋值
   */
  inline CanDevice &operator=(const CanDevice &old) {
    // 同上
    can_ = old.can_;
    rx_std_ids_ = old.rx_std_ids_;
    // 设备对象复制之后，把新对象注册给CAN总线
    for (const auto id : old.rx_std_ids_) {
      can_->RegisterDevice(*this, id);
    }
    return *this;
  }

  /**
   * @brief 移动构造
   */
  inline CanDevice(CanDevice &&old) {
    can_ = old.can_;
    rx_std_ids_ = std::move(old.rx_std_ids_);
    // 把被移动的设备对象从CAN总线的注册列表中移除，然后把新对象注册给CAN总线
    can_->UnregisterDevice(old);
    for (const auto &id : rx_std_ids_) {
      can_->RegisterDevice(*this, id);
    }
  }

  /**
   * @brief 移动赋值
   */
  inline CanDevice &operator=(CanDevice &&old) {
    can_ = old.can_;
    rx_std_ids_ = std::move(old.rx_std_ids_);
    // 同上
    can_->UnregisterDevice(old);
    for (const auto &id : rx_std_ids_) {
      can_->RegisterDevice(*this, id);
    }
    return *this;
  }

  virtual void RxCallback(const hal::CanMsg *msg) = 0;

 protected:
  hal::CanInterface *can_;
  std::vector<u16> rx_std_ids_;
};

}  // namespace rm::device

#endif  // LIBRM_CAN_DEVICE_HPP
