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
 * @file  librm/device/can_device.hpp
 * @brief CAN设备抽象类
 */

#ifndef LIBRM_CAN_DEVICE_HPP
#define LIBRM_CAN_DEVICE_HPP

#include <utility>

#include "librm/hal/can_interface.hpp"
#include "librm/device/device.hpp"

namespace rm::device {

/**
 * @brief CAN设备的基类
 * @note  这个类是抽象基类，用来被CAN设备类继承，不能实例化。
 * @note  使用以下几个步骤，实现一个使用CAN总线通信的设备类：
 * @note  1. public继承CanDevice
 * @note  2. 代理构造，表明自己属于哪一条CAN总线
 * @note  3. 实现RxCallback(const hal::CanFrame *msg)函数，编写收到报文时的处理逻辑（实现时注意线程安全）
 * @note  框架对CAN总线和设备的封装使用观察者模式，CAN设备构造时向CAN总线注册自己，析构时注销。
 *        总线每收到一条报文都会交给全部设备，设备可以自行挑选其中关心的报文处理。
 */
class CanDevice : public Device, public hal::CanFrameObserver {
 public:
  /**
   * @param can CAN外设对象
   */
  explicit CanDevice(hal::CanInterface &can) : can_(&can) { can_->add_observer(*this); }

  /**
   * @brief 析构
   */
  ~CanDevice() override { can_->remove_observer(*this); }

  /**
   * @brief 拷贝构造
   */
  CanDevice(const CanDevice &old) : Device(old), can_(old.can_) {
    // 设备对象复制之后，把新对象注册给CAN总线
    can_->add_observer(*this);
  }

  /**
   * @brief 拷贝赋值
   */
  CanDevice &operator=(const CanDevice &old) {
    if (this != &old) {
      if (can_ != nullptr) {
        can_->remove_observer(*this);
      }
      Device::operator=(old);
      can_ = old.can_;
      can_->add_observer(*this);
    }
    return *this;
  }

  /**
   * @brief 移动构造
   */
  CanDevice(CanDevice &&old) noexcept : Device(std::move(old)), can_(old.can_) {
    // 身份转移到新对象，先注销旧对象再注册新对象
    can_->remove_observer(old);
    can_->add_observer(*this);
  }

  /**
   * @brief 移动赋值
   */
  CanDevice &operator=(CanDevice &&old) noexcept {
    if (this != &old) {
      if (can_ != nullptr) {
        can_->remove_observer(*this);
      }
      Device::operator=(std::move(old));
      can_ = old.can_;
      can_->remove_observer(old);
      can_->add_observer(*this);
    }
    return *this;
  }

  /**
   * @brief 收到CAN报文的处理入口
   * @param msg 收到的报文
   * @note  总线收到任何报文都会调用这个函数，实现时可以按 std_id 或数据内容自行挑选关心的报文处理。
   */
  virtual void RxCallback(const hal::CanFrame *msg) = 0;

 protected:
  hal::CanInterface *can_{nullptr};

 private:
  /// @brief 观察者入口，直接转发给RxCallback
  void notification(const hal::CanFrame &msg) final { RxCallback(&msg); }
};

}  // namespace rm::device

#endif  // LIBRM_CAN_DEVICE_HPP
