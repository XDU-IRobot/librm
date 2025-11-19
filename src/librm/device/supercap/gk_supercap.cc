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
 * @file  librm/device/supercap/gk_supercap.cc
 * @brief 港科超级电容
 */

#include "gk_supercap.hpp"

#include <cstring>

namespace rm::device {

GkSupercap::GkSupercap(hal::CanInterface &can) : CanDevice(can, 0x051) {}

/**
 * @brief 更新超级电容设置
 * @param tx_data   发送数据结构体
 * @note  发送数据前需要进行结构体初始化，保留位默认全都需要置零，其他成员变量根据需求赋值即可
 */
void GkSupercap::Update(const TxData &tx_data) {
  std::memcpy(tx_buf_, &tx_data, sizeof(tx_buf_));
  can_->Write(0x061, tx_buf_, 8);
}

/**
 * @brief CAN回调函数，解码收到的反馈报文
 * @param msg 收到的消息
 */
void GkSupercap::RxCallback(const hal::CanFrame *msg) { memcpy(&rx_data_, &msg->data[0], sizeof(RxData)); }

}  // namespace rm::device