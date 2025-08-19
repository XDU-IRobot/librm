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
 * @file  librm/device/supercap/supercap.cc
 * @brief 超级电容
 */

#include "supercap.h"

#include <cstring>

#include "librm/modules/algorithm/utils.hpp"

namespace rm::device {

using modules::algorithm::utils::Map;

SuperCap::SuperCap(hal::CanInterface &can) : CanDevice(can, 0x30) {}

/**
 * @return 电容两端电压(V)
 */
f32 SuperCap::voltage() const { return this->voltage_; }

/**
 * @return 电容充放电电流(A)
 */
f32 SuperCap::current() const { return this->current_; }

/**
 * @param error 错误标志
 * @return 是否存在该错误
 */
bool SuperCap::error(SuperCapError error) const { return error_flags_ & static_cast<u16>(error); }

/**
 * @brief 更新底盘功率缓冲
 * @param power_buffer 底盘功率缓冲
 * @note  裁判系统功率缓冲每0.1秒更新一次，请及时转发。如果直接固定以10Hz频率转发，可能有0.1s的延迟。
 *        可以在程序中以50Hz频率检测该项数据是否更新，更新了就发送，或直接以50Hz频率转发。
 *        注意请至少以10Hz频率发送，因为超级电容超过1秒未接收到主控的信息时会自动切断输出，等待下次接收到信息才会再次开始工作。
 */
void SuperCap::UpdateChassisBuffer(i16 power_buffer) {
  this->tx_buf_[1] = power_buffer;
  this->can_->Write(0x2e, this->tx_buf_, 8);
}

/**
 * @brief 更新超级电容设置
 * @param power_limit   底盘功率限制上限
 * @param output_limit  限制电容放电功率
 * @param input_limit   限制电容充电功率
 * @param power_switch  电容开关
 * @param enable_log    记录开关
 * @note  底盘功率限制上限为裁判系统给出的底盘可用的最大功率，机器人升级时请及时更新并发送。
 *        限制电容放电功率是指电容不会以超过这个值的功率放电，并不是指电容恒定以这个功率放电。
 *        电容工作时优先根据能量缓冲计算当前输出/输入功率，如果该功率超过限制功率，则让它等于限制功率。
 *        设置为负数时电容将以不小于该数绝对值的功率充电，达到降低底盘功率上限以加快电容充电的效果，默认值为300。
 *        限制电容充电功率不大于某个值可以留足能量缓冲给底盘将要进行的大功率动作做准备。不过一般不需要这么做，
 *        因为超级电容响应速度是非常快的（10ms左右），超级电容电量充足时不用担心底盘快速增大功率导致裁判系统超功率，默认值为150。并不需要把它改成底盘功率限制上限。
 *        电容开关：默认值为0，上电电容不会自动开始工作，需要发一次将电容开关置1的信息。机器人死亡时需置0。
 *        记录开关：默认值为0，当该值置1时开始记录数据，置0时停止记录数据。在OLED显示屏上会显示记录的数据结果。
 */
void SuperCap::UpdateSettings(i16 power_limit, i16 output_limit, i16 input_limit, bool power_switch, bool enable_log) {
  this->tx_buf_[0] = power_limit >> 8;
  this->tx_buf_[1] = power_limit;
  this->tx_buf_[2] = output_limit >> 8;
  this->tx_buf_[3] = output_limit;
  this->tx_buf_[4] = input_limit >> 8;
  this->tx_buf_[5] = input_limit;
  this->tx_buf_[6] = 0u;
  this->tx_buf_[7] = power_switch | (enable_log << 1);
  this->can_->Write(0x2f, this->tx_buf_, 8);
}

/**
 * @brief CAN回调函数，解码收到的反馈报文
 * @param msg 收到的消息
 */
void SuperCap::RxCallback(const hal::CanMsg *msg) {
  i16 current_int16 = (msg->data[2] << 8) | msg->data[3];
  this->voltage_ = Map((msg->data[0] << 8) | msg->data[1], -32000, 32000, 0, 30);
  this->current_ = Map(current_int16, -32000, 32000, -20, 20);
  this->error_flags_ = (msg->data[4] << 8) | msg->data[5];
}

}  // namespace rm::device
