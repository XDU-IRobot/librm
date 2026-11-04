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
 * @file    librm/hal/can_interface.hpp
 * @brief   CAN的接口类
 */

#ifndef LIBRM_HAL_CAN_INTERFACE_HPP
#define LIBRM_HAL_CAN_INTERFACE_HPP

#include "librm/core/typedefs.hpp"

#include <array>
#include <cstddef>

#include <etl/observer.h>

namespace rm::hal {

struct CanFrame {
  std::array<u8, 64> data;  ///< 数据
  u16 std_id;               ///< 标准帧ID（收时为帧的来源ID，发时为帧的目标ID）
  enum Dlc : u8 {
    kDlc0 = 0,
    kDlc1 = 1,
    kDlc2 = 2,
    kDlc3 = 3,
    kDlc4 = 4,
    kDlc5 = 5,
    kDlc6 = 6,
    kDlc7 = 7,
    kDlc8 = 8,
    kDlc12 = 12,
    kDlc16 = 16,
    kDlc20 = 20,
    kDlc24 = 24,
    kDlc32 = 32,
    kDlc48 = 48,
    kDlc64 = 64,
  };
  Dlc dlc;                  ///< 数据长度码，取值限定为 Dlc 枚举中的值
  bool is_fd_frame{false};  ///< 是否为FD帧

  /**
   * @brief 判断字节长度是否为 CAN FD 规范允许的 DLC 取值（0~8、12、16、20、24、32、48、64）
   * @param length 数据长度（字节）
   * @note  经典CAN的合法长度（0~8）是本集合的子集，因此各后端可以统一用本函数做校验
   */
  static constexpr bool IsLegalLength(usize length) {
    return (length <= 8u) || (length == 12u) || (length == 16u) || (length == 20u) || (length == 24u) ||
           (length == 32u) || (length == 48u) || (length == 64u);
  }
};

/**
 * @brief 单条CAN总线上最多允许的观察者数量
 * @note  包含所有CanDevice以及总线统计等非设备观察者，超出的注册会被忽略
 */
inline constexpr usize kMaxCanObservers = 64;

/**
 * @brief CAN帧观察者接口
 * @note  总线收到任何报文都会通知所有已注册的观察者，由观察者自行决定要处理哪些报文
 */
using CanFrameObserver = etl::observer<const CanFrame &>;

/**
 * @brief CAN接口类
 * @note  采用观察者模式：所有关心CAN报文的实体都向总线注册自己，注销时自动移除。
 */
class CanInterface : public etl::observable<CanFrameObserver, kMaxCanObservers> {
 public:
  virtual ~CanInterface() = default;

  /**
   * @brief 立即向总线上发送数据。当数据长度大于8字节，且硬件支持时，发送FD帧；否则发送经典CAN 2.0帧
   * @deprecated    老接口，无法显式指定发送FD帧或经典CAN帧，未来将逐步废弃并删除，建议使用Write(const CanFrame &)
   * @param id      数据帧ID
   * @param data    数据指针
   * @param size    数据长度
   * @warning       长度必须是合法的CAN帧长度（0~8、12、16、20、24、32、48、64），否则抛出异常
   */
  virtual void Write(u16 id, const u8 *data, usize size) = 0;

  /**
   * @brief   立即向总线上发送数据。
   * @warning frame.dlc 必须是合法的CAN帧长度（0~8、12、16、20、24、32、48、64），否则抛出异常；
   *          若 frame.is_fd_frame 为 true 但硬件不支持（或被配置为）发送FD帧，同样抛出异常。
   * @param   frame   数据帧
   */
  virtual void Write(const CanFrame &frame) = 0;

  /**
   * @brief 设置过滤器
   * @param id
   * @param mask
   */
  virtual void SetFilter(u16 id, u16 mask) = 0;

  /**
   * @brief 启动CAN外设
   */
  virtual void Begin() = 0;

  /**
   * @brief 停止CAN外设
   */
  virtual void Stop() = 0;
};

}  // namespace rm::hal

#endif  // LIBRM_HAL_CAN_INTERFACE_HPP
