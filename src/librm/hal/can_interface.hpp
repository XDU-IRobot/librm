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
#include <vector>
#include <unordered_map>

namespace rm::device {
class CanDevice;
}

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
 * @brief CAN接口类
 */
class CanInterface {
  friend class device::CanDevice;

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

  // 设备注册机制相关
 protected:
  /**
   * @brief 注册CAN设备
   * @param device    设备对象
   * @param rx_stdid  该设备要接收的标准帧ID
   * @note  同一设备在同一ID下只能注册一次（以UUID为准）
   */
  void RegisterDevice(device::CanDevice &device, u16 rx_stdid);

  /**
   * @brief 取消注册CAN设备
   * @param device 要取消注册的设备对象
   * @note  以UUID为准查找并移除，与设备对象的内存地址无关
   */
  void UnregisterDevice(device::CanDevice &device);

  /**
   * @brief 根据接收ID获取已注册的设备列表
   */
  const std::vector<device::CanDevice *> &GetDeviceListByRxStdid(u16 rx_stdid) const;

 private:
  std::vector<device::CanDevice *> empty_device_list_{};  ///< 用于在GetDeviceList中返回空列表
  std::unordered_map<u16, std::vector<device::CanDevice *>>
      rx_id_to_device_list_map_{};  ///< 订阅ID到设备对象列表的映射
};

}  // namespace rm::hal

#endif  // LIBRM_HAL_CAN_INTERFACE_HPP
