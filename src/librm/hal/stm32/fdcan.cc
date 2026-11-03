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
 * @file  librm/hal/stm32/fdcan.cc
 * @brief fdCAN类库
 */

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_FDCAN_MODULE_ENABLED)

#include "fdcan.hpp"

#include <etl/string.h>
#include <etl/string_stream.h>

#include "librm/device/can_device.hpp"
#include "librm/hal/stm32/helper_macro.hpp"

namespace rm::hal::stm32 {

/**
 * @brief 存储FDCAN1、FDCAN2、FDCAN3对应的FdCan实例指针
 */
static FdCan *fdcan_instances[3] = {nullptr, nullptr, nullptr};

/**
 * @brief  根据FDCAN实例地址获取fdcan_instances数组索引
 * @param  hfdcan  HAL库的FDCAN_HandleTypeDef
 * @return 数组索引（0、1或2），如果不是有效的FDCAN实例则返回-1
 */
__attribute__((always_inline)) static inline int GetFdCanIndex(FDCAN_HandleTypeDef *hfdcan) {
  const auto instance = reinterpret_cast<uintptr_t>(hfdcan->Instance);
  if (instance == FDCAN1_BASE) return 0;
#if defined(FDCAN2_BASE)
  if (instance == FDCAN2_BASE) return 1;
#endif
#if defined(FDCAN3_BASE)
  if (instance == FDCAN3_BASE) return 2;
#endif
  return -1;
}

/**
 * @brief  注册给HAL库的FDCAN接收回调函数
 * @param  hfdcan  HAL库的FDCAN_HandleTypeDef
 * @param  RxFifo0ITs  中断标志
 */
void FdCanRxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan, u32 RxFifo0ITs) {
  const int fdcan_idx = GetFdCanIndex(hfdcan);
  if (fdcan_idx >= 0) {
    FdCan *instance = fdcan_instances[fdcan_idx];
    if (instance != nullptr) {
      instance->Fifo0MsgPendingCallback();
    }
  }
}

/**
 * @brief  Bus-Off错误回调函数，当总线进入Bus-Off状态时自动恢复
 * @param  hfdcan  HAL库的FDCAN_HandleTypeDef
 */
void FdCanErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, u32 ErrorStatusITs) {
  if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
    const int fdcan_idx = GetFdCanIndex(hfdcan);
    if (fdcan_idx >= 0) {
      FdCan *instance = fdcan_instances[fdcan_idx];
      if (instance != nullptr) {
        instance->Restart();
      }
    }
  }
}

/**
 * @param hfdcan HAL库的CAN_HandleTypeDef
 */
FdCan::FdCan(FDCAN_HandleTypeDef &hfdcan)
    : hfdcan_(&hfdcan),
      // 判断这个FDCAN句柄对应的外设是否被设置为FD模式
      fd_mode_available_{hfdcan.Init.FrameFormat == FDCAN_FRAME_FD_NO_BRS ||  //
                         hfdcan.Init.FrameFormat == FDCAN_FRAME_FD_BRS} {
  int idx = GetFdCanIndex(hfdcan_);
  if (idx >= 0) {
    fdcan_instances[idx] = this;
  }
}

FdCan::~FdCan() {
  Stop();
  int idx = GetFdCanIndex(hfdcan_);
  if (idx >= 0) {
    fdcan_instances[idx] = nullptr;
  }
}

FdCan::FdCan(FdCan &&other) noexcept : hfdcan_(other.hfdcan_), fd_mode_available_(other.fd_mode_available_) {
  // 把全局数组里的指针指向新的自己
  int idx = GetFdCanIndex(hfdcan_);
  if (idx >= 0) {
    fdcan_instances[idx] = this;
  }
}

FdCan &FdCan::operator=(FdCan &&other) noexcept {
  if (this != &other) {
    hfdcan_ = other.hfdcan_;
    const_cast<bool &>(fd_mode_available_) = other.fd_mode_available_;
    // 把全局数组里的指针指向新的自己
    int idx = GetFdCanIndex(hfdcan_);
    if (idx >= 0) {
      fdcan_instances[idx] = this;
    }
  }
  return *this;
}

/**
 * @brief 设置过滤器
 * @param id
 * @param mask
 */
void FdCan::SetFilter(u16 id, u16 mask) {
  FDCAN_FilterTypeDef can_filter_st;
  can_filter_st.IdType = FDCAN_STANDARD_ID;
  can_filter_st.FilterType = FDCAN_FILTER_MASK;
  can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  can_filter_st.FilterID1 = id;
  can_filter_st.FilterID2 = mask;
  can_filter_st.RxBufferIndex = 0;
  can_filter_st.IsCalibrationMsg = 0;
  if (reinterpret_cast<u32>(hfdcan_->Instance) == FDCAN1_BASE) {
    can_filter_st.FilterIndex = 0;
  }
  if (reinterpret_cast<u32>(hfdcan_->Instance) == FDCAN2_BASE) {
    can_filter_st.FilterIndex = 1;
  }
  if (reinterpret_cast<u32>(hfdcan_->Instance) == FDCAN3_BASE) {
    can_filter_st.FilterIndex = 2;
  }

  LIBRM_STM32_HAL_ASSERT(
      HAL_FDCAN_ConfigGlobalFilter(hfdcan_, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, DISABLE, DISABLE));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_ConfigFilter(hfdcan_, &can_filter_st));
}

namespace {

/// @brief 字节长度 -> STM32 HAL 使用的 FDCAN DLC 编码
constexpr u32 ToFdcanDlcCode(u8 dlc) {
  switch (dlc) {
    case 12u:
      return FDCAN_DLC_BYTES_12;
    case 16u:
      return FDCAN_DLC_BYTES_16;
    case 20u:
      return FDCAN_DLC_BYTES_20;
    case 24u:
      return FDCAN_DLC_BYTES_24;
    case 32u:
      return FDCAN_DLC_BYTES_32;
    case 48u:
      return FDCAN_DLC_BYTES_48;
    case 64u:
      return FDCAN_DLC_BYTES_64;
    default:
      return dlc;  // 0~8 字节时 DLC 编码与字节数相同
  }
}

}  // namespace

/**
 * @brief 立刻向总线上发送数据
 * @param id    标准帧ID
 * @param data  数据指针
 * @param size  数据长度
 * @note  已废弃，请改用 Write(const CanFrame &)。为保持旧行为，长度大于8字节时按FD帧发送；
 *        长度不合法或硬件不支持FD时抛异常
 */
void FdCan::Write(u16 id, const u8 *data, usize size) {
  if (size > 64) {
    Throw(rm::runtime_error("fdcan: CAN frame too long"));
  }
  WriteFrame(id, data, static_cast<u8>(size), size > 8);
}

/**
 * @brief 立刻向总线上发送一帧
 * @param frame 数据帧，std_id 为发送的目标ID，is_fd_frame 显式指定发FD帧还是经典CAN帧
 */
void FdCan::Write(const CanFrame &frame) {
  WriteFrame(frame.std_id, frame.data.data(), static_cast<u8>(frame.dlc), frame.is_fd_frame);
}

/**
 * @brief 发送一帧的公共实现
 * @param id          标准帧ID
 * @param data        数据指针
 * @param dlc         数据长度（字节）
 * @param is_fd_frame 是否发送FD帧
 * @note  长度必须是 CAN FD 规范允许的取值（0~8、12、16、20、24、32、48、64），否则直接抛异常，
 *        不做向上取整，也不做补零。
 */
void FdCan::WriteFrame(u16 id, const u8 *data, u8 dlc, bool is_fd_frame) {
  if (!CanFrame::IsLegalLength(dlc)) {
    Throw(rm::runtime_error("fdcan: dlc is not a legal CAN frame length"));
  }
  if (!is_fd_frame && (dlc > 8u)) {
    Throw(rm::runtime_error("fdcan: data too long for a classic CAN frame"));
  }
  if (is_fd_frame && !fd_mode_available_) {
    Throw(rm::runtime_error("fdcan: CAN FD frame requested, but the peripheral is configured for classic CAN"));
  }

  hal_tx_header_.FDFormat = is_fd_frame ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
  hal_tx_header_.DataLength = fd_mode_available_ ? ToFdcanDlcCode(dlc) : dlc;
  hal_tx_header_.Identifier = id;

  // 检查是否处于Bus-Off状态，如果是则尝试恢复
  FDCAN_ProtocolStatusTypeDef protocol_status;
  HAL_FDCAN_GetProtocolStatus(hfdcan_, &protocol_status);
  if (protocol_status.BusOff) {
    Restart();
    return;  // 这一帧放弃发送
  }

  // 等待TX FIFO有空闲位置并入队，若入队失败则继续等待重试，增加超时保护
  constexpr u32 kTimeoutLimit = 100000;
  constexpr u32 kMaxRetries = 10;
  u32 retry_count = 0;
  while (true) {
    u32 timeout_counter = 0;
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan_) == 0) {
      if (++timeout_counter > kTimeoutLimit) {
        HAL_FDCAN_GetProtocolStatus(hfdcan_, &protocol_status);
        if (protocol_status.BusOff) {
          Restart();
        }
        return;  // 放弃这一帧
      }
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &hal_tx_header_, data) == HAL_OK) {
      break;  // 入队成功
    }
    if (++retry_count >= kMaxRetries) {
      return;  // 超过最大重试次数，放弃这一帧
    }
  }
}

/**
 * @brief     校验所有已注册的FDCAN实例的Message RAM区间互不重叠
 *
 * @warning   若此处检查未通过，请去CubeMX修正各FDCAN实例的Message RAM配置，确保它们的起始地址和结束地址不重叠。
 *            不要尝试bypass这个检查强行运行程序。
 *
 * @note      H7系列上FDCAN1/2/3共享同一块10KB（0x2800字节）的Message RAM，各实例的
 *            起始位置由Init.MessageRAMOffset决定。HAL_FDCAN_Init()内部只检查"本实例不
 *            越过这块RAM的末尾"，并不检查实例之间的重叠；一旦配置重叠，多个FDCAN实例同时收发时就会静默出现数据错乱，难以排查。
 *            这里直接使用HAL已经算好的msgRam.StandardFilterSA和msgRam.EndAddress做两两
 *            区间检查（左闭右开，单位word），检测到异常就抛异常，不做降级、不做修正。
 */
void FdCan::ValidateMessageRamLayout() {
  struct RamRange {
    int index;
    u32 begin;
    u32 end;
  };

  constexpr int kInstanceCount = sizeof(fdcan_instances) / sizeof(fdcan_instances[0]);
  RamRange ranges[kInstanceCount] = {};
  int range_count = 0;

  for (int i = 0; i < kInstanceCount; ++i) {
    const FdCan *instance = fdcan_instances[i];
    if (instance == nullptr) {
      continue;
    }
    const FDCAN_HandleTypeDef *hfdcan = instance->hfdcan_;

    // HAL_FDCAN_Init()因Message RAM溢出等原因失败时会把State置为ERROR，
    // 此时msgRam里的地址不能反映实际配置，直接停机
    if (hfdcan->State == HAL_FDCAN_STATE_ERROR) {
      Throw(rm::runtime_error("fdcan: HAL_FDCAN_Init() failed, check MessageRAM element sizes and MessageRAMOffset"));
    }

    // 从未调用过HAL_FDCAN_Init()的句柄，其msgRam是零初始化的
    if (hfdcan->msgRam.EndAddress == 0u) {
      Throw(rm::runtime_error("fdcan: HAL_FDCAN_Init() has not been called for this handle"));
    }

    // 各元素数量均为0时区间为空，不参与重叠判断
    if (hfdcan->msgRam.EndAddress <= hfdcan->msgRam.StandardFilterSA) {
      continue;
    }

    ranges[range_count].index = i;
    ranges[range_count].begin = (hfdcan->msgRam.StandardFilterSA - SRAMCAN_BASE) / 4u;
    ranges[range_count].end = (hfdcan->msgRam.EndAddress - SRAMCAN_BASE) / 4u;
    ++range_count;
  }

  // 两个区间 [a0,a1) 与 [b0,b1) 相交 <=> a0 < b1 且 b0 < a1
  for (int i = 0; i < range_count; ++i) {
    for (int j = i + 1; j < range_count; ++j) {
      if ((ranges[i].begin < ranges[j].end) && (ranges[j].begin < ranges[i].end)) {
        etl::string<96> message;
        etl::string_stream stream(message);
        stream << "fdcan: Message RAM overlap: FDCAN" << (ranges[i].index + 1) << " [" << ranges[i].begin << ","
               << ranges[i].end << ") vs FDCAN" << (ranges[j].index + 1) << " [" << ranges[j].begin << ","
               << ranges[j].end << ") words";
        Throw(rm::runtime_error(message));
      }
    }
  }
}

/**
 * @brief 启动CAN外设
 */
void FdCan::Begin() {
  ValidateMessageRamLayout();

  // 如果这个FDCAN外设处于FD模式，就把发送头设置为FD格式
  if (fd_mode_available_) {
    hal_tx_header_.FDFormat = FDCAN_FD_CAN;
  } else {
    hal_tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
  }

  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_BUS_OFF, 0));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_RegisterRxFifo0Callback(hfdcan_, FdCanRxFifo0MsgPendingCallback));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_RegisterErrorStatusCallback(hfdcan_, FdCanErrorStatusCallback));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_Start(hfdcan_));
}

/**
 * @brief 停止CAN外设
 */
void FdCan::Stop() { LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_Stop(hfdcan_)); }

/**
 * @brief 重启外设，用于从Bus-Off状态恢复
 * @note  停止外设、重新激活中断通知、再启动外设
 */
void FdCan::Restart() const {
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_Stop(hfdcan_));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_BUS_OFF, 0));
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_Start(hfdcan_));
}

/**
 * @brief 利用Register callbacks机制，用这个函数替代HAL_CAN_RxFifo0MsgPendingCallback
 * @note  这个函数替代了HAL_CAN_RxFifo0MsgPendingCallback，HAL库会调用这个函数，不要手动调用
 */
void FdCan::Fifo0MsgPendingCallback() const {
  FDCAN_RxHeaderTypeDef rx_header;
  CanFrame rx_frame;
  LIBRM_STM32_HAL_ASSERT(HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header, rx_frame.data.data()));
  auto &device_list_ = GetDeviceListByRxStdid(rx_header.Identifier);
  if (device_list_.empty()) {
    return;
  }
  rx_frame.is_fd_frame = (rx_header.FDFormat == FDCAN_FD_CAN);
  rx_frame.std_id = rx_header.Identifier;
  if (rx_frame.is_fd_frame) {
    // 长于8字节的FD帧，DLC编码值并不等于实际的字节数，需要做转换
    if (rx_header.DataLength <= FDCAN_DLC_BYTES_8) {
      rx_frame.dlc = static_cast<CanFrame::Dlc>(rx_header.DataLength);  // 8字节以下DLC码和数据长度相同
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_12) {
      rx_frame.dlc = CanFrame::kDlc12;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_16) {
      rx_frame.dlc = CanFrame::kDlc16;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_20) {
      rx_frame.dlc = CanFrame::kDlc20;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_24) {
      rx_frame.dlc = CanFrame::kDlc24;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_32) {
      rx_frame.dlc = CanFrame::kDlc32;
    } else if (rx_header.DataLength == FDCAN_DLC_BYTES_48) {
      rx_frame.dlc = CanFrame::kDlc48;
    } else {
      rx_frame.dlc = CanFrame::kDlc64;
    }
  } else {
    rx_frame.dlc = static_cast<CanFrame::Dlc>(rx_header.DataLength);
  }
  for (auto &device : device_list_) {
    device->RxCallback(&rx_frame);
  }
}

}  // namespace rm::hal::stm32

#endif