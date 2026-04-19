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
 * @file  librm/hal/stm32/mcp2515.hpp
 * @brief MCP2515 CAN控制器封装
 */

#ifndef LIBRM_HAL_STM32_MCP2515_HPP
#define LIBRM_HAL_STM32_MCP2515_HPP

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_SPI_MODULE_ENABLED) && defined(HAL_GPIO_MODULE_ENABLED)

#include "librm/core/traits.hpp"
#include "librm/core/typedefs.hpp"
#include "librm/hal/can_interface.hpp"

namespace rm::hal::stm32 {

/**
 * @brief 基于HAL SPI + EXTI中断的MCP2515 CAN控制器封装
 * @note  INT引脚低电平有效，外部在EXTI中断触发并清除pending后调用ExtiIrqHandler()
 */
class Mcp2515 : public CanInterface, rm::detail::NonCopyable {
 public:
  /**
   * @brief CAN波特率（基于8MHz晶振）
   */
  enum class Bitrate : u8 {
    k125kbps,
    k250kbps,
    k500kbps,
    k1Mbps,
  };

  /**
   * @param hspi           SPI句柄
   * @param cs_gpio_port   CS引脚GPIO端口
   * @param cs_pin         CS引脚
   * @param bitrate        波特率（默认500kbps）
   */
  explicit Mcp2515(SPI_HandleTypeDef &hspi, GPIO_TypeDef *cs_gpio_port, u16 cs_pin,
                   Bitrate bitrate = Bitrate::k500kbps);
  ~Mcp2515() override;

  Mcp2515(Mcp2515 &&other) noexcept;
  Mcp2515 &operator=(Mcp2515 &&other) noexcept;

  void Write(u16 id, const u8 *data, usize size) override;
  void SetFilter(u16 id, u16 mask) override;
  void Begin() override;
  void Stop() override;

  /**
   * @brief EXTI中断入口，外部清除pending后调用
   */
  void ExtiIrqHandler();

 private:
  void HandleRxInterrupt();

  // SPI指令
  static constexpr u8 kCmdReset = 0xC0;
  static constexpr u8 kCmdRead = 0x03;
  static constexpr u8 kCmdReadRxB0Sidh = 0x90;
  static constexpr u8 kCmdReadRxB0D0 = 0x92;
  static constexpr u8 kCmdReadRxB1Sidh = 0x94;
  static constexpr u8 kCmdReadRxB1D0 = 0x96;
  static constexpr u8 kCmdWrite = 0x02;
  static constexpr u8 kCmdLoadTxB0Sidh = 0x40;
  static constexpr u8 kCmdLoadTxB0D0 = 0x41;
  static constexpr u8 kCmdLoadTxB1Sidh = 0x42;
  static constexpr u8 kCmdLoadTxB1D0 = 0x43;
  static constexpr u8 kCmdLoadTxB2Sidh = 0x44;
  static constexpr u8 kCmdLoadTxB2D0 = 0x45;
  static constexpr u8 kCmdRtsTx0 = 0x81;
  static constexpr u8 kCmdRtsTx1 = 0x82;
  static constexpr u8 kCmdRtsTx2 = 0x84;
  static constexpr u8 kCmdReadStatus = 0xA0;
  static constexpr u8 kCmdRxStatus = 0xB0;
  static constexpr u8 kCmdBitModify = 0x05;

  // 寄存器地址
  static constexpr u8 kRegCanStat = 0x0E;
  static constexpr u8 kRegCanCtrl = 0x0F;
  static constexpr u8 kRegTec = 0x1C;
  static constexpr u8 kRegRec = 0x1D;
  static constexpr u8 kRegCnf3 = 0x28;
  static constexpr u8 kRegCnf2 = 0x29;
  static constexpr u8 kRegCnf1 = 0x2A;
  static constexpr u8 kRegCanInte = 0x2B;
  static constexpr u8 kRegCanIntf = 0x2C;
  static constexpr u8 kRegEflg = 0x2D;

  static constexpr u8 kRegTxB0Ctrl = 0x30;
  static constexpr u8 kRegTxB1Ctrl = 0x40;
  static constexpr u8 kRegTxB2Ctrl = 0x50;
  static constexpr u8 kRegRxB0Ctrl = 0x60;
  static constexpr u8 kRegRxB0Sidh = 0x61;
  static constexpr u8 kRegRxB1Ctrl = 0x70;
  static constexpr u8 kRegRxB1Sidh = 0x71;

  // 过滤器/掩码寄存器
  static constexpr u8 kRegRxF0Sidh = 0x00;
  static constexpr u8 kRegRxF0Sidl = 0x01;
  static constexpr u8 kRegRxF0Eid8 = 0x02;
  static constexpr u8 kRegRxF0Eid0 = 0x03;
  static constexpr u8 kRegRxF1Sidh = 0x04;
  static constexpr u8 kRegRxF1Sidl = 0x05;
  static constexpr u8 kRegRxF1Eid8 = 0x06;
  static constexpr u8 kRegRxF1Eid0 = 0x07;
  static constexpr u8 kRegRxF2Sidh = 0x08;
  static constexpr u8 kRegRxF2Sidl = 0x09;
  static constexpr u8 kRegRxF2Eid8 = 0x0A;
  static constexpr u8 kRegRxF2Eid0 = 0x0B;
  static constexpr u8 kRegRxF3Sidh = 0x10;
  static constexpr u8 kRegRxF3Sidl = 0x11;
  static constexpr u8 kRegRxF3Eid8 = 0x12;
  static constexpr u8 kRegRxF3Eid0 = 0x13;
  static constexpr u8 kRegRxF4Sidh = 0x14;
  static constexpr u8 kRegRxF4Sidl = 0x15;
  static constexpr u8 kRegRxF4Eid8 = 0x16;
  static constexpr u8 kRegRxF4Eid0 = 0x17;
  static constexpr u8 kRegRxF5Sidh = 0x18;
  static constexpr u8 kRegRxF5Sidl = 0x19;
  static constexpr u8 kRegRxF5Eid8 = 0x1A;
  static constexpr u8 kRegRxF5Eid0 = 0x1B;
  static constexpr u8 kRegRxM0Sidh = 0x20;
  static constexpr u8 kRegRxM0Sidl = 0x21;
  static constexpr u8 kRegRxM0Eid8 = 0x22;
  static constexpr u8 kRegRxM0Eid0 = 0x23;
  static constexpr u8 kRegRxM1Sidh = 0x24;
  static constexpr u8 kRegRxM1Sidl = 0x25;
  static constexpr u8 kRegRxM1Eid8 = 0x26;
  static constexpr u8 kRegRxM1Eid0 = 0x27;

  // RX状态
  static constexpr u8 kMsgInRxB0 = 0x01;
  static constexpr u8 kMsgInRxB1 = 0x02;
  static constexpr u8 kMsgInBothBuffers = 0x03;

  // SPI底层
  void CsLow();
  void CsHigh();
  void SpiTx(u8 data);
  void SpiTxBuffer(const u8 *buffer, u8 length);
  u8 SpiRx();
  void SpiRxBuffer(u8 *buffer, u8 length);

  // 寄存器操作
  void ChipReset();
  u8 ReadRegister(u8 address);
  void ReadRxSequence(u8 instruction, u8 *data, u8 length);
  void WriteRegister(u8 address, u8 data);
  void WriteRegisterSequence(u8 start_address, u8 end_address, u8 *data);
  void LoadTxSequence(u8 instruction, u8 *id_reg, u8 dlc, const u8 *data);
  void RequestToSend(u8 instruction);
  u8 ReadStatus();
  u8 GetRxStatus();
  void BitModify(u8 address, u8 mask, u8 data);

  // 模式切换
  bool SetConfigMode();
  bool SetNormalMode();
  bool SetSleepMode();

  // CAN ID转换
  struct IdReg {
    u8 sidh;
    u8 sidl;
    u8 eid8;
    u8 eid0;
  };

  static void ConvertCanIdToReg(u32 id, u8 id_type, IdReg &reg);
  static u32 ConvertRegToStandardCanId(u8 sidh, u8 sidl);
  static u32 ConvertRegToExtendedCanId(u8 eid8, u8 eid0, u8 sidh, u8 sidl);

  void ConfigBitTiming();

  SPI_HandleTypeDef *hspi_{nullptr};
  GPIO_TypeDef *cs_gpio_port_{nullptr};
  u16 cs_pin_{0};
  Bitrate bitrate_{Bitrate::k500kbps};
  CanFrame rx_buffer_{};

  static constexpr u8 kSpiTimeout = 10;
  static constexpr u8 kStandardCanMsgId = 1;
  static constexpr u8 kExtendedCanMsgId = 2;
};

}  // namespace rm::hal::stm32

#endif  // HAL_SPI_MODULE_ENABLED && HAL_GPIO_MODULE_ENABLED

#endif  // LIBRM_HAL_STM32_MCP2515_HPP
