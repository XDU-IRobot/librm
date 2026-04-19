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
 * @file  librm/hal/stm32/mcp2515.cc
 * @brief MCP2515 CAN控制器封装
 */

#include "librm/hal/stm32/hal.hpp"
#if defined(HAL_SPI_MODULE_ENABLED) && defined(HAL_GPIO_MODULE_ENABLED)

#include "mcp2515.hpp"

#include "librm/device/can_device.hpp"

namespace rm::hal::stm32 {

// ============================================================================
// 构造 / 析构 / 移动
// ============================================================================

Mcp2515::Mcp2515(SPI_HandleTypeDef &hspi, GPIO_TypeDef *cs_gpio_port, u16 cs_pin, Bitrate bitrate)
    : hspi_(&hspi), cs_gpio_port_(cs_gpio_port), cs_pin_(cs_pin), bitrate_(bitrate) {
  CsHigh();
}

Mcp2515::~Mcp2515() { Stop(); }

Mcp2515::Mcp2515(Mcp2515 &&other) noexcept
    : hspi_(other.hspi_), cs_gpio_port_(other.cs_gpio_port_), cs_pin_(other.cs_pin_), bitrate_(other.bitrate_) {
  other.hspi_ = nullptr;
}

Mcp2515 &Mcp2515::operator=(Mcp2515 &&other) noexcept {
  if (this != &other) {
    hspi_ = other.hspi_;
    cs_gpio_port_ = other.cs_gpio_port_;
    cs_pin_ = other.cs_pin_;
    bitrate_ = other.bitrate_;
    other.hspi_ = nullptr;
  }
  return *this;
}

// ============================================================================
// SPI底层
// ============================================================================

void Mcp2515::CsLow() { HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_RESET); }

void Mcp2515::CsHigh() { HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_SET); }

void Mcp2515::SpiTx(u8 data) { HAL_SPI_Transmit(hspi_, &data, 1, kSpiTimeout); }

void Mcp2515::SpiTxBuffer(const u8 *buffer, u8 length) {
  HAL_SPI_Transmit(hspi_, const_cast<u8 *>(buffer), length, kSpiTimeout);
}

u8 Mcp2515::SpiRx() {
  u8 ret_val = 0;
  HAL_SPI_Receive(hspi_, &ret_val, 1, kSpiTimeout);
  return ret_val;
}

void Mcp2515::SpiRxBuffer(u8 *buffer, u8 length) { HAL_SPI_Receive(hspi_, buffer, length, kSpiTimeout); }

// ============================================================================
// 寄存器操作
// ============================================================================

void Mcp2515::ChipReset() {
  CsLow();
  SpiTx(kCmdReset);
  CsHigh();
  HAL_Delay(10);  // 等待振荡器稳定
}

u8 Mcp2515::ReadRegister(u8 address) {
  u8 ret_val;
  CsLow();
  SpiTx(kCmdRead);
  SpiTx(address);
  ret_val = SpiRx();
  CsHigh();
  return ret_val;
}

void Mcp2515::ReadRxSequence(u8 instruction, u8 *data, u8 length) {
  CsLow();
  SpiTx(instruction);
  SpiRxBuffer(data, length);
  CsHigh();
}

void Mcp2515::WriteRegister(u8 address, u8 data) {
  CsLow();
  SpiTx(kCmdWrite);
  SpiTx(address);
  SpiTx(data);
  CsHigh();
}

void Mcp2515::WriteRegisterSequence(u8 start_address, u8 end_address, u8 *data) {
  CsLow();
  SpiTx(kCmdWrite);
  SpiTx(start_address);
  SpiTxBuffer(data, end_address - start_address + 1);
  CsHigh();
}

void Mcp2515::LoadTxSequence(u8 instruction, u8 *id_reg, u8 dlc, const u8 *data) {
  CsLow();
  SpiTx(instruction);
  SpiTxBuffer(id_reg, 4);
  SpiTx(dlc);
  SpiTxBuffer(data, dlc);
  CsHigh();
}

void Mcp2515::RequestToSend(u8 instruction) {
  CsLow();
  SpiTx(instruction);
  CsHigh();
}

u8 Mcp2515::ReadStatus() {
  u8 ret_val;
  CsLow();
  SpiTx(kCmdReadStatus);
  ret_val = SpiRx();
  CsHigh();
  return ret_val;
}

u8 Mcp2515::GetRxStatus() {
  u8 ret_val;
  CsLow();
  SpiTx(kCmdRxStatus);
  ret_val = SpiRx();
  CsHigh();
  return ret_val;
}

void Mcp2515::BitModify(u8 address, u8 mask, u8 data) {
  CsLow();
  SpiTx(kCmdBitModify);
  SpiTx(address);
  SpiTx(mask);
  SpiTx(data);
  CsHigh();
}

// ============================================================================
// 模式切换
// ============================================================================

bool Mcp2515::SetConfigMode() {
  WriteRegister(kRegCanCtrl, 0x80);
  u8 loop = 10;
  do {
    if ((ReadRegister(kRegCanStat) & 0xE0) == 0x80) return true;
    HAL_Delay(1);
    loop--;
  } while (loop > 0);
  return false;
}

bool Mcp2515::SetNormalMode() {
  WriteRegister(kRegCanCtrl, 0x00);
  u8 loop = 10;
  do {
    if ((ReadRegister(kRegCanStat) & 0xE0) == 0x00) return true;
    loop--;
  } while (loop > 0);
  return false;
}

bool Mcp2515::SetSleepMode() {
  WriteRegister(kRegCanCtrl, 0x20);
  u8 loop = 10;
  do {
    if ((ReadRegister(kRegCanStat) & 0xE0) == 0x20) return true;
    loop--;
  } while (loop > 0);
  return false;
}

// ============================================================================
// CAN ID转换
// ============================================================================

void Mcp2515::ConvertCanIdToReg(u32 id, u8 id_type, IdReg &reg) {
  if (id_type == kExtendedCanMsgId) {
    reg.eid0 = 0xFF & id;
    id >>= 8;
    reg.eid8 = 0xFF & id;
    id >>= 8;
    u8 wip_sidl = 0x03 & id;
    id <<= 3;
    wip_sidl = (0xE0 & id) + wip_sidl;
    wip_sidl = wip_sidl + 0x08;  // EXIDE位
    reg.sidl = 0xEB & wip_sidl;
    id >>= 8;
    reg.sidh = 0xFF & id;
  } else {
    reg.eid8 = 0;
    reg.eid0 = 0;
    id <<= 5;
    reg.sidl = 0xFF & id;
    id >>= 8;
    reg.sidh = 0xFF & id;
  }
}

u32 Mcp2515::ConvertRegToStandardCanId(u8 sidh, u8 sidl) { return (static_cast<u32>(sidh) << 3) + (sidl >> 5); }

u32 Mcp2515::ConvertRegToExtendedCanId(u8 eid8, u8 eid0, u8 sidh, u8 sidl) {
  u8 lo2bits = sidl & 0x03;
  u8 hi3bits = sidl >> 5;
  u32 id = (static_cast<u32>(sidh) << 3) + hi3bits;
  id = (id << 2) + lo2bits;
  id = (id << 8) + eid8;
  id = (id << 8) + eid0;
  return id;
}

// ============================================================================
// 波特率配置（Fosc = 8MHz）
// ============================================================================

void Mcp2515::ConfigBitTiming() {
  // CNF1/CNF2/CNF3配置值由8MHz晶振计算得出
  switch (bitrate_) {
    case Bitrate::k1Mbps:
      WriteRegister(kRegCnf1, 0x00);
      WriteRegister(kRegCnf2, 0x80);
      WriteRegister(kRegCnf3, 0x80);
      break;
    case Bitrate::k500kbps:
      WriteRegister(kRegCnf1, 0x00);
      WriteRegister(kRegCnf2, 0xE5);
      WriteRegister(kRegCnf3, 0x83);
      break;
    case Bitrate::k250kbps:
      WriteRegister(kRegCnf1, 0x01);
      WriteRegister(kRegCnf2, 0xE5);
      WriteRegister(kRegCnf3, 0x83);
      break;
    case Bitrate::k125kbps:
      WriteRegister(kRegCnf1, 0x03);
      WriteRegister(kRegCnf2, 0xE5);
      WriteRegister(kRegCnf3, 0x83);
      break;
  }
}

// ============================================================================
// CanInterface接口实现
// ============================================================================

/**
 * @brief 初始化MCP2515并启动CAN通信
 */
void Mcp2515::Begin() {
  u8 loop = 10;
  while (loop > 0) {
    if (HAL_SPI_GetState(hspi_) == HAL_SPI_STATE_READY) break;
    loop--;
  }
  if (loop == 0) {
    Throw(std::runtime_error("MCP2515: SPI not ready"));
  }

  ChipReset();

  if (!SetConfigMode()) {
    Throw(std::runtime_error("MCP2515: Failed to enter config mode"));
  }

  // 掩码全0，接收所有帧
  u8 zero4[4] = {0, 0, 0, 0};
  WriteRegisterSequence(kRegRxM0Sidh, kRegRxM0Eid0, zero4);
  WriteRegisterSequence(kRegRxM1Sidh, kRegRxM1Eid0, zero4);

  u8 filter_std[4] = {0x00, 0x00, 0x00, 0x00};
  u8 filter_ext[4] = {0x00, 0x08, 0x00, 0x00};  // EXIDE=1
  WriteRegisterSequence(kRegRxF0Sidh, kRegRxF0Eid0, filter_std);
  WriteRegisterSequence(kRegRxF1Sidh, kRegRxF1Eid0, filter_ext);
  WriteRegisterSequence(kRegRxF2Sidh, kRegRxF2Eid0, filter_std);
  WriteRegisterSequence(kRegRxF3Sidh, kRegRxF3Eid0, filter_std);
  WriteRegisterSequence(kRegRxF4Sidh, kRegRxF4Eid0, filter_std);
  WriteRegisterSequence(kRegRxF5Sidh, kRegRxF5Eid0, filter_ext);

  WriteRegister(kRegRxB0Ctrl, 0x04);  // 启用BUKT（溢出到RXB1），接收所有
  WriteRegister(kRegRxB1Ctrl, 0x01);  // 接收所有

  ConfigBitTiming();

  WriteRegister(kRegCanInte, 0x03);  // 启用RX0IE + RX1IE

  if (!SetNormalMode()) {
    Throw(std::runtime_error("MCP2515: Failed to enter normal mode"));
  }
}

/**
 * @brief 停止MCP2515，进入睡眠模式
 */
void Mcp2515::Stop() {
  if (hspi_ == nullptr) return;
  BitModify(kRegCanIntf, 0x40, 0x00);  // 清除唤醒中断标志
  BitModify(kRegCanInte, 0x40, 0x40);  // 使能唤醒中断
  SetSleepMode();
}

/**
 * @brief 设置接收过滤器（标准帧）
 * @param id    过滤ID
 * @param mask  掩码
 */
void Mcp2515::SetFilter(u16 id, u16 mask) {
  if (hspi_ == nullptr) return;
  if (!SetConfigMode()) return;

  u8 mask_regs[4] = {
      static_cast<u8>((mask >> 3) & 0xFF),
      static_cast<u8>((mask << 5) & 0xE0),
      0x00,
      0x00,
  };
  WriteRegisterSequence(kRegRxM0Sidh, kRegRxM0Eid0, mask_regs);
  WriteRegisterSequence(kRegRxM1Sidh, kRegRxM1Eid0, mask_regs);

  u8 filter_regs[4] = {
      static_cast<u8>((id >> 3) & 0xFF),
      static_cast<u8>((id << 5) & 0xE0),
      0x00,
      0x00,
  };
  WriteRegisterSequence(kRegRxF0Sidh, kRegRxF0Eid0, filter_regs);
  WriteRegisterSequence(kRegRxF1Sidh, kRegRxF1Eid0, filter_regs);
  WriteRegisterSequence(kRegRxF2Sidh, kRegRxF2Eid0, filter_regs);
  WriteRegisterSequence(kRegRxF3Sidh, kRegRxF3Eid0, filter_regs);
  WriteRegisterSequence(kRegRxF4Sidh, kRegRxF4Eid0, filter_regs);
  WriteRegisterSequence(kRegRxF5Sidh, kRegRxF5Eid0, filter_regs);

  SetNormalMode();
}

/**
 * @brief 发送标准CAN帧
 * @param id    标准帧ID
 * @param data  数据指针
 * @param size  数据长度（最大8字节）
 */
void Mcp2515::Write(u16 id, const u8 *data, usize size) {
  if (hspi_ == nullptr) return;
  if (size > 8) {
    Throw(std::runtime_error("MCP2515: Data too long for std CAN frame!"));
  }

  IdReg id_reg{};
  ConvertCanIdToReg(id, kStandardCanMsgId, id_reg);

  u8 status = ReadStatus();

  // status bit2: TXB0REQ, bit4: TXB1REQ, bit6: TXB2REQ
  if (!(status & 0x04)) {
    LoadTxSequence(kCmdLoadTxB0Sidh, &id_reg.sidh, static_cast<u8>(size), data);
    RequestToSend(kCmdRtsTx0);
  } else if (!(status & 0x10)) {
    LoadTxSequence(kCmdLoadTxB1Sidh, &id_reg.sidh, static_cast<u8>(size), data);
    RequestToSend(kCmdRtsTx1);
  } else if (!(status & 0x40)) {
    LoadTxSequence(kCmdLoadTxB2Sidh, &id_reg.sidh, static_cast<u8>(size), data);
    RequestToSend(kCmdRtsTx2);
  }
  // 三个TX Buffer均忙则丢弃本帧
}

// ============================================================================
// EXTI中断处理
// ============================================================================

void Mcp2515::ExtiIrqHandler() { HandleRxInterrupt(); }

void Mcp2515::HandleRxInterrupt() {
  // RX状态寄存器 bit[7:6]: 00=无消息, 01=RXB0, 10=RXB1, 11=两者均有
  u8 rx_status = GetRxStatus();
  u8 rx_buffer = (rx_status >> 6) & 0x03;

  while (rx_buffer != 0) {
    u8 rx_reg[13] = {};  // SIDH SIDL EID8 EID0 DLC D0..D7

    if ((rx_buffer == kMsgInRxB0) || (rx_buffer == kMsgInBothBuffers)) {
      ReadRxSequence(kCmdReadRxB0Sidh, rx_reg, sizeof(rx_reg));
    } else if (rx_buffer == kMsgInRxB1) {
      ReadRxSequence(kCmdReadRxB1Sidh, rx_reg, sizeof(rx_reg));
    }

    // bit[4:3]: 消息类型，>=2为扩展帧
    u8 msg_type = (rx_status >> 3) & 0x03;
    u32 can_id = (msg_type >= 2) ? ConvertRegToExtendedCanId(rx_reg[2], rx_reg[3], rx_reg[0], rx_reg[1])
                                 : ConvertRegToStandardCanId(rx_reg[0], rx_reg[1]);

    u8 dlc = rx_reg[4] & 0x0F;
    if (dlc > 8) dlc = 8;

    auto &device_list = GetDeviceListByRxStdid(static_cast<u16>(can_id));
    if (!device_list.empty()) {
      rx_buffer_.rx_std_id = static_cast<u16>(can_id);
      rx_buffer_.dlc = dlc;
      rx_buffer_.is_fd_frame = false;
      for (u8 i = 0; i < dlc; ++i) {
        rx_buffer_.data[i] = rx_reg[5 + i];
      }
      for (auto &device : device_list) {
        device->RxCallback(&rx_buffer_);
      }
    }

    if ((rx_buffer == kMsgInRxB0) || (rx_buffer == kMsgInBothBuffers)) {
      BitModify(kRegCanIntf, 0x01, 0x00);  // 清RX0IF
    }
    if ((rx_buffer == kMsgInRxB1) || (rx_buffer == kMsgInBothBuffers)) {
      BitModify(kRegCanIntf, 0x02, 0x00);  // 清RX1IF
    }

    rx_status = GetRxStatus();
    rx_buffer = (rx_status >> 6) & 0x03;
  }
}

}  // namespace rm::hal::stm32

#endif
