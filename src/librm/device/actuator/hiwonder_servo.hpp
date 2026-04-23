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
 * @file  librm/device/actuator/hiwonder_servo.hpp
 * @brief 幻尔串口总线舵机驱动
 * @note  半双工总线通讯，波特率 115200bps，协议格式：
 *        帧头(0x55 0x55) | ID | Length | Cmd | Params... | Checksum
 *        使用 74HC126 三态缓冲器，由 TX_EN / RX_EN 两个 GPIO 控制收发方向
 */

#ifndef LIBRM_DEVICE_ACTUATOR_HTS35H_SERVO_HPP
#define LIBRM_DEVICE_ACTUATOR_HTS35H_SERVO_HPP

#include <etl/span.h>

#include "librm/core/typedefs.hpp"
#include "librm/device/device.hpp"
#include "librm/hal/serial_interface.hpp"
#include "librm/hal/gpio_interface.hpp"

namespace rm::device {

/**
 * @brief 幻尔串口总线舵机驱动类
 * @note  已在LX-20S、HTS-35H两款舵机上测试过可用，其他型号理论上也适用
 *
 * 硬件连接：MCU UART TX/RX 经 74HC126 三态缓冲器连接到舵机总线
 *   - TX_EN GPIO：高电平使能发送通道
 *   - RX_EN GPIO：高电平使能接收通道
 *
 * @note  收发互斥：发送时 TX_EN=1, RX_EN=0；发送完成后切回 TX_EN=0, RX_EN=1
 */
class HiWonderServo : public Device {
 public:
  /// @brief 舵机指令值（协议表2）
  enum class Cmd : u8 {
    kMoveTimeWrite = 1,
    kMoveTimeRead = 2,
    kMoveTimeWaitWrite = 7,
    kMoveTimeWaitRead = 8,
    kMoveStart = 11,
    kMoveStop = 12,
    kIdWrite = 13,
    kIdRead = 14,
    kAngleOffsetAdjust = 17,
    kAngleOffsetWrite = 18,
    kAngleOffsetRead = 19,
    kAngleLimitWrite = 20,
    kAngleLimitRead = 21,
    kVinLimitWrite = 22,
    kVinLimitRead = 23,
    kTempMaxLimitWrite = 24,
    kTempMaxLimitRead = 25,
    kTempRead = 26,
    kVinRead = 27,
    kPosRead = 28,
    kOrMotorModeWrite = 29,
    kOrMotorModeRead = 30,
    kLoadOrUnloadWrite = 31,
    kLoadOrUnloadRead = 32,
    kLedCtrlWrite = 33,
    kLedCtrlRead = 34,
    kLedErrorWrite = 35,
    kLedErrorRead = 36,
    kDisRead = 48,
  };

  /// @brief 舵机反馈数据
  struct Feedback {
    i16 position{0};      ///< 当前角度位置值 (signed, 0~1000 => 0~240°, 可为负值)
    u16 voltage{0};       ///< 输入电压 (mV)
    u8 temperature{0};    ///< 内部温度 (℃)
    u8 servo_mode{0};     ///< 0: 位置控制模式, 1: 电机控制模式
    u8 rotation_mode{0};  ///< 0: 固定占空比模式, 1: 固定转速模式
    i16 speed{0};         ///< 转动速度值（仅电机模式有效）
    i32 distance{0};      ///< 转动距离 (4096 脉冲/圈)
    u8 loaded{0};         ///< 0: 卸载(无力矩), 1: 装载(有力矩)
  };

  /**
   * @brief 构造函数
   * @param serial  串口接口引用
   * @param tx_en   TX 使能引脚（74HC126 TX_EN，高有效）
   * @param rx_en   RX 使能引脚（74HC126 RX_EN，高有效）
   * @param servo_id 舵机 ID，范围 0~253，默认 1
   */
  HiWonderServo(hal::SerialInterface &serial, hal::PinInterface &tx_en, hal::PinInterface &rx_en, u8 servo_id = 1);
  ~HiWonderServo() override = default;

  ///////////// WRITE

  /**
   * @brief 位置时间控制（立即执行）
   * @param angle   目标角度，范围 0~1000 (对应 0~240°)
   * @param time_ms 运动时间，范围 0~30000 ms
   */
  void MoveTime(u16 angle, u16 time_ms);

  /**
   * @brief 位置时间控制（等待 MoveStart 后执行）
   * @param angle   目标角度，范围 0~1000
   * @param time_ms 运动时间，范围 0~30000 ms
   */
  void MoveTimeWait(u16 angle, u16 time_ms);

  /**
   * @brief 触发等待中的运动指令
   */
  void MoveStart();

  /**
   * @brief 立即停止转动
   */
  void MoveStop();

  /**
   * @brief 设置舵机 ID（掉电保存）
   * @param new_id 新 ID，范围 0~253
   */
  void SetId(u8 new_id);

  /**
   * @brief 调整角度偏差（不掉电保存）
   * @param offset 偏差值，范围 -125~125 (对应 -30°~30°)
   */
  void AngleOffsetAdjust(i8 offset);

  /**
   * @brief 保存当前偏差值（掉电保存）
   */
  void AngleOffsetSave();

  /**
   * @brief 设置角度限制（掉电保存）
   * @param min_angle 最小角度，范围 0~1000
   * @param max_angle 最大角度，范围 0~1000，且 min < max
   */
  void SetAngleLimit(u16 min_angle, u16 max_angle);

  /**
   * @brief 设置输入电压限制（掉电保存）
   * @param min_mv 最小电压 (mV)，范围 4500~14000
   * @param max_mv 最大电压 (mV)，范围 4500~14000，且 min < max
   */
  void SetVinLimit(u16 min_mv, u16 max_mv);

  /**
   * @brief 设置最高温度限制（掉电保存）
   * @param max_temp 最高温度 (℃)，范围 50~100，默认 85
   */
  void SetTempMaxLimit(u8 max_temp);

  /**
   * @brief 设置为电机控制模式
   * @param rotation_mode 转动模式: 0=固定占空比, 1=固定转速
   * @param speed 转动速度: 占空比模式 -1000~1000, 转速模式 -50~50
   */
  void SetMotorMode(u8 rotation_mode, i16 speed);

  /**
   * @brief 设置为位置(舵机)控制模式
   */
  void SetServoMode();

  /**
   * @brief 装载/卸载电机
   * @param load true=装载(有力矩), false=卸载(无力矩)
   */
  void SetLoad(bool load);

  /**
   * @brief 设置 LED 灯状态（掉电保存）
   * @param on true=常亮, false=常灭
   */
  void SetLed(bool on);

  /**
   * @brief 设置 LED 故障报警值
   * @param error_flags 范围 0~7（见协议表3）
   */
  void SetLedError(u8 error_flags);

  ////////////// READ

  /**
   * @brief 读取当前位置（结果通过 feedback() 获取）
   */
  void ReadPosition();

  /**
   * @brief 读取内部温度
   */
  void ReadTemperature();

  /**
   * @brief 读取输入电压
   */
  void ReadVoltage();

  /**
   * @brief 读取舵机/电机模式参数
   */
  void ReadMotorMode();

  /**
   * @brief 读取装载/卸载状态
   */
  void ReadLoadStatus();

  /**
   * @brief 读取转动距离（4096 脉冲/圈）
   */
  void ReadDistance();

  /**
   * @brief 读取舵机 ID
   */
  void ReadId();

  /**
   * @brief 读取角度偏差
   */
  void ReadAngleOffset();

  /**
   * @brief 读取角度限制
   */
  void ReadAngleLimit();

  /**
   * @brief 读取电压限制
   */
  void ReadVinLimit();

  /**
   * @brief 读取最高温度限制
   */
  void ReadTempMaxLimit();

  /**
   * @brief 读取 MoveTime 写入的角度和时间
   */
  void ReadMoveTime();

  /**
   * @brief 读取 MoveTimeWait 写入的角度和时间
   */
  void ReadMoveTimeWait();

  /**
   * @brief 读取 LED 灯状态
   */
  void ReadLedCtrl();

  /**
   * @brief 读取 LED 故障报警值
   */
  void ReadLedError();

  /**
   * @brief 获取反馈数据
   * @return 反馈数据结构体（只读引用）
   */
  [[nodiscard]] const Feedback &feedback() const { return feedback_; }

  /**
   * @brief 获取当前角度（度）
   * @return 角度值，0~240°
   */
  [[nodiscard]] f32 angle_deg() const { return static_cast<f32>(feedback_.position) * 0.24f; }

  /**
   * @brief 获取当前输入电压（伏）
   */
  [[nodiscard]] f32 voltage_v() const { return static_cast<f32>(feedback_.voltage) * 0.001f; }

 private:
  /**
   * @brief 串口接收回调函数，解析舵机返回的数据包
   */
  void RxCallback(etl::span<const u8> data);

  /**
   * @brief 切换到发送模式（TX_EN=1, RX_EN=0）
   */
  void SetTxMode() const;

  /**
   * @brief 切换到接收模式（TX_EN=0, RX_EN=1）
   */
  void SetRxMode() const;

  /**
   * @brief 构建指令包并发送
   * @param cmd    指令
   * @param params 参数数组指针（可为 nullptr）
   * @param param_len 参数长度
   */
  void SendPacket(Cmd cmd, const u8 *params, u8 param_len);

  /**
   * @brief 发送无参数的读指令
   * @param cmd 读指令
   */
  void SendReadCmd(Cmd cmd);

  /**
   * @brief 计算校验和
   * @param id     舵机 ID
   * @param length 数据长度字段
   * @param cmd    指令值
   * @param params 参数指针
   * @param param_len 参数长度
   * @return 校验和
   */
  static u8 CalcChecksum(u8 id, u8 length, u8 cmd, const u8 *params, u8 param_len);

  hal::SerialInterface *serial_;
  hal::PinInterface *tx_en_;
  hal::PinInterface *rx_en_;
  const u8 servo_id_;
  Feedback feedback_{};
  u8 tx_buffer_[16]{};  ///< 发送缓冲区（最大包长: 2+1+1+1+4+1 = 10 字节）
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_HTS35H_SERVO_HPP
