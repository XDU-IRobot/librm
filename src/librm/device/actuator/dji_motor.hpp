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
 * @file  librm/device/actuator/dji_motor.hpp
 * @brief 大疆电机类库
 *
 * @note DJI GM6020电机
 * @note 反馈报文stdID:     0x205 + ID
 * @note 控制报文stdID:     0x1ff(1234), 0x2ff(567)
 * @note Voltage range:   -30000 ~ 30000, big-endian
 * @note STD, DATA, DLC=8
 * @note https://www.robomaster.com/zh-CN/products/components/general/GM6020
 *
 * @note DJI M3508电机/C620电调
 * @note 反馈报文stdID:     0x200 + ID
 * @note 控制报文stdID:     0x200(1234), 0x1ff(5678)
 * @note 电流范围:         -16384 ~ 16384(-20A ~ 20A), big-endian
 * @note STD, DATA, DLC=8
 * @note https://www.robomaster.com/zh-CN/products/components/general/M3508
 *
 * @note DJI M2006电机/C610电调
 * @note 反馈报文stdID:     0x200 + ID
 * @note 控制报文stdID:     0x200(1234), 0x1ff(5678)
 * @note 电流范围:          -10000 ~ 10000(-10A ~ 10A), big-endian
 * @note STD, DATA, DLC=8
 * @note https://www.robomaster.com/zh-CN/products/components/general/M2006
 */

#ifndef LIBRM_DEVICE_ACTUATOR_DJI_MOTOR_HPP
#define LIBRM_DEVICE_ACTUATOR_DJI_MOTOR_HPP

#include <etl/unordered_map.h>

#include "librm/device/can_device.hpp"
#include "librm/modules/utils.hpp"

namespace rm::device {

/**
 * @brief DjiMotor的非模板基类，用于存储所有DjiMotor实例共享的静态成员，以及提供发送控制消息的静态函数
 */
class DjiMotorBase {
 public:
  /**
   * @brief 发送控制消息给所有大疆电机
   */
  static void SendCommand() {
    for (auto &[can, buffers] : tx_buf_) {
      if (buffers.dirty_200) {
        can->Write(0x200, buffers.data_200, 8);
        buffers.dirty_200 = false;
      }
      if (buffers.dirty_1ff) {
        can->Write(0x1ff, buffers.data_1ff, 8);
        buffers.dirty_1ff = false;
      }
      if (buffers.dirty_2ff) {
        can->Write(0x2ff, buffers.data_2ff, 8);
        buffers.dirty_2ff = false;
      }
      if (buffers.dirty_1fe) {
        can->Write(0x1fe, buffers.data_1fe, 8);
        buffers.dirty_2ff = false;
      }
      if (buffers.dirty_2fe) {
        can->Write(0x2fe, buffers.data_2fe, 8);
        buffers.dirty_2ff = false;
      }
    }
  }

  /**
   * @brief 发送控制消息给某条CAN总线上的所有大疆电机
   * @param can 目标CAN总线
   */
  static void SendCommand(hal::CanInterface &can) {
    auto target_can = tx_buf_.find(&can);
    if (target_can == tx_buf_.end()) {
      return;
    }
    auto &buffers = target_can->second;

    if (buffers.dirty_200) {
      can.Write(0x200, buffers.data_200, 8);
      buffers.dirty_200 = false;
    }
    if (buffers.dirty_1ff) {
      can.Write(0x1ff, buffers.data_1ff, 8);
      buffers.dirty_1ff = false;
    }
    if (buffers.dirty_2ff) {
      can.Write(0x2ff, buffers.data_2ff, 8);
      buffers.dirty_2ff = false;
    }
    if (buffers.dirty_1fe) {
      can.Write(0x1fe, buffers.data_1fe, 8);
      buffers.dirty_2ff = false;
    }
    if (buffers.dirty_2fe) {
      can.Write(0x2fe, buffers.data_2fe, 8);
      buffers.dirty_2ff = false;
    }
  }

 protected:
  /**
   * @brief
   * 对DJI电机来说，一条CAN总线上可以用帧id区分三条Tx通道：0x200、0x1ff、0x2ff，这个结构体就用来表示这三条通道的缓冲区。
   */
  struct TxBuffers {
    u8 data_200[8];  ///< 0x200通道的发送缓冲区
    u8 data_1ff[8];  ///< 同上
    u8 data_2ff[8];  ///< 同上
    u8 data_1fe[8];  ///< 同上
    u8 data_2fe[8];  ///< 同上
    bool dirty_200{false};  ///< 0x200通道的发送缓冲区是否已修改，是true的话就说明SendCommand函数需要发送这条消息
    bool dirty_1ff{false};  ///< 同上
    bool dirty_2ff{false};  ///< 同上
    bool dirty_1fe{false};  ///< 同上
    bool dirty_2fe{false};  ///< 同上
  };

  static etl::unordered_map<hal::CanInterface *, TxBuffers,
                            5>  // 最多5条CAN总线，一般情况下肯定够用，不够可以调大
      tx_buf_;
};

enum class DjiMotorType {
  kGM6020,
  kM3508,
  kM2006,
  kGM6020Current,  ///< 6020更新1.0.11.2版本固件之后支持转矩电流控制
};

/**
 * @brief  大疆电机(GM6020, M3508, M2006)
 * @tparam motor_type 电机类型(GM6020, M3508, M2006)
 */
template <DjiMotorType motor_type>
class DjiMotor : public CanDevice, protected DjiMotorBase {
  /**
   * @brief 获取对应型号电机的反馈报文ID基址
   */
  constexpr static u16 GetRxIdBase() {
    switch (motor_type) {
      case DjiMotorType::kGM6020:
      case DjiMotorType::kGM6020Current: {
        return 0x204;
      }
      case DjiMotorType::kM3508:
      case DjiMotorType::kM2006: {
        return 0x200;
      }
      default: {
        return 0;
      }
    }
  }

  /**
   * @brief 获取对应型号电机的电流命令上限
   */
  constexpr static i16 GetCurrentBound() {
    switch (motor_type) {
      case DjiMotorType::kGM6020:
        return 30000;
      case DjiMotorType::kM3508:
        return 16384;
      case DjiMotorType::kM2006:
        return 10000;
      case DjiMotorType::kGM6020Current:
        return 16384;
      default:
        return 0;
    }
  }

 public:
  constexpr static i16 kMaxEncoderValue = 8191;

  DjiMotor() = delete;
  ~DjiMotor() override = default;

  /**
   * @param can      指向CAN总线对象的引用
   * @param id       电机ID
   * @param reversed 是否反转
   */
  DjiMotor(hal::CanInterface &can, u16 id, bool reversed = false)
      : CanDevice(can, GetRxIdBase() + id), id_(id), reversed_(reversed) {
    // 检查id是否合法
    if (id_ < 1 || id_ > 8) {
      rm::Throw(std::runtime_error("DjiMotor: invalid motor id"));
    }
    if constexpr (motor_type == DjiMotorType::kGM6020) {
      if (id_ > 7) {  // 6020没有8号
        rm::Throw(std::runtime_error("DjiMotor: invalid motor id for GM6020"));
      }
    }
    // 如果这个电机所属的CAN总线还没有对应的发送缓冲区，就创建一个
    if (tx_buf_.find(&can) == tx_buf_.end()) {
      tx_buf_.insert({&can, TxBuffers{}});
    }
  }

  /**
   * @brief  设置电机的输出电流
   * @note   这个函数不会发送控制消息，在设置电流值后需要调用SendCommand函数向所有电机发出控制消息
   * @param  current    设定电流值
   */
  void SetCurrent(i16 current) {
    // 限幅到电机能接受的最大电流
    current = modules::Clamp(current, -GetCurrentBound(), GetCurrentBound());
    // 处理反转
    if (this->reversed_) {
      current = -current;
    }
    // 找到这个电机所属的CAN总线的发送缓冲区
    auto buffers = tx_buf_.find(this->can_);
    if (buffers == tx_buf_.end()) {
      // 这个电机没有对应的CAN总线缓冲区，理论上不会发生，但是为安全起见还是加上这个判断
      return;
    }
    auto &[_, buf] = *buffers;
    // 根据电机型号和ID(this->id_)找到对应的发送缓冲区，写入电流值，标记缓冲区已修改
    if constexpr (motor_type == DjiMotorType::kGM6020) {
      if (1 <= this->id_ && this->id_ <= 4) {
        buf.data_1ff[(this->id_ - 1) * 2] = (current >> 8) & 0xff;
        buf.data_1ff[(this->id_ - 1) * 2 + 1] = current & 0xff;
        buf.dirty_1ff = true;
      } else if (5 <= this->id_ && this->id_ <= 8) {
        buf.data_2ff[(this->id_ - 5) * 2] = (current >> 8) & 0xff;
        buf.data_2ff[(this->id_ - 5) * 2 + 1] = current & 0xff;
        buf.dirty_2ff = true;
      }
    } else if constexpr (motor_type == DjiMotorType::kM3508 ||  //
                         motor_type == DjiMotorType::kM2006) {
      if (1 <= this->id_ && this->id_ <= 4) {
        buf.data_200[(this->id_ - 1) * 2] = (current >> 8) & 0xff;
        buf.data_200[(this->id_ - 1) * 2 + 1] = current & 0xff;
        buf.dirty_200 = true;
      } else if (5 <= this->id_ && this->id_ <= 8) {
        buf.data_1ff[(this->id_ - 5) * 2] = (current >> 8) & 0xff;
        buf.data_1ff[(this->id_ - 5) * 2 + 1] = current & 0xff;
        buf.dirty_1ff = true;
      }
    } else if constexpr (motor_type == DjiMotorType::kGM6020Current) {
      if (1 <= this->id_ && this->id_ <= 4) {
        buf.data_1fe[(this->id_ - 1) * 2] = (current >> 8) & 0xff;
        buf.data_1fe[(this->id_ - 1) * 2 + 1] = current & 0xff;
        buf.dirty_1fe = true;
      } else if (5 <= this->id_ && this->id_ <= 8) {
        buf.data_2fe[(this->id_ - 5) * 2] = (current >> 8) & 0xff;
        buf.data_2fe[(this->id_ - 5) * 2 + 1] = current & 0xff;
        buf.dirty_2fe = true;
      }
    }
  }

  /** 取值函数 **/
  [[nodiscard]] u16 encoder() const { return this->encoder_; }
  [[nodiscard]] i16 rpm() const { return this->rpm_; }
  [[nodiscard]] i16 current() const { return this->current_; }
  [[nodiscard]] u8 temperature() const { return this->temperature_; }

  [[nodiscard]] f32 pos_degree() const { return (f32)this->encoder() / kMaxEncoderValue * 360.f; }
  [[nodiscard]] f32 pos_rad() const { return (f32)this->encoder() / kMaxEncoderValue * M_PI * 2; }
  /*************/

 private:
  void RxCallback(const hal::CanFrame *msg) override {
    ReportStatus(kOk);
    this->encoder_ = (msg->data[0] << 8) | msg->data[1];
    this->rpm_ = (msg->data[2] << 8) | msg->data[3];
    this->current_ = (msg->data[4] << 8) | msg->data[5];
    this->temperature_ = msg->data[6];
    if (reversed_) {
      this->encoder = this->kMaxEncoderValue - this->encoder;
      this->rpm_ = -this->rpm_;
      this->current = -this->current_;
    }
  }

  u16 id_{};         ///< 电机ID
  bool reversed_{};  ///< 是否反转
  /**   FEEDBACK DATA   **/
  u16 encoder_{};     ///< 电机编码器值
  i16 rpm_{};         ///< 电机转速
  i16 current_{};     ///< 电机实际电流
  u8 temperature_{};  ///< 电机温度
  /***********************/
};

using GM6020 = DjiMotor<DjiMotorType::kGM6020>;
using M3508 = DjiMotor<DjiMotorType::kM3508>;
using M2006 = DjiMotor<DjiMotorType::kM2006>;
using GM6020Current = DjiMotor<DjiMotorType::kGM6020Current>;

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_DJI_MOTOR_HPP
