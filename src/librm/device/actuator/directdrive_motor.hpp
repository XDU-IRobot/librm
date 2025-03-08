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
 * @file  librm/device/actuator/directdrive_motor.hpp
 * @brief 本末电机类库
 * @todo  这个驱动是用p1010b_111写的，其他型号的本末电机没有测试过
 * @todo  rs485模式
 * @todo  还有一些杂七杂八的什么主动查询、设置反馈方式都没写，暂时用不上
 */

#ifndef LIBRM_DEVICE_ACTUATOR_DIRECTDRIVE_MOTOR_HPP
#define LIBRM_DEVICE_ACTUATOR_DIRECTDRIVE_MOTOR_HPP

#include <type_traits>
#include <unordered_map>
#include <cstring>
#include <chrono>

#include "librm/core/typedefs.h"
#include "librm/device/can_device.hpp"
#include "librm/hal/can_interface.h"
#include "librm/modules/algorithm/utils.hpp"

namespace rm::device {

class DirectDriveMotor : public CanDevice {
 public:
  DirectDriveMotor(hal::CanInterface &can, usize id)
      : CanDevice(can, 0x50 + id, 0x60 + id, 0x70 + id, 0x80 + id, 0x90 + id, 0xa0 + id, 0xb0 + id), id_(id) {
    if (tx_buffer_table_.find(&can) == tx_buffer_table_.end()) {
      tx_buffer_table_.try_emplace(&can, TxBufferTable{});
    }
  }

  DirectDriveMotor(DirectDriveMotor &&) noexcept = default;

 private:
  template <typename ParamType, usize OpCode>
  struct Parameter {
    using param_type = ParamType;

    explicit Parameter(ParamType value) : value(value) {}
    constexpr static usize kOpCode = OpCode;
    ParamType value;
  };

  template <typename T>
  struct is_parameter : std::false_type {};

  template <typename ParamType, usize OpCode>
  struct is_parameter<Parameter<ParamType, OpCode>> : std::true_type {};

  template <typename T>
  static constexpr bool is_parameter_v = is_parameter<T>::value;

 public:
  /**
   * @brief 电机的所有指令参数
   * @note  每个参数的详细说明参见P1010B_111电机规格书的附录2
   * @note  注意：某些参数有取值范围，最好参考规格书设置参数，不要盲目设置
   */
  struct Parameters {
    /* op0 reserved */
    using Version = Parameter<const u32, 1>;                    ///< 版本号
    using SerialNumber = Parameter<const u32, 2>;               ///< 序列号
    using InitPhase = Parameter<const u32, 3>;                  ///< 初始相位
    using EncoderResolution = Parameter<const u32, 4>;          ///< 编码器码数
    using PwmFrequency = Parameter<const u32, 5>;               ///< PWM频率
    using CalibratedMaxPhaseCurrent = Parameter<const f32, 6>;  ///< 校准最大相电流
    using MasterOvercurrentThreshold =
        Parameter<f32, 7>;  ///< 母线过流点：只能在电机失能时设置。实际发送数据=想要设置的值*100。
    using AbsoluteZero = Parameter<const u32, 8>;  ///< 绝对值零位：绝对值编码器相对于用户设置的绝对值零位的偏移量值。
    /* op9 op10 reserved */
    using MaskFault = Parameter<u32, 11>;  ///< 故障屏蔽：Bit31-Bit0 每个位可屏蔽不同的故障（目前仅bit15 - bit0有效）
    /* op12 reserved */
    using UserZero = Parameter<u32, 13>;
    /* op14-op16 reserved */
    // op17 暂未启用
    // op18 暂未启用
    using EnableTorqueFeedFwd = Parameter<u32, 19>;
    // op20 暂未启用
    /* op21 reserved */
    using EnableBusHeartbeat = Parameter<u32, 22>;
    /* op23 reserved */
    using EnablePosisionPlanning = Parameter<u32, 24>;
    /* op25-op27 reserved */
    using Mode = Parameter<u32, 28>;
    /* op29-op41 reserved */
    using Id = Parameter<u32, 42>;
    using CanBitrate = Parameter<u32, 43>;
    // op44 暂未启用
    // op45 暂未启用
    /* op46 reserved */
    using HeartbeatTimeoutMs = Parameter<u32, 47>;
    /* op48-op54 reserved*/
    using AdcBaselineErrorTolerance = Parameter<u32, 55>;
    /* op56-60 reserved */
    using RateCurrent = Parameter<f32, 61>;
    using PolePairs = Parameter<const u32, 62>;
    /* op63 reserved */
    // op64 暂未启用
    /* op65-op73 reserved */
    using PositionKp = Parameter<f32, 74>;
    using PositionKi = Parameter<f32, 75>;
    using PositionKd = Parameter<f32, 76>;
    using PositionPlanningMaxSpeed = Parameter<u32, 77>;
    using PositionPlanningAccel = Parameter<u32, 78>;
    using PositionPlanningDeaccel = Parameter<u32, 79>;
    /* op80-op81 reserved */
    using SpeedController = Parameter<u32, 82>;
    using VoltageOpenLoopAccel = Parameter<u32, 83>;
    using SpeedLoopAccel = Parameter<u32, 84>;
    using CurrentLoopAccel = Parameter<u32, 85>;
    /* op86-op87 reserved */
    using FirstSpeedLoopKp = Parameter<f32, 89>;
    using FirstSpeedLoopKi = Parameter<f32, 90>;
    using FirstSpeedLoopKd = Parameter<f32, 91>;
    using SecondSpeedLoopKp = Parameter<f32, 92>;
    using SecondSpeedLoopKi = Parameter<f32, 93>;
    using SecondSpeedLoopKd = Parameter<f32, 94>;
    using SpeedLoopParamSwitchPoint = Parameter<u32, 95>;
    using LadrB0 = Parameter<f32, 96>;
    using LadrOmega = Parameter<f32, 97>;
    // op98 暂未启用
    // op99 暂未启用
    // op100 暂未启用
    /* op101-op103 reserved */
    using CurrentLoopKp = Parameter<const f32, 104>;
    using CurrentLoopKi = Parameter<const f32, 105>;
    using CurrentLoopFeedFwdCoeff = Parameter<f32, 106>;
    /* op107 reserved */
    // op108 暂未启用
    using MaxIq = Parameter<f32, 109>;
    /* op110-op112 reserved */
    using ThermalLimit = Parameter<u32, 113>;
    /* op114-op115 reserved */
    using MaxVoltage = Parameter<u32, 116>;
    using MinVoltage = Parameter<u32, 117>;
    using PhaseResistance = Parameter<f32, 118>;
    using PhaseInductance = Parameter<f32, 119>;
    using CurrentLoopBandwidth = Parameter<u32, 120>;
    // op121-op248 不允许读写
    /* op249 reserved */
    using MaxPhaseCurrent = Parameter<f32, 250>;
    using MaxPhaseCurrentDelta = Parameter<f32, 251>;
    /* op252-op254 reserved */
    using EndIdentifier = Parameter<const u32, 255>;
  };

  /**
   * @brief 电机报警码含义
   * @note  在不重新上电的情况下，警报可自动恢复，故障不可自动恢复；
   * @note  可通过设置参数表中第11个参数（故障屏蔽）进行故障屏蔽。
   */
  enum ErrorCode {
    kFlashWriteFail = 101,
    kCalibrationFail,
    kTempSensorOffline,
    kWeakMagneticField,
    kEncoderOverspeed,
    kHighTemperature,
    kMosHighTemperature,
  };
  struct TxBufferTable {
    u8 command_data[16]{};
    bool command_data_updated_flag_1234{};
    bool command_data_updated_flag_5678{};
    u8 set_feedback_data[8]{};
    u8 query_data[8]{};
    u8 mode_control_data[8]{};
    u8 save_parameter_data[8]{};
  };

 private:
  enum TxCommandId {
    kDrive1234 = 0x32,
    kDrive5678,
    kSetFeedback,
    kQuery,
    kSetParameter,
    kReadParameter,
    kModeControl,
    kSaveParameter,
    kSoftwareReset,
  };
  enum class FeedbackDataType {
    kShaftSpeed = 1u,  ///< 中心轴速度*10
    kBusCurrent,       ///< 母线电流*100
    kIq,               ///< 电流交轴分量
    kEncoder,          ///< 转子位置 (0-32768)
    kError,            ///< 故障信息
    kWarning,          ///< 警告信息
    kMosTemperature,   ///< MOS温度
    kCoilTemperature,  ///< 绕组温度
    kMode,             ///< 当前模式
    kSystemVoltage,    ///< 系统电压*10
    kCycleCounter,     ///< 转子累计旋转圈数*100
    kSystemStatus,     ///< 系统状态
    kAbsoluteEncoder,  ///< 绝对位置
    kMaxPhaseCurrent,  ///< 相电流最大值*100
  };
  ErrorCode error_code_{};
  usize id_{};
  static std::unordered_map<hal::CanInterface *, TxBufferTable> tx_buffer_table_;
  enum Mode {
    kUnknown = -1,     ///< 程序不知道现在电机处于什么模式
    kVoltageOpenloop,  ///< 电压开环
    kCurrent,  ///< 电流闭环，可控制扭矩：扭矩Nm = 给定电流值（A）/1.414 * 转矩常数（1.2Nm/A）
    kSpeed,    ///< 速度闭环
    kPosition,  ///< 位置闭环
  } current_mode_{Mode::kUnknown};
  struct {
    f32 rpm{};             ///< 中心轴转速
    f32 iq{};              ///< IQ实际反馈电流
    u16 encoder{};         ///< 编码器绝对位置
    f32 master_voltage{};  ///< 系统母线电压
  } feedback_{};

 public:
  /**
   * @brief   获取电机参数
   * @todo    暂时没有实现，框架缺少一个统一的异步处理interface来等待参数返回
   * @todo    要么给cmsis rtos实现posix，要么在std::future和cmsis rtos上面再写一个future
   * @tparam  ParamT      要获取的参数
   * @return  参数对象
   */
  template <typename ParamT, std::enable_if_t<is_parameter_v<ParamT>, int> = 0>
  [[deprecated("function unfinished, do not use!")]] ParamT GetParameter() {
    if constexpr (std::is_same_v<std::remove_cv_t<typename ParamT::param_type>, f32>) {
    } else if constexpr (std::is_same_v<std::remove_cv_t<typename ParamT::param_type>, u32>) {
    }
    return ParamT(1);
  }

  template <typename ParamType, usize OpCode>
  void SetParameter(const Parameter<ParamType, OpCode> &param) {
    static_assert(!std::is_const_v<ParamType>, "this parameter is read-only.");
    u8 data[8]{};
    data[0] = id_;
    data[1] = OpCode;
    std::memcpy(&data[2], &param.value, sizeof(ParamType));
    can_->Write(TxCommandId::kSetParameter, data, 8);
  }

  void Set(f32 control_value);
  void Set(f32 control_value, Mode mode);

  void Enable(bool enable);
  static void ResetAllOn(hal::CanInterface &can);
  static void ResetAll();
  static void SendCommand();

  [[nodiscard]] inline ErrorCode error() const { return error_code_; }
  [[nodiscard]] inline f32 rpm() const { return feedback_.rpm; }
  [[nodiscard]] inline f32 iq() const { return feedback_.iq; }
  [[nodiscard]] inline u16 encoder() const { return feedback_.encoder; }
  [[nodiscard]] inline f32 master_voltage() const { return feedback_.master_voltage; }

 private:
  void RxCallback(const hal::CanMsg *msg) override;
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_DIRECTDRIVE_MOTOR_HPP