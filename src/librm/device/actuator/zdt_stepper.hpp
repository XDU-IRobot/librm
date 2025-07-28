
#ifndef ZDT_STEPPER_HPP
#define ZDT_STEPPER_HPP

#include "librm/core/typedefs.h"
#include "librm/hal/serial_interface.h"

using namespace rm;

namespace rm::device {

class ZdtStepper {
  public:
  ZdtStepper(hal::SerialInterface &serial, u8 motor_id = 0);
  ~ZdtStepper() = default;

  void MotorVelCtrl(u8 addr, u8 dir, u16 vel, u8 acc, bool snF);
  void MotorPosCtrl(u8 addr, u8 dir, u16 vel, u8 acc, u32 clk, bool raF, bool snF);

  void MotorSyncCtrl(u8 addr);

  void ReadPos(u8 addr);
  void ReadVel(u8 addr);

  void RxCallback(const std::vector<u8> &data, u16 rx_len);

  [[nodiscard]] f32 vel() { return this->motor_vel_; }
  [[nodiscard]] f32 pos() { return this->motor_pos_; }

  private:
    hal::SerialInterface *serial_;
    u32 pos_{0};
    u16 vel_{0};

    f32 motor_pos_{0.0f};
    f32 motor_vel_{0.0f};

    u8 motor_id_{0};
};

}

#endif