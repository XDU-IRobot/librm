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
 * @file  librm.hpp
 * @brief librm
 */

#ifndef LIBRM_HPP
#define LIBRM_HPP

/******** CORE ********/
#include "librm/core/cmsis_rtos.hpp"
#include "librm/core/thread_pool.hpp"
#include "librm/core/typedefs.hpp"
#include "librm/core/exception.hpp"
#include "librm/core/time.hpp"
/****************/

/******** HAL ********/
#include "librm/hal/can.hpp"
#include "librm/hal/gpio.hpp"
#include "librm/hal/serial.hpp"
#include "librm/hal/spi.hpp"
#include "librm/hal/timer.hpp"
/****************/

/******** DEVICE ********/
#include "librm/device/device.hpp"
#include "librm/device/can_device.hpp"
#include "librm/device/actuator/directdrive_motor.hpp"
#include "librm/device/actuator/dji_motor.hpp"
#include "librm/device/actuator/dm_motor.hpp"
#include "librm/device/actuator/unitree_motor.hpp"
#include "librm/device/actuator/go8010_motor.hpp"
#include "librm/device/actuator/zdt_stepper.hpp"
#include "librm/device/referee/referee.hpp"
#include "librm/device/remote/dr16.hpp"
#include "librm/device/remote/vt03.hpp"
#include "librm/device/sensor/bmi088.hpp"
#include "librm/device/sensor/ist8310.hpp"
#include "librm/device/supercap/supercap.hpp"
/****************/

/******** MODULES ********/
#include "librm/modules/ahrs/mahony.hpp"
#include "librm/modules/ahrs/ekf.hpp"
#include "librm/modules/chassis_fkik.hpp"
#include "librm/modules/crc.hpp"
#include "librm/modules/float16.hpp"
#include "librm/modules/pid.hpp"
#include "librm/modules/threshold_trigger.hpp"
#include "librm/modules/utils.hpp"
#include "librm/modules/vofa_plotter.hpp"
/****************/

#endif  // LIBRM_HPP
