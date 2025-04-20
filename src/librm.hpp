/*
  Copyright (c) 2024 XDU-IRobot

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
#include "librm/core/cmsis_rtos.h"
#include "librm/core/thread_pool.hpp"
#include "librm/core/typedefs.h"
#include "librm/core/exception.h"
#include "librm/core/time.hpp"
/****************/

/******** HAL ********/
#include "librm/hal/can.h"
#include "librm/hal/gpio.h"
#include "librm/hal/serial.h"
#include "librm/hal/spi.h"
/****************/

/******** DEVICE ********/
#include "librm/device/device.h"
#include "librm/device/can_device.hpp"
#include "librm/device/actuator/directdrive_motor.hpp"
#include "librm/device/actuator/dji_motor.hpp"
#include "librm/device/actuator/dm_motor.hpp"
#include "librm/device/actuator/unitree_motor.hpp"
#include "librm/device/actuator/go8010_motor.hpp"
#include "librm/device/referee/referee.hpp"
#include "librm/device/remote/dr16.h"
#include "librm/device/remote/vt03.hpp"
#include "librm/device/sensor/bmi088.h"
#include "librm/device/sensor/ist8310.h"
#include "librm/device/supercap/supercap.h"
/****************/

/******** MISC MODULES ********/
#include "librm/modules/algorithm/ahrs/mahony.h"
#include "librm/modules/algorithm/ahrs/ekf.h"
#include "librm/modules/algorithm/chassis_fkik.hpp"
#include "librm/modules/algorithm/crc.h"
#include "librm/modules/algorithm/pid.hpp"
#include "librm/modules/algorithm/threshold_trigger.hpp"
#include "librm/modules/algorithm/utils.hpp"
#include "librm/modules/vofa_plotter.hpp"
/****************/

#endif  // LIBRM_HPP
