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
 * @file  librm/device/referee/protocol_v170.cc
 * @brief 裁判系统串口协议V1.7.0(2024-12-25)
 */

#include "protocol_v170.hpp"

namespace rm::device {

// clang-format off
template <>
const etl::unordered_map<u16, usize, kRefProtocolMaxCmdIdEntries> referee_protocol_memory_map<RefereeRevision::kV170>{
    {RefereeCmdId<RefereeRevision::kV170>::kGameStatus, offsetof(RefereeProtocol<RefereeRevision::kV170>, game_status)},
    {RefereeCmdId<RefereeRevision::kV170>::kGameResult, offsetof(RefereeProtocol<RefereeRevision::kV170>, game_result)},
    {RefereeCmdId<RefereeRevision::kV170>::kGameRobotHp, offsetof(RefereeProtocol<RefereeRevision::kV170>, game_robot_HP)},
    {RefereeCmdId<RefereeRevision::kV170>::kEventData, offsetof(RefereeProtocol<RefereeRevision::kV170>, event_data)},
    {RefereeCmdId<RefereeRevision::kV170>::kRefereeWarning, offsetof(RefereeProtocol<RefereeRevision::kV170>, referee_warning)},
    {RefereeCmdId<RefereeRevision::kV170>::kDartInformation, offsetof(RefereeProtocol<RefereeRevision::kV170>, dart_info)},
    {RefereeCmdId<RefereeRevision::kV170>::kRobotStatus, offsetof(RefereeProtocol<RefereeRevision::kV170>, robot_status)},
    {RefereeCmdId<RefereeRevision::kV170>::kPowerHeatData, offsetof(RefereeProtocol<RefereeRevision::kV170>, power_heat_data)},
    {RefereeCmdId<RefereeRevision::kV170>::kRobotPos, offsetof(RefereeProtocol<RefereeRevision::kV170>, robot_pos)},
    {RefereeCmdId<RefereeRevision::kV170>::kBuff, offsetof(RefereeProtocol<RefereeRevision::kV170>, buff)},
    {RefereeCmdId<RefereeRevision::kV170>::kHurtData, offsetof(RefereeProtocol<RefereeRevision::kV170>, hurt_data)},
    {RefereeCmdId<RefereeRevision::kV170>::kShootData, offsetof(RefereeProtocol<RefereeRevision::kV170>, shoot_data)},
    {RefereeCmdId<RefereeRevision::kV170>::kProjectileAllowance, offsetof(RefereeProtocol<RefereeRevision::kV170>, projectile_allowance)},
    {RefereeCmdId<RefereeRevision::kV170>::kRfidStatus, offsetof(RefereeProtocol<RefereeRevision::kV170>, rfid_status)},
    {RefereeCmdId<RefereeRevision::kV170>::kDartClientCmd, offsetof(RefereeProtocol<RefereeRevision::kV170>, dart_client_cmd)},
    {RefereeCmdId<RefereeRevision::kV170>::kGroundRobotPosition, offsetof(RefereeProtocol<RefereeRevision::kV170>, ground_robot_position)},
    {RefereeCmdId<RefereeRevision::kV170>::kRadarMarkData, offsetof(RefereeProtocol<RefereeRevision::kV170>, radar_mark_data)},
    {RefereeCmdId<RefereeRevision::kV170>::kSentryInfo, offsetof(RefereeProtocol<RefereeRevision::kV170>, sentry_info)},
    {RefereeCmdId<RefereeRevision::kV170>::kRadarInfo, offsetof(RefereeProtocol<RefereeRevision::kV170>, radar_info)},
    {RefereeCmdId<RefereeRevision::kV170>::kCustomRobotData, offsetof(RefereeProtocol<RefereeRevision::kV170>, custom_robot_data)},
    {RefereeCmdId<RefereeRevision::kV170>::kMapCommand, offsetof(RefereeProtocol<RefereeRevision::kV170>, map_command)},
    {RefereeCmdId<RefereeRevision::kV170>::kRemoteControl, offsetof(RefereeProtocol<RefereeRevision::kV170>, remote_control)}
};
// clang-format on

}  // namespace rm::device
