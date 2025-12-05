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
 * @file  librm/device/referee/protocol_v164.cc
 * @brief 裁判系统串口协议V1.6.4(2024-7-15)
 */

#include "protocol_v164.hpp"

namespace rm::device {

// clang-format off
template <>
const etl::unordered_map<u16, usize, kRefProtocolMaxCmdIdEntries> referee_protocol_memory_map<RefereeRevision::kV164>{
    {RefereeCmdId<RefereeRevision::kV164>::kGameStatus, offsetof(RefereeProtocol<RefereeRevision::kV164>, game_status)},
    {RefereeCmdId<RefereeRevision::kV164>::kGameResult, offsetof(RefereeProtocol<RefereeRevision::kV164>, game_result)},
    {RefereeCmdId<RefereeRevision::kV164>::kGameRobotHp, offsetof(RefereeProtocol<RefereeRevision::kV164>, game_robot_HP)},
    {RefereeCmdId<RefereeRevision::kV164>::kEventData, offsetof(RefereeProtocol<RefereeRevision::kV164>, event_data)},
    {RefereeCmdId<RefereeRevision::kV164>::kExtSupplyProjectileAction, offsetof(RefereeProtocol<RefereeRevision::kV164>, ext_supply_projectile_action)},
    {RefereeCmdId<RefereeRevision::kV164>::kRefereeWarning, offsetof(RefereeProtocol<RefereeRevision::kV164>, referee_warning)},
    {RefereeCmdId<RefereeRevision::kV164>::kDartInformation, offsetof(RefereeProtocol<RefereeRevision::kV164>, dart_info)},
    {RefereeCmdId<RefereeRevision::kV164>::kRobotStatus, offsetof(RefereeProtocol<RefereeRevision::kV164>, robot_status)},
    {RefereeCmdId<RefereeRevision::kV164>::kPowerHeatData, offsetof(RefereeProtocol<RefereeRevision::kV164>, power_heat_data)},
    {RefereeCmdId<RefereeRevision::kV164>::kRobotPos, offsetof(RefereeProtocol<RefereeRevision::kV164>, robot_pos)},
    {RefereeCmdId<RefereeRevision::kV164>::kBuff, offsetof(RefereeProtocol<RefereeRevision::kV164>, buff)},
    {RefereeCmdId<RefereeRevision::kV164>::kAirSupportData, offsetof(RefereeProtocol<RefereeRevision::kV164>, air_support_data)},
    {RefereeCmdId<RefereeRevision::kV164>::kHurtData, offsetof(RefereeProtocol<RefereeRevision::kV164>, hurt_data)},
    {RefereeCmdId<RefereeRevision::kV164>::kShootData, offsetof(RefereeProtocol<RefereeRevision::kV164>, shoot_data)},
    {RefereeCmdId<RefereeRevision::kV164>::kProjectileAllowance, offsetof(RefereeProtocol<RefereeRevision::kV164>, projectile_allowance)},
    {RefereeCmdId<RefereeRevision::kV164>::kRfidStatus, offsetof(RefereeProtocol<RefereeRevision::kV164>, rfid_status)},
    {RefereeCmdId<RefereeRevision::kV164>::kDartClientCmd, offsetof(RefereeProtocol<RefereeRevision::kV164>, dart_client_cmd)},
    {RefereeCmdId<RefereeRevision::kV164>::kGroundRobotPosition, offsetof(RefereeProtocol<RefereeRevision::kV164>, ground_robot_position)},
    {RefereeCmdId<RefereeRevision::kV164>::kRadarMarkData, offsetof(RefereeProtocol<RefereeRevision::kV164>, radar_mark_data)},
    {RefereeCmdId<RefereeRevision::kV164>::kSentryInfo, offsetof(RefereeProtocol<RefereeRevision::kV164>, sentry_info)},
    {RefereeCmdId<RefereeRevision::kV164>::kRadarInfo, offsetof(RefereeProtocol<RefereeRevision::kV164>, radar_info)},
    {RefereeCmdId<RefereeRevision::kV164>::kCustomRobotData, offsetof(RefereeProtocol<RefereeRevision::kV164>, custom_robot_data)},
    {RefereeCmdId<RefereeRevision::kV164>::kMapCommand, offsetof(RefereeProtocol<RefereeRevision::kV164>, map_command)},
    {RefereeCmdId<RefereeRevision::kV164>::kRemoteControl, offsetof(RefereeProtocol<RefereeRevision::kV164>, remote_control)}
};
// clang-format on

}  // namespace rm::device
