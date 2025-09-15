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
 * @file  librm/device/referee/protocol_v164.hpp
 * @brief 裁判系统串口协议V1.6.4(2024-7-15)
 */

#ifndef LIBRM_DEVICE_REFEREE_PROTOCOL_V164_HPP
#define LIBRM_DEVICE_REFEREE_PROTOCOL_V164_HPP

#include "protocol.hpp"

#include <cstddef>

namespace rm::device {

template <>
struct RefereeCmdId<RefereeRevision::kV164> {
  constexpr static u16 kGameStatus = 0x1;
  constexpr static u16 kGameResult = 0x2;
  constexpr static u16 kGameRobotHp = 0x3;
  constexpr static u16 kEventData = 0x101;
  constexpr static u16 kExtSupplyProjectileAction = 0x102;
  constexpr static u16 kRefereeWarning = 0x104;
  constexpr static u16 kDartInformation = 0x105;
  constexpr static u16 kRobotStatus = 0x201;
  constexpr static u16 kPowerHeatData = 0x202;
  constexpr static u16 kRobotPos = 0x203;
  constexpr static u16 kBuff = 0x204;
  constexpr static u16 kAirSupportData = 0x205;
  constexpr static u16 kHurtData = 0x206;
  constexpr static u16 kShootData = 0x207;
  constexpr static u16 kProjectileAllowance = 0x208;
  constexpr static u16 kRfidStatus = 0x209;
  constexpr static u16 kDartClientCmd = 0x20a;
  constexpr static u16 kGroundRobotPosition = 0x20b;
  constexpr static u16 kRadarMarkData = 0x20c;
  constexpr static u16 kSentryInfo = 0x20d;
  constexpr static u16 kRadarInfo = 0x20e;
  constexpr static u16 kCustomRobotData = 0x302;  // 图传链路
  constexpr static u16 kMapCommand = 0x303;
  constexpr static u16 kRemoteControl = 0x304;  // 图传链路
};

#pragma pack(push, 1)

template <>
struct RefereeProtocol<RefereeRevision::kV164> {
  struct {
    u8 game_type : 4;
    u8 game_progress : 4;
    u16 stage_remain_time;
    u64 SyncTimeStamp;
  } game_status;
  struct {
    u8 winner;
  } game_result;
  struct {
    u16 red_1_robot_HP;
    u16 red_2_robot_HP;
    u16 red_3_robot_HP;
    u16 red_4_robot_HP;
    u16 red_5_robot_HP;
    u16 red_7_robot_HP;
    u16 red_outpost_HP;
    u16 red_base_HP;
    u16 blue_1_robot_HP;
    u16 blue_2_robot_HP;
    u16 blue_3_robot_HP;
    u16 blue_4_robot_HP;
    u16 blue_5_robot_HP;
    u16 blue_7_robot_HP;
    u16 blue_outpost_HP;
    u16 blue_base_HP;
  } game_robot_HP;
  struct {
    u32 event_data;
  } event_data;
  struct {
    u8 reserved;
    u8 supply_robot_id;
    u8 supply_projectile_step;
    u8 supply_projectile_num;
  } ext_supply_projectile_action;
  struct {
    u8 level;
    u8 offending_robot_id;
    u8 count;
  } referee_warning;
  struct {
    u8 dart_remaining_time;
    u16 dart_info;
  } dart_info;
  struct {
    u8 robot_id;
    u8 robot_level;
    u16 current_HP;
    u16 maximum_HP;
    u16 shooter_barrel_cooling_value;
    u16 shooter_barrel_heat_limit;
    u16 chassis_power_limit;
    u8 power_management_gimbal_output : 1;
    u8 power_management_chassis_output : 1;
    u8 power_management_shooter_output : 1;
  } robot_status;
  struct {
    u16 chasiss_voltage;
    u16 chassis_current;
    f32 chassis_power;
    u16 buffer_energy;
    u16 shooter_17mm_1_barrel_heat;
    u16 shooter_17mm_2_barrel_heat;
    u16 shooter_42mm_barrel_heat;
  } power_heat_data;
  struct {
    f32 x;
    f32 y;
    f32 angle;
  } robot_pos;
  struct {
    u8 recovery_buff;
    u8 cooling_buff;
    u8 defence_buff;
    u8 vulnerability_buff;
    u16 attack_buff;
  } buff;
  struct {
    u8 airforce_status;
    u8 time_remain;
  } air_support_data;
  struct {
    u8 armor_id : 4;
    u8 HP_deduction_reason : 4;
  } hurt_data;
  struct {
    u8 bullet_type;
    u8 shooter_number;
    u8 launching_frequency;
    f32 initial_speed;
  } shoot_data;
  struct {
    u16 projectile_allowance_17mm;
    u16 projectile_allowance_42mm;
    u16 remaining_gold_coin;
  } projectile_allowance;
  struct {
    u32 rfid_status;
  } rfid_status;
  struct {
    u8 dart_launch_opening_status;
    u8 reserved;
    u16 target_change_time;
    u16 latest_launch_cmd_time;
  } dart_client_cmd;
  struct {
    f32 hero_x;
    f32 hero_y;
    f32 engineer_x;
    f32 engineer_y;
    f32 standard_3_x;
    f32 standard_3_y;
    f32 standard_4_x;
    f32 standard_4_y;
    f32 standard_5_x;
    f32 standard_5_y;
  } ground_robot_position;
  struct {
    u8 mark_hero_progress;
    u8 mark_engineer_progress;
    u8 mark_standard_3_progress;
    u8 mark_standard_4_progress;
    u8 mark_standard_5_progress;
  } radar_mark_data;
  struct {
    u32 sentry_info;
    u16 sentry_info_2;
  } sentry_info;
  struct {
    u8 radar_info;
  } radar_info;
  struct {
    u8 data[30];
  } custom_robot_data;
  struct {
    f32 target_position_x;
    f32 target_position_y;
    u8 cmd_keyboard;
    u8 target_robot_id;
    u16 cmd_source;
  } map_command;
  struct {
    i16 mouse_x;
    i16 mouse_y;
    i16 mouse_z;
    i8 left_button_down;
    i8 right_button_down;
    u16 keyboard_value;
    u16 reserved;
  } remote_control;
};

#pragma pack(pop)

// clang-format off
template <>
const inline std::unordered_map<u16, usize> referee_protocol_memory_map<RefereeRevision::kV164>{
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

#endif  // LIBRM_DEVICE_REFEREE_PROTOCOL_V164_HPP