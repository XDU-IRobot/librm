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
 * @file  librm/device/referee/protocol_v170.hpp
 * @brief 裁判系统串口协议V1.7.0(2024-12-25)
 */

#ifndef LIBRM_DEVICE_REFEREE_PROTOCOL_V170_HPP
#define LIBRM_DEVICE_REFEREE_PROTOCOL_V170_HPP

#include "protocol.hpp"

#include <cstddef>

namespace rm::device {

template <>
struct RefereeCmdId<RefereeRevision::kV170> {
  constexpr static u16 kGameStatus = 0x1;
  constexpr static u16 kGameResult = 0x2;
  constexpr static u16 kGameRobotHp = 0x3;
  constexpr static u16 kEventData = 0x101;
  constexpr static u16 kRefereeWarning = 0x104;
  constexpr static u16 kDartInformation = 0x105;
  constexpr static u16 kRobotStatus = 0x201;
  constexpr static u16 kPowerHeatData = 0x202;
  constexpr static u16 kRobotPos = 0x203;
  constexpr static u16 kBuff = 0x204;
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
struct RefereeProtocol<RefereeRevision::kV170> {
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
    u16 reserved;
    u16 red_7_robot_HP;
    u16 red_outpost_HP;
    u16 red_base_HP;
    u16 blue_1_robot_HP;
    u16 blue_2_robot_HP;
    u16 blue_3_robot_HP;
    u16 blue_4_robot_HP;
    u16 reserved_2;
    u16 blue_7_robot_HP;
    u16 blue_outpost_HP;
    u16 blue_base_HP;
  } game_robot_HP;
  struct {
    u32 event_data;
  } event_data;
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
    u16 reserved;
    u16 reserved_2;
    f32 reserved_3;
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
    u8 remaining_energy;
  } buff;
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
    f32 reserved;
    f32 reserved_2;
  } ground_robot_position;
  struct {
    u8 mark_progress;
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

}  // namespace rm::device

#endif  // LIBRM_DEVICE_REFEREE_PROTOCOL_V170_HPP