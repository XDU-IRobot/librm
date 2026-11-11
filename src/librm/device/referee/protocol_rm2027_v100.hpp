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
 * @file  librm/device/referee/protocol_rm2027_v100.hpp
 * @brief 裁判系统通信协议2027 V1.0.0（2026-10-29）
 */

#ifndef LIBRM_DEVICE_REFEREE_PROTOCOL_RM2027_V100_HPP
#define LIBRM_DEVICE_REFEREE_PROTOCOL_RM2027_V100_HPP

#include "protocol.hpp"

#include <cstddef>

#include <mapbox/eternal.hpp>

namespace rm::device {

template <>
struct RefereeCmdId<RefereeRevision::kRm2027V100> {
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
  constexpr static u16 kRobotInteraction = 0x301;
  constexpr static u16 kMapCommand = 0x303;
  constexpr static u16 kRadarInfoToClient = 0x305;
  constexpr static u16 kMapData = 0x307;
  constexpr static u16 kCustomInfo = 0x308;
  constexpr static u16 kRobotCustomData2 = 0x310;  // 图传链路
  constexpr static u16 kCustomControl = 0x311;     // 图传链路
};

struct RefereeInteractionSubCmdId2027 {
  constexpr static u16 kSentryCmd = 0x0120;
  constexpr static u16 kRadarCmd = 0x0121;
  constexpr static u16 kRobotDataFirst = 0x0200;
  constexpr static u16 kRobotDataLast = 0x02ff;
};

#pragma pack(push, 1)

template <>
struct RefereeProtocol<RefereeRevision::kRm2027V100> {
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
    u16 ally_1_robot_HP;
    u16 reserved_robot_HP;
    u16 ally_3_robot_HP;
    u16 ally_4_robot_HP;
    i16 damage_difference;
    u16 ally_7_robot_HP;
    u16 outpost_R_HP;
    u16 ally_base_HP;
    u16 outpost_B_HP;
    u16 enemy_base_HP;
    u16 ally_control_degree;
    u16 enemy_control_degree;
  } game_robot_HP;
  struct {
    u32 event_data;
    u32 event_data_2;
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
    f32 bullet_speed_limit;
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
    u16 shooter_42mm_barrel_heat;
  } power_heat_data;
  struct {
    f32 x;
    f32 y;
    f32 angle;
  } robot_pos;
  struct {
    u8 recovery_buff;
    u16 cooling_buff;
    u8 defence_buff;
    u8 vulnerability_buff;
    u16 attack_buff;
  } buff;
  struct {
    u8 armor_id : 4;
    u8 HP_deduction_reason : 4;
    u16 hit_sequence;
    i16 hit_position_x;
    i16 hit_position_y;
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
    u16 projectile_allowance_fortress;
  } projectile_allowance;
  struct {
    u32 rfid_status;
    u16 rfid_status_2;
  } rfid_status;
  struct {
    u8 dart_launch_opening_status;
    u8 reserved;
    u16 target_change_time;
    u16 latest_launch_cmd_time;
  } dart_client_cmd;
  struct {
    f32 heavyengineer_x;
    f32 heavyengineer_y;
    f32 reserved_1;
    f32 reserved_2;
    f32 standard_3_x;
    f32 standard_3_y;
    f32 standard_4_x;
    f32 standard_4_y;
    f32 reserved_3;
    f32 reserved_4;
  } ground_robot_position;
  struct {
    u16 mark_progress;
  } radar_mark_data;
  struct {
    u32 sentry_info;
    u16 sentry_info_2;
    u64 sentry_info_3;
  } sentry_info;
  struct {
    u8 radar_info;
    u8 radar_info_2;
    u16 fortify_center_x;
    u16 fortify_center_y;
    u16 fortify_radius;
  } radar_info;
  struct {
    u32 sentry_cmd;
  } sentry_cmd;
  struct {
    u8 radar_cmd;
    u8 password_cmd;
    u8 password[6];
    u8 armor_fortify_cmd;
    u16 fortify_center_x;
    u16 fortify_center_y;
  } radar_cmd;
  struct {
    u8 data[300];
  } robot_custom_data_2;
  struct {
    u8 data[60];
  } custom_control;
  struct {
    u16 data_cmd_id;
    u16 sender_id;
    u16 receiver_id;
    u8 user_data[112];
  } robot_interaction;
  struct {
    f32 target_position_x;
    f32 target_position_y;
    u8 cmd_keyboard;
    u8 target_robot_id;
    u16 cmd_source;
  } map_command;
  struct {
    u16 opponent_heavyengineer_position_x;
    u16 opponent_heavyengineer_position_y;
    u16 opponent_reserved_position_x;
    u16 opponent_reserved_position_y;
    u16 opponent_infantry_3_position_x;
    u16 opponent_infantry_3_position_y;
    u16 opponent_infantry_4_position_x;
    u16 opponent_infantry_4_position_y;
    u16 opponent_reserved_2_position_x;
    u16 opponent_reserved_2_position_y;
    u16 opponent_sentry_position_x;
    u16 opponent_sentry_position_y;
    u16 ally_heavyengineer_position_x;
    u16 ally_heavyengineer_position_y;
    u16 ally_reserved_position_x;
    u16 ally_reserved_position_y;
    u16 ally_infantry_3_position_x;
    u16 ally_infantry_3_position_y;
    u16 ally_infantry_4_position_x;
    u16 ally_infantry_4_position_y;
    u16 ally_reserved_2_position_x;
    u16 ally_reserved_2_position_y;
    u16 ally_sentry_position_x;
    u16 ally_sentry_position_y;
  } radar_info_to_client;
  struct {
    u8 intention;
    u16 start_position_x;
    u16 start_position_y;
    i8 delta_x[49];
    i8 delta_y[49];
    u16 sender_id;
  } map_data;
  struct {
    u16 sender_id;
    u16 receiver_id;
    u8 user_data[30];
  } custom_info;
};

#pragma pack(pop)

static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::game_robot_HP)) == 24);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::event_data)) == 8);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::buff)) == 7);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::hurt_data)) == 7);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::rfid_status)) == 6);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::ground_robot_position)) == 40);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::radar_info)) == 8);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::sentry_cmd)) == 4);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::radar_cmd)) == 13);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::robot_interaction)) == 118);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::custom_control)) == 60);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::robot_status)) == 17);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::game_status)) == 11);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::game_result)) == 1);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::referee_warning)) == 3);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::dart_info)) == 3);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::power_heat_data)) == 14);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::robot_pos)) == 12);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::shoot_data)) == 7);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::projectile_allowance)) == 8);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::dart_client_cmd)) == 6);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::radar_mark_data)) == 2);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::sentry_info)) == 14);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::map_command)) == 12);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::radar_info_to_client)) == 48);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::map_data)) == 105);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::custom_info)) == 34);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kRm2027V100>::robot_custom_data_2)) == 300);

template <>
struct RefereeProtocolMemoryMap<RefereeRevision::kRm2027V100> {
  // clang-format off
  static MAPBOX_ETERNAL_CONSTEXPR const auto map = mapbox::eternal::map<u16, usize>({
  {RefereeCmdId<RefereeRevision::kRm2027V100>::kGameStatus, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, game_status)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kGameResult, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, game_result)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kGameRobotHp, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, game_robot_HP)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kEventData, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, event_data)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRefereeWarning, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, referee_warning)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kDartInformation, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, dart_info)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRobotStatus, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, robot_status)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kPowerHeatData, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, power_heat_data)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRobotPos, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, robot_pos)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kBuff, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, buff)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kHurtData, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, hurt_data)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kShootData, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, shoot_data)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kProjectileAllowance, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, projectile_allowance)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRfidStatus, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, rfid_status)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kDartClientCmd, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, dart_client_cmd)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kGroundRobotPosition, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, ground_robot_position)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRadarMarkData, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, radar_mark_data)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kSentryInfo, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, sentry_info)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRadarInfo, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, radar_info)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRobotInteraction, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, robot_interaction)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kMapCommand, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, map_command)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRadarInfoToClient, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, radar_info_to_client)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kMapData, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, map_data)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kCustomInfo, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, custom_info)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRobotCustomData2, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, robot_custom_data_2)},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kCustomControl, offsetof(RefereeProtocol<RefereeRevision::kRm2027V100>, custom_control)}
  });
  // clang-format on
};

template <>
struct RefereeProtocolPayloadLength<RefereeRevision::kRm2027V100> {
  static MAPBOX_ETERNAL_CONSTEXPR const auto map = mapbox::eternal::map<u16, usize>({
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kGameStatus, 11},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kGameResult, 1},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kGameRobotHp, 24},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kEventData, 8},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRefereeWarning, 3},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kDartInformation, 3},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRobotStatus, 17},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kPowerHeatData, 14},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRobotPos, 12},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kBuff, 7},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kHurtData, 7},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kShootData, 7},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kProjectileAllowance, 8},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRfidStatus, 6},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kDartClientCmd, 6},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kGroundRobotPosition, 40},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRadarMarkData, 2},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kSentryInfo, 14},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRadarInfo, 8},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kMapCommand, 12},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRadarInfoToClient, 48},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kMapData, 105},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kCustomInfo, 34},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kRobotCustomData2, 300},
      {RefereeCmdId<RefereeRevision::kRm2027V100>::kCustomControl, 60}
  });
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_REFEREE_PROTOCOL_RM2027_V100_HPP
