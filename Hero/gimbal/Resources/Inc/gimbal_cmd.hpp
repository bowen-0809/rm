/**
 *******************************************************************************
 * @file      : gimbal_cmd.hpp
 * @brief     : 底盘 -> 云台 CAN 命令帧
 * @attention :
 *  1. 下板通过 CAN1 以 100 Hz 发送，上板收到后刷新 IWDG。
 *  2. yaw_rate 单位 0.001 rad/s，pitch 单位 0.001 rad。
 *******************************************************************************
 */

#ifndef GIMBAL_CMD_HPP_
#define GIMBAL_CMD_HPP_

#include <cstdint>

#pragma pack(push, 1)
struct GimbalCmd
{
  int16_t yaw_rate_mrad;  // 0.001 rad/s
  int16_t pitch_mrad;     // 0.001 rad/s，Pitch 速度指令
  uint8_t enable;         // bit0=使能，bit2:1=模式(0分离/1跟随/2小陀螺)
  uint8_t estop;          // 1 = 急停
  uint8_t counter;        // 高 2 位：模式(0 分离/1 跟随/2 小陀螺)，低 6 位：计数
  uint8_t reserved;       // int8 底盘实际 wz，单位 0.075 rad/s
};
#pragma pack(pop)

static constexpr uint32_t kGimbalCmdId = 0x7FE;  // 底盘 -> 云台命令/心跳

#endif  // GIMBAL_CMD_HPP_
