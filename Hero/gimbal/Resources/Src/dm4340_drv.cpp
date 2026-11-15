/**
 *******************************************************************************
 * @file      : dm4340_drv.cpp
 * @brief     : 达妙 DM-J4340P-2EC 驱动
 *******************************************************************************
 */

#include "dm4340_drv.hpp"

#include <cstring>

namespace
{

uint16_t dm4340_float_to_uint(float x, float x_min, float x_max, int bits)
{
  if (x < x_min)
  {
    x = x_min;
  }
  else if (x > x_max)
  {
    x = x_max;
  }

  const float span = x_max - x_min;
  if (span <= 0.0f)
  {
    return 0;
  }

  return static_cast<uint16_t>((x - x_min) *
                               static_cast<float>((1u << bits) - 1u) / span);
}

float dm4340_uint_to_float(uint16_t x, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  return static_cast<float>(x) * span /
             static_cast<float>((1u << bits) - 1u) +
         x_min;
}

}  // namespace

DM4340::DM4340(CAN_HandleTypeDef* hcan, uint8_t id)
    : hcan_(hcan), id_(id)
{
}

void DM4340::setMitLimits(float p_max, float v_max, float t_max)
{
  if (p_max > 0.0f)
  {
    p_max_ = p_max;
  }
  if (v_max > 0.0f)
  {
    v_max_ = v_max;
  }
  if (t_max > 0.0f)
  {
    t_max_ = t_max;
  }
}

void DM4340::enable()
{
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  CAN_Send_Msg(hcan_, data, static_cast<uint32_t>(id_ + kMitMode), 8);
}

void DM4340::disable()
{
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  CAN_Send_Msg(hcan_, data, static_cast<uint32_t>(id_ + kMitMode), 8);
}

void DM4340::mit(float pos, float vel, float kp, float kd, float torque)
{
  const uint16_t pos_tmp = dm4340_float_to_uint(pos, -p_max_, p_max_, 16);
  const uint16_t vel_tmp = dm4340_float_to_uint(vel, -v_max_, v_max_, 12);
  const uint16_t kp_tmp = dm4340_float_to_uint(kp, 0.0f, kKpMax, 12);
  const uint16_t kd_tmp = dm4340_float_to_uint(kd, 0.0f, kKdMax, 12);
  const uint16_t tor_tmp = dm4340_float_to_uint(torque, -t_max_, t_max_, 12);

  uint8_t data[8];
  data[0] = static_cast<uint8_t>(pos_tmp >> 8);
  data[1] = static_cast<uint8_t>(pos_tmp & 0xFF);
  data[2] = static_cast<uint8_t>(vel_tmp >> 4);
  data[3] = static_cast<uint8_t>(((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8));
  data[4] = static_cast<uint8_t>(kp_tmp & 0xFF);
  data[5] = static_cast<uint8_t>(kd_tmp >> 4);
  data[6] = static_cast<uint8_t>(((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8));
  data[7] = static_cast<uint8_t>(tor_tmp & 0xFF);

  CAN_Send_Msg(hcan_, data, static_cast<uint32_t>(id_ + kMitMode), 8);
}

void DM4340::posSpeed(float pos, float vel)
{
  uint8_t data[8];
  std::memcpy(&data[0], &pos, sizeof(float));
  std::memcpy(&data[4], &vel, sizeof(float));

  CAN_Send_Msg(hcan_, data, static_cast<uint32_t>(id_ + kPosMode), 8);
}

void DM4340::velocity(float vel)
{
  uint8_t data[4];
  std::memcpy(data, &vel, sizeof(float));

  CAN_Send_Msg(hcan_, data, static_cast<uint32_t>(id_ + kSpeedMode), 4);
}

void DM4340::update(const uint8_t data[8])
{
  error_ = static_cast<uint8_t>(data[0] >> 4);

  position_raw_ = static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) |
                                        data[2]);
  velocity_raw_ = static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 4) |
                                        (data[4] >> 4));
  torque_raw_ = static_cast<uint16_t>(((data[4] & 0x0F) << 8) | data[5]);

  position_ = dm4340_uint_to_float(position_raw_, -p_max_, p_max_, 16);
  velocity_ = dm4340_uint_to_float(velocity_raw_, -v_max_, v_max_, 12);
  torque_ = dm4340_uint_to_float(torque_raw_, -t_max_, t_max_, 12);

  temp_mos_ = static_cast<float>(data[6]);
  temp_rotor_ = static_cast<float>(data[7]);

  last_rx_tick_ = HAL_GetTick();
  has_rx_ = true;
}

bool DM4340::online() const
{
  return has_rx_ && (HAL_GetTick() - last_rx_tick_ < 100u);
}
