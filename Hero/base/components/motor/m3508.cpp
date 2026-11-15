/**
 *******************************************************************************
 * @file      : m3508.cpp
 * @brief     : RoboMaster M3508 + C620 电机类
 *******************************************************************************
 */

#include "m3508.hpp"

#include <cmath>

#include "stm32f4xx_hal.h"

namespace hello_world
{
namespace motor
{

M3508::M3508() { initPid(); }

M3508::M3508(uint8_t id) : id_(id) { initPid(); }

void M3508::initPid()
{
  speed_pid_.setGains(2.0f, 0.0f, 0.0f);
  speed_pid_.setOutputLimit(-3000.0f, 3000.0f);
  speed_pid_.setIntegralLimit(-10000.0f, 10000.0f);
}

void M3508::update(const uint8_t data[8])
{
  angle_ = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);

  rotor_rpm_ = static_cast<float>(
      static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]));

  current_ = static_cast<float>(
      static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]));

  temperature_ = static_cast<float>(data[6]);

  last_rx_tick_ = HAL_GetTick();
  has_rx_ = true;
}

void M3508::setTargetRpm(float rpm)
{
  if (rpm > kMaxRotorRpm)
  {
    rpm = kMaxRotorRpm;
  }
  else if (rpm < -kMaxRotorRpm)
  {
    rpm = -kMaxRotorRpm;
  }

  target_rpm_ = rpm;
}

float M3508::calcCurrentCommand(float dt)
{
  if (!online())
  {
    speed_pid_.reset();
    return 0.0f;
  }

  /* 目标为 0 且转速接近 0 时清积分，避免静止时来回抖 */
  if (target_rpm_ == 0.0f && std::fabs(rotor_rpm_) < 30.0f)
  {
    speed_pid_.reset();
    return 0.0f;
  }

  return speed_pid_.calc(target_rpm_, rotor_rpm_, dt);
}

void M3508::setGains(float kp, float ki, float kd)
{
  speed_pid_.setGains(kp, ki, kd);
}

void M3508::setCurrentLimit(float limit)
{
  if (limit < 0.0f)
  {
    limit = 0.0f;
  }
  else if (limit > kMaxCurrentCmd)
  {
    limit = kMaxCurrentCmd;
  }

  speed_pid_.setOutputLimit(-limit, limit);
  speed_pid_.setIntegralLimit(-limit * 3.0f, limit * 3.0f);
}

void M3508::reset()
{
  target_rpm_ = 0.0f;
  speed_pid_.reset();
}

bool M3508::online() const
{
  return has_rx_ && (HAL_GetTick() - last_rx_tick_ < 100u);
}

}  // namespace motor
}  // namespace hello_world
