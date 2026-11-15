/**
 *******************************************************************************
 * @file      : chassis.cpp
 * @brief     : 四麦轮底盘运动学与四轮速度环
 *******************************************************************************
 */

#include "chassis.hpp"

#include <cmath>

namespace hello_world
{
namespace chassis
{

void Chassis::init(hello_world::motor::M3508* wheels)
{
  wheel_ = wheels;
  reset();
}

void Chassis::setVelocity(float vx, float vy, float wz)
{
  if (vx > kMaxLinearSpeed)
  {
    vx = kMaxLinearSpeed;
  }
  else if (vx < -kMaxLinearSpeed)
  {
    vx = -kMaxLinearSpeed;
  }

  if (vy > kMaxLinearSpeed)
  {
    vy = kMaxLinearSpeed;
  }
  else if (vy < -kMaxLinearSpeed)
  {
    vy = -kMaxLinearSpeed;
  }

  if (wz > kMaxAngularSpeed)
  {
    wz = kMaxAngularSpeed;
  }
  else if (wz < -kMaxAngularSpeed)
  {
    wz = -kMaxAngularSpeed;
  }

  vx_ = vx;
  vy_ = vy;
  wz_ = wz;
}

void Chassis::update(float dt)
{
  if (wheel_ == nullptr)
  {
    for (int i = 0; i < 4; ++i)
    {
      current_cmd_[i] = 0;
    }
    return;
  }

  constexpr float kPi = 3.14159265358979323846f;
  const float a_b = kHalfBasePlusTrack;

  /* 四麦轮逆解：轮子“向前滚动”的线速度，单位 m/s */
  const float wheel_speed[4] = {
      vx_ - vy_ - wz_ * a_b,  // 左前
      vx_ + vy_ - wz_ * a_b,  // 左后
      vx_ - vy_ + wz_ * a_b,  // 右后
      vx_ + vy_ + wz_ * a_b,  // 右前
  };

  /* 线速度 -> 转子 rpm，并补偿右侧电机安装方向 */
  const float wheel_to_rotor = (60.0f * kGearRatio) / (2.0f * kPi * kWheelRadius);

  float rotor_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float max_abs_rpm = 0.0f;

  for (int i = 0; i < 4; ++i)
  {
    rotor_rpm[i] = wheel_speed[i] * wheel_to_rotor * kWheelDirection[i];
    const float abs_rpm = std::fabs(rotor_rpm[i]);
    if (abs_rpm > max_abs_rpm)
    {
      max_abs_rpm = abs_rpm;
    }
  }

  /* 任一目标超过最大转子转速时，四轮等比缩小，保持运动方向 */
  if (max_abs_rpm > kMaxRotorRpm)
  {
    const float scale = kMaxRotorRpm / max_abs_rpm;
    for (int i = 0; i < 4; ++i)
    {
      rotor_rpm[i] *= scale;
    }
  }

  for (int i = 0; i < 4; ++i)
  {
    wheel_[i].setTargetRpm(rotor_rpm[i]);
    current_cmd_[i] =
        static_cast<int16_t>(wheel_[i].calcCurrentCommand(dt));
  }
}

void Chassis::reset()
{
  vx_ = 0.0f;
  vy_ = 0.0f;
  wz_ = 0.0f;

  for (int i = 0; i < 4; ++i)
  {
    current_cmd_[i] = 0;
    if (wheel_ != nullptr)
    {
      wheel_[i].setTargetRpm(0.0f);
      wheel_[i].reset();
    }
  }
}

}  // namespace chassis
}  // namespace hello_world
