/**
 *******************************************************************************
 * @file      : pid.cpp
 * @brief     : 通用 PID 控制器
 *******************************************************************************
 */

#include "pid.hpp"

namespace hello_world
{
namespace control
{

void PID::setGains(float kp, float ki, float kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;

  /* 改增益时清掉积分，避免旧积分造成冲击 */
  integral_ = 0.0f;
  has_last_measure_ = false;
}

void PID::setOutputLimit(float min, float max)
{
  if (min > max)
  {
    const float tmp = min;
    min = max;
    max = tmp;
  }

  out_min_ = min;
  out_max_ = max;
}

void PID::setIntegralLimit(float min, float max)
{
  if (min > max)
  {
    const float tmp = min;
    min = max;
    max = tmp;
  }

  integral_min_ = min;
  integral_max_ = max;
}

void PID::reset()
{
  integral_ = 0.0f;
  last_measure_ = 0.0f;
  output_ = 0.0f;
  has_last_measure_ = false;
}

float PID::calc(float target, float measure, float dt)
{
  if (dt <= 0.0f || dt > 0.1f)
  {
    dt = 0.001f;
  }

  const float error = target - measure;

  integral_ += error * dt;
  if (integral_ > integral_max_)
  {
    integral_ = integral_max_;
  }
  else if (integral_ < integral_min_)
  {
    integral_ = integral_min_;
  }

  /* 微分项用测量值微分，避免目标突变时产生冲击 */
  float derivative = 0.0f;
  if (has_last_measure_)
  {
    derivative = (measure - last_measure_) / dt;
  }
  last_measure_ = measure;
  has_last_measure_ = true;

  output_ = kp_ * error + ki_ * integral_ - kd_ * derivative;

  if (output_ > out_max_)
  {
    output_ = out_max_;
  }
  else if (output_ < out_min_)
  {
    output_ = out_min_;
  }

  return output_;
}

}  // namespace control
}  // namespace hello_world
