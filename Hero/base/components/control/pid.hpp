/**
 *******************************************************************************
 * @file      : pid.hpp
 * @brief     : 通用 PID 控制器
 * @attention :
 *  1. 微分项使用“测量值微分”，避免目标突变时产生微分冲击。
 *  2. 输出和积分都有限幅，防止积分饱和。
 *******************************************************************************
 */

#ifndef COMPONENTS_CONTROL_PID_HPP_
#define COMPONENTS_CONTROL_PID_HPP_

namespace hello_world
{
namespace control
{

class PID
{
 public:
  PID() = default;

  PID(float kp, float ki, float kd, float out_min, float out_max)
      : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max)
  {
  }

  void setGains(float kp, float ki, float kd);
  void setOutputLimit(float min, float max);
  void setIntegralLimit(float min, float max);

  void reset();

  /**
   * @brief       计算 PID 输出
   * @param       target: 目标值
   * @param       measure: 测量值
   * @param       dt: 控制周期，单位 s
   * @retval      限幅后的输出
   */
  float calc(float target, float measure, float dt);

  float output() const { return output_; }

 private:
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;

  float out_min_ = -1.0e9f;
  float out_max_ = 1.0e9f;

  float integral_min_ = -1.0e9f;
  float integral_max_ = 1.0e9f;

  float integral_ = 0.0f;
  float last_measure_ = 0.0f;
  float output_ = 0.0f;
  bool has_last_measure_ = false;
};

}  // namespace control
}  // namespace hello_world

#endif  // COMPONENTS_CONTROL_PID_HPP_
