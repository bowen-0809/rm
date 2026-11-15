/**
 *******************************************************************************
 * @file      : chassis.hpp
 * @brief     : 四麦轮底盘运动学与四轮速度环
 * @attention :
 *  1. 坐标系：x 向前，y 向左，z 向上，wz 正方向为逆时针。
 *  2. 轮序：左前 ID1、左后 ID2、右后 ID3、右前 ID4。
 *  3. 右侧电机安装方向与左侧相反，因此 kWheelDirection 右侧取 -1。
 *******************************************************************************
 */

#ifndef COMPONENTS_CHASSIS_CHASSIS_HPP_
#define COMPONENTS_CHASSIS_CHASSIS_HPP_

#include <cstdint>

#include "m3508.hpp"

namespace hello_world
{
namespace chassis
{

class Chassis
{
 public:
  Chassis() = default;

  /**
   * @brief       绑定四个 M3508 实例
   * @param       wheels: 指向 wheel[4] 的指针
   */
  void init(hello_world::motor::M3508* wheels);

  /**
   * @brief       设置底盘速度指令
   * @param       vx: 前进速度，m/s
   * @param       vy: 左移速度，m/s
   * @param       wz: 自转角速度，rad/s
   */
  void setVelocity(float vx, float vy, float wz);

  /**
   * @brief       四麦轮逆解 + 四轮速度环
   * @param       dt: 控制周期，单位 s
   */
  void update(float dt);

  /**
   * @brief       清零目标和 PID，电流指令全部置 0
   */
  void reset();

  const int16_t* currentCommand() const { return current_cmd_; }

 private:
  static constexpr float kWheelRadius = 0.07786f;       // m
  static constexpr float kGearRatio = 14.0f;            // 按 PDF
  static constexpr float kHalfBasePlusTrack = 0.5f;     // (0.45 + 0.55) / 2
  static constexpr float kMaxRotorRpm = 9000.0f;
  static constexpr float kMaxLinearSpeed = 3.0f;        // m/s
  static constexpr float kMaxAngularSpeed = 9.0f;       // rad/s，支持小陀螺 60 rpm 和更快跟随

  /**
   * 电机安装方向：右侧两个轮子与左侧相反。
   * 如果以后改了接线，只改这里。
   */
  static constexpr float kWheelDirection[4] = {1.0f, 1.0f, -1.0f, -1.0f};

  hello_world::motor::M3508* wheel_ = nullptr;

  float vx_ = 0.0f;
  float vy_ = 0.0f;
  float wz_ = 0.0f;

  int16_t current_cmd_[4] = {0, 0, 0, 0};
};

}  // namespace chassis
}  // namespace hello_world

#endif  // COMPONENTS_CHASSIS_CHASSIS_HPP_
