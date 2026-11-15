/**
 *******************************************************************************
 * @file      : m3508.hpp
 * @brief     : RoboMaster M3508 + C620 电机类
 * @attention :
 *  1. 本类只负责反馈解析、目标转速和速度环计算，不直接发送 CAN。
 *  2. 0x200 电流帧由 Chassis / HW_can 统一打包发送。
 *  3. 反馈帧标准 ID 为 0x201 ~ 0x204，对应电机 ID 1 ~ 4。
 *******************************************************************************
 */

#ifndef COMPONENTS_MOTOR_M3508_HPP_
#define COMPONENTS_MOTOR_M3508_HPP_

#include <cstdint>

#include "pid.hpp"

namespace hello_world
{
namespace motor
{

class M3508
{
 public:
  M3508();
  explicit M3508(uint8_t id);

  /**
   * @brief       解析 C620 反馈帧（8 字节）
   * @param       data: CAN 数据
   * @note        在 CAN RX 中断中调用
   */
  void update(const uint8_t data[8]);

  /**
   * @brief       设置目标转子转速
   * @param       rpm: 减速箱前的转子转速，单位 rpm
   */
  void setTargetRpm(float rpm);

  /**
   * @brief       速度环计算
   * @param       dt: 控制周期，单位 s
   * @retval      -16384 ~ 16384 的电流指令
   */
  float calcCurrentCommand(float dt);

  /**
   * @brief       设置速度环增益
   */
  void setGains(float kp, float ki, float kd);

  /**
   * @brief       设置电流指令限幅，默认 3000（约 3.7 A）
   */
  void setCurrentLimit(float limit);

  /**
   * @brief       清空目标、积分和上一次误差
   */
  void reset();

  uint8_t id() const { return id_; }
  float targetRpm() const { return target_rpm_; }
  float rotorRpm() const { return rotor_rpm_; }
  float current() const { return current_; }
  float temperature() const { return temperature_; }
  uint16_t angle() const { return angle_; }

  /**
   * @brief       最近 100 ms 内是否收到过反馈
   */
  bool online() const;

 private:
  static constexpr float kMaxRotorRpm = 9000.0f;
  static constexpr float kMaxCurrentCmd = 16384.0f;

  void initPid();

  uint8_t id_ = 0;

  /* 反馈数据 */
  uint16_t angle_ = 0;
  float rotor_rpm_ = 0.0f;
  float current_ = 0.0f;
  float temperature_ = 0.0f;
  uint32_t last_rx_tick_ = 0;
  bool has_rx_ = false;

  /* 目标与速度环 */
  float target_rpm_ = 0.0f;
  hello_world::control::PID speed_pid_;
};

}  // namespace motor
}  // namespace hello_world

#endif  // COMPONENTS_MOTOR_M3508_HPP_
