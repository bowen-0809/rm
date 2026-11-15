/**
 *******************************************************************************
 * @file      : dm4340_drv.hpp
 * @brief     : 达妙 DM-J4340P-2EC 驱动（MIT / 位置速度 / 速度）
 * @attention :
 *  1. 默认 PMAX=12.5 rad，VMAX=8 rad/s，TMAX=28 Nm（24V 型号）；
 *     如果达妙调试助手里改过这三个参数，必须调用 setMitLimits() 同步。
 *  2. MIT 模式控制帧 8 字节，位置 16 bit、速度/Kp/Kd/扭矩各 12 bit。
 *  3. 反馈帧 ID 默认为电机 ID + 0x10。
 *******************************************************************************
 */

#ifndef __DM4340_DRV_H__
#define __DM4340_DRV_H__

#include "HW_can.hpp"
#include "system_user.hpp"

class DM4340
{
 public:
  DM4340(CAN_HandleTypeDef* hcan, uint8_t id);

  void enable();
  void disable();

  /**
   * @brief       MIT 模式控制
   * @param       pos: 目标位置，rad
   * @param       vel: 目标速度，rad/s
   * @param       kp: 位置增益，0~500
   * @param       kd: 速度阻尼，0~5（位置控制时不能为 0）
   * @param       torque: 前馈扭矩，Nm
   */
  void mit(float pos, float vel, float kp, float kd, float torque);

  /**
   * @brief       位置速度模式（0x100 + ID）
   */
  void posSpeed(float pos, float vel);

  /**
   * @brief       速度模式（0x200 + ID）
   */
  void velocity(float vel);

  /**
   * @brief       解析 8 字节反馈帧
   */
  void update(const uint8_t data[8]);

  void setMitLimits(float p_max, float v_max, float t_max);

  uint8_t id() const { return id_; }
  uint8_t error() const { return error_; }
  float position() const { return position_; }
  float velocity() const { return velocity_; }
  float torque() const { return torque_; }
  float tempMos() const { return temp_mos_; }
  float tempRotor() const { return temp_rotor_; }
  bool online() const;

 private:
  static constexpr float kKpMax = 500.0f;
  static constexpr float kKdMax = 5.0f;
  static constexpr uint16_t kMitMode = 0x000;
  static constexpr uint16_t kPosMode = 0x100;
  static constexpr uint16_t kSpeedMode = 0x200;

  CAN_HandleTypeDef* hcan_ = nullptr;
  uint8_t id_ = 0;

  float p_max_ = 12.5f;
  float v_max_ = 8.0f;
  float t_max_ = 28.0f;

  uint16_t position_raw_ = 0;
  uint16_t velocity_raw_ = 0;
  uint16_t torque_raw_ = 0;

  float position_ = 0.0f;
  float velocity_ = 0.0f;
  float torque_ = 0.0f;
  float temp_mos_ = 0.0f;
  float temp_rotor_ = 0.0f;
  uint8_t error_ = 0;

  uint32_t last_rx_tick_ = 0;
  bool has_rx_ = false;
};

#endif  // __DM4340_DRV_H__
