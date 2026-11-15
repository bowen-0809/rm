/**
 *******************************************************************************
 * @file      : mahony.hpp
 * @brief     : Mahony 算法实现
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-12      Caikunzhen      1. 完成编写（未测试）
 *  V1.0.0      2023-12-15      Caikunzhen      1. 完成测试
 *  V1.1.0      2004-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_AHRS_MAHONY_HPP_
#define HW_COMPONENTS_ALGORITHMS_AHRS_MAHONY_HPP_

/* Includes ------------------------------------------------------------------*/
#include "ahrs_base.hpp"

namespace hello_world
{
namespace ahrs
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Mahony : public Ahrs
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Mahony(void) = default;
  /**
   * @brief       Mahony 初始化
   * @param        samp_freq: 采样频率，单位：Hz
   * @param        kp: 比例系数（>=0）
   * @param        ki: 积分系数（>=0）
   * @retval       None
   * @note        None
   */
  Mahony(float samp_freq, float kp, float ki);
  /**
   * @brief       Mahony 初始化
   * @param        quat_init: 初始化单位四元数，[qw qx qy qz]
   * @param        samp_freq: 采样频率，单位：Hz
   * @param        kp: 比例系数（>=0）
   * @param        ki: 积分系数（>=0）
   * @retval       None
   * @note        None
   */
  Mahony(const float quat_init[4], float samp_freq, float kp, float ki);
  Mahony(const Mahony&) = default;
  Mahony& operator=(const Mahony&) = default;
  Mahony(Mahony&&) = default;
  Mahony& operator=(Mahony&&) = default;

  virtual ~Mahony(void) = default;

  /* 重载方法 */

  /**
   * @brief       根据反馈数据进行姿态更新
   * @param        acc_data: 加速度计三轴数据，[ax ay az]，无单位要求，全为 0 时不使
   *               用加速度计数据更新姿态
   * @param        gyro_data: 陀螺仪三轴数据，[wx wy wz]，单位：rad/s
   * @retval       None
   * @note        加速度计三轴数据需包含重力加速度项
   */
  virtual void update(
      const float acc_data[3], const float gyro_data[3]) override;

  /* 配置方法 */

  /**
   * @brief       Mahony 初始化，使用默认构造函数后请务必调用此函数
   * @param        samp_freq: 采样频率，单位：Hz
   * @param        kp: 比例系数（>=0）
   * @param        ki: 积分系数（>=0）
   * @retval       None
   * @note        None
   */
  void init(float samp_freq, float kp, float ki);

  /**
   * @brief       Mahony 初始化，使用默认构造函数后请务必调用此函数
   * @param        quat_init: 初始化单位四元数，[qw qx qy qz]
   * @param        samp_freq: 采样频率，单位：Hz
   * @param        kp: 比例系数（>=0）
   * @param        ki: 积分系数（>=0）
   * @retval       None
   * @note        None
   */
  void init(const float quat_init[4], float samp_freq, float kp, float ki);

 private:
  float samp_freq_ = 1000.0f;
  float dbl_kp_ = 0.5f;  ///* 2 * kp
  float dbl_ki_ = 0.0f;  ///* 2 * ki
  float i_out_[3] = {0, 0, 0};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace ahrs
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_AHRS_MAHONY_HPP_ */
