/**
 *******************************************************************************
 * @file      : ahrs_base.hpp
 * @brief     : 姿态解算基类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-12      Caikunzhen      1. 完成编写（未测试）
 *  V1.0.0      2023-12-15      Caikunzhen      1. 完成测试
 *  V1.1.0      2004-07-13      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_AHRS_AHRS_BASE_HPP_
#define HW_COMPONENTS_ALGORITHMS_AHRS_AHRS_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstring>

namespace hello_world
{
namespace ahrs
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Ahrs
{
 public:
  Ahrs(void) = default;
  /**
   * @brief       AHRS 初始化
   * @param        quat_init: 初始化单位四元数，[qw qx qy qz]
   * @retval       None
   * @note        None
   */
  Ahrs(const float quat_init[4]);
  Ahrs(const Ahrs&) = default;
  Ahrs& operator=(const Ahrs&) = default;
  Ahrs(Ahrs&&) = default;
  Ahrs& operator=(Ahrs&&) = default;

  virtual ~Ahrs(void) = default;

  /* 功能性方法 */

  /**
   * @brief       根据反馈数据进行姿态更新
   * @param        acc_data: 加速度计三轴数据，[ax ay az]，无单位要求
   * @param        gyro_data: 陀螺仪三轴数据，[wx wy wz]，单位：rad/s
   * @retval       None
   * @note        加速度计三轴数据需包含重力加速度项
   */
  virtual void update(const float acc_data[3], const float gyro_data[3]) = 0;

  /* 数据修改与获取 */

  /**
   * @brief       获取当前姿态对应的四元数
   * @param        quat: 当前姿态对应的四元数，[qw qx qy qz]
   * @retval       None
   * @note        姿态为相对起始时的右手系，满足 x 朝前，y 朝左，z 朝上
   */
  void getQuat(float quat[4]) const;

  /**
   * @brief       获取当前姿态对应的欧拉角（Z-Y-X）
   * @param        euler_angle: 当前姿态对应的欧拉角（Z-Y-X），[roll pitch yaw]，单
   *               位：rad
   * @retval       None
   * @note        姿态为相对起始时的右手系，满足 x 朝前，y 朝左，z 朝上
   */
  void getEulerAngle(float euler_angle[3]) const;

 protected:
  /**
   * @brief       快速计算 1/sqrt(x)
   * @param        x: 数（>0）
   * @retval       计算结果
   * @note        x 必须为正数
   */
  float invSqrt(float x) const;

  float quat_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace ahrs
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_AHRS_AHRS_BASE_HPP_ */
