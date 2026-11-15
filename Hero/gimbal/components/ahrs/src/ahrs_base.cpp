/**
 *******************************************************************************
 * @file      : ahrs_base.cpp
 * @brief     : 姿态解算基类
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
/* Includes ------------------------------------------------------------------*/
#include "ahrs_base.hpp"

#include "arm_math.h"

namespace hello_world
{
namespace ahrs
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

Ahrs::Ahrs(const float quat_init[4])
{
  memcpy(quat_, quat_init, sizeof(float) * 4);
}

void Ahrs::getQuat(float quat[4]) const
{
  memcpy(quat, quat_, sizeof(float) * 4);
}

void Ahrs::getEulerAngle(float euler_angle[3]) const
{
  arm_atan2_f32(quat_[0] * quat_[1] + quat_[2] * quat_[3],
                quat_[0] * quat_[0] + quat_[3] * quat_[3] - 0.5f,
                euler_angle + 0);
  euler_angle[1] = asinf(-2.0f * (quat_[1] * quat_[3] - quat_[0] * quat_[2]));
  arm_atan2_f32(quat_[0] * quat_[3] + quat_[1] * quat_[2],
                quat_[0] * quat_[0] + quat_[1] * quat_[1] - 0.5f,
                euler_angle + 2);
}

float Ahrs::invSqrt(float x) const
{
  /* 变量检查 */
  if (x < 0) {
    return 0;
  }

  /* http://en.wikipedia.org/wiki/Fast_inverse_square_root */
  float x_2 = 0.5f * x;
  float y = x;
  int32_t i;
  memcpy(&i, &y, sizeof(int32_t));
  i = 0x5f3759df - (i >> 1);
  memcpy(&y, &i, sizeof(float));
  y = y * (1.5f - (x_2 * y * y));

  return y;
}
/* Private function definitions ----------------------------------------------*/
}  // namespace ahrs
}  // namespace hello_world
