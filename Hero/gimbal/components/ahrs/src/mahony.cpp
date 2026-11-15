/**
 *******************************************************************************
 * @file      : mahony.cpp
 * @brief     : Mahony 算法实现
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-12      Caikunzhen      1. 完成编写（未测试）
 *  V1.0.0      2023-12-15      Caikunzhen      1. 完成测试
 *  V1.1.0      2004-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention : 具体算法可查看：https://zhuanlan.zhihu.com/p/342703388
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "mahony.hpp"

#include <cstring>

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

/**
 * @brief       计算两向量的叉乘
 * @param        vec1: 向量1
 * @param        vec2: 向量2
 * @param        vec_out: 叉乘结果
 * @retval       None
 * @note        None
 */
static inline void cross(
    const float vec1[3], const float vec2[3], float vec_out[3]);
/* Exported function definitions ---------------------------------------------*/

Mahony::Mahony(float samp_freq, float kp, float ki)
    : Ahrs(), samp_freq_(samp_freq), dbl_kp_(2 * kp), dbl_ki_(2 * ki)
{

}

Mahony::Mahony(const float quat_init[4], float samp_freq, float kp, float ki)
    : Ahrs(quat_init), samp_freq_(samp_freq), dbl_kp_(2 * kp), dbl_ki_(2 * ki)
{

}

void Mahony::update(const float acc_data[3], const float gyro_data[3])
{
  float norm_scale = 0, err_2[3] = {0, 0, 0};
  if (!(acc_data[0] == 0 && acc_data[1] == 0 && acc_data[2] == 0)) {
    float v_bar[3], v_hat_2[3];
    norm_scale = invSqrt(acc_data[0] * acc_data[0] + acc_data[1] * acc_data[1] +
                         acc_data[2] * acc_data[2]);
    arm_scale_f32(acc_data, norm_scale, v_bar, 3);

    v_hat_2[0] = quat_[1] * quat_[3] - quat_[0] * quat_[2];
    v_hat_2[1] = quat_[2] * quat_[3] + quat_[0] * quat_[1];
    v_hat_2[2] = 0.5f - quat_[1] * quat_[1] - quat_[2] * quat_[2];

    cross(v_bar, v_hat_2, err_2);
  }

  float w_x_dt_2[3] = {0}, gyro_err = 0;
  for (uint8_t i = 0; i < 3; i++) {
    i_out_[i] += dbl_ki_ * err_2[i] / samp_freq_;
    gyro_err = dbl_kp_ * err_2[i] + i_out_[i];
    w_x_dt_2[i] = (gyro_data[i] + gyro_err) * 0.5f / samp_freq_;
  }

  /* 更新四元数 */
  float delta_quat[4] = {0},
        w_x_dt_2_ex[4] = {0, w_x_dt_2[0], w_x_dt_2[1], w_x_dt_2[2]};
  arm_quaternion_product_single_f32(quat_, w_x_dt_2_ex, delta_quat);
  float quat[4];
  arm_add_f32(quat_, delta_quat, quat, 4);

  /* 归一化四元数 */
  norm_scale = invSqrt(quat[0] * quat[0] + quat[1] * quat[1] +
                       quat[2] * quat[2] + quat[3] * quat[3]);
  arm_scale_f32(quat, norm_scale, quat_, 4);
}

void Mahony::init(float samp_freq, float kp, float ki)
{
  samp_freq_ = samp_freq;
  dbl_kp_ = 2 * kp;
  dbl_ki_ = 2 * ki;

  memset(quat_, 0, sizeof(float) * 4);
  quat_[0] = 1.0f;

  memset(i_out_, 0, sizeof(float) * 3);
}

void Mahony::init(const float quat_init[4], float samp_freq, float kp, float ki)
{
  samp_freq_ = samp_freq;
  dbl_kp_ = 2 * kp;
  dbl_ki_ = 2 * ki;

  memcpy(quat_, quat_init, sizeof(float) * 4);

  memset(i_out_, 0, sizeof(float) * 3);
}
/* Private function definitions ----------------------------------------------*/

static inline void cross(
    const float vec1[3], const float vec2[3], float vec_out[3])
{
  vec_out[0] = vec1[1] * vec2[2] - vec2[1] * vec1[2];
  vec_out[1] = vec2[0] * vec1[2] - vec1[0] * vec2[2];
  vec_out[2] = vec1[0] * vec2[1] - vec2[0] * vec1[1];
}

}  // namespace ahrs
}  // namespace hello_world
