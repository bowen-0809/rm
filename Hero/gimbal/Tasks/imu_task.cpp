/**
*******************************************************************************
 * @file      :imu_task.cpp
* @brief     : IMU数据处理任务
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      2024-09-08      Jinletian       1. Create this file.
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "imu_task.hpp"

#include "imu.hpp"
#include "mahony.hpp"
#include "spi.h"

#include <math.h>
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hello_world::imu::BMI088 *bmi088_ptr = nullptr;
hello_world::ahrs::Mahony *mahony_ptr = nullptr;
float acc_data[3], gyro_data[3], temp;

/* 固件版本标记 */
volatile uint8_t dbg_imu_map_version = 16u;

/* IMU 原始数据调试：用于标定安装矩阵 */
volatile float dbg_gyro_x = 0.0f;
volatile float dbg_gyro_y = 0.0f;
volatile float dbg_gyro_z = 0.0f;
volatile float dbg_acc_x = 0.0f;
volatile float dbg_acc_y = 0.0f;
volatile float dbg_acc_z = 0.0f;

/* 上电静止陀螺零偏校准 */
static constexpr uint32_t kGyroCalibSamples = 1500;  // 1.5 s @ 1 kHz
static uint32_t gyro_calib_count = 0;
static float gyro_calib_sum[3] = {0.0f, 0.0f, 0.0f};
static float gyro_calib_bias[3] = {0.0f, 0.0f, 0.0f};
static bool gyro_calibrated = false;
volatile uint8_t dbg_imu_calibrated = 0u;
volatile uint32_t dbg_imu_calib_count = 0u;
float quat[4];
/* External variables --------------------------------------------------------*/

float euler_angles[3];
/* Private function prototypes -----------------------------------------------*/

static void Quat2Euler(float *quat, float *euler);

/**
 * @brief       IMU 初始化
 * @retval       None
 * @note        使用 IMU 前必须调用此函数进行初始化，并且注意该函数会阻塞直到
 * IMU 初始化完成
 */
void ImuInit() {
  hello_world::imu::BMI088HWConfig default_params = {
      .hspi = &hspi1,
      .acc_cs_port = GPIOA,
      .acc_cs_pin = GPIO_PIN_4,
      .gyro_cs_port = GPIOB,
      .gyro_cs_pin = GPIO_PIN_0,
  };
  /*
   * version 13 的数据表明：静止时 acc_x 为 -9.8，重力被映射到了机体 X 轴
   * 说明那次矩阵把姿态转成了侧躺，不能用
   *
   * 原始数据（version 10）中静止重力在传感器 +Z 轴
   *   物理 Yaw   -> 传感器 Z 轴
   *   物理 Pitch -> 传感器 X 轴
   *   物理 Roll  -> 传感器 Y 轴
   * 正确的右手矩阵是
   *   Roll  = -传感器 Y 轴
   *   Pitch =  传感器 X 轴
   *   Yaw   =  传感器 Z 轴
   * 这样静止时 acc_z 才会是 +9.8 左右
   * Yaw/Pitch 的显示符号在 ImuUpdate() 后面单独取反
   */
  float rot_mat_flatten[9] = {0.0f, -1.0f, 0.0f,
                              1.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, 1.0f};

  /**< Mahony 滤波 kp 参数 */
  float mahony_kp = 0.5f; ///< Mahony 滤波 kp 参数
  float mahony_ki = 0.0f; ///< Mahony 滤波 ki 参数

  /**< Mahony 滤波器初始四元数，[w, x, y, z] */
  float mahony_init_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float sample_freq = 1000.0f;
  bmi088_ptr = new hello_world::imu::BMI088(default_params, rot_mat_flatten);
  bmi088_ptr->imuInit();
  mahony_ptr = new hello_world::ahrs::Mahony(mahony_init_quat, sample_freq,
                                             mahony_kp, mahony_ki);
}

/**
 * @brief       IMU 更新
 * @retval       None
 * @note
 */
void ImuUpdate() {
  dbg_imu_map_version = 16u;
  bmi088_ptr->getData(acc_data, gyro_data, &temp);

  /*
   * 上电静止陀螺零偏校准：
   * 前 1.5 s 内保持云台静止，累计三轴陀螺均值；
   * 如果检测到明显运动，则重新开始累计。
   */
  if (!gyro_calibrated)
  {
    const float gyro_abs = sqrtf(gyro_data[0] * gyro_data[0] +
                                 gyro_data[1] * gyro_data[1] +
                                 gyro_data[2] * gyro_data[2]);
    if (gyro_abs > 0.5f)
    {
      gyro_calib_count = 0;
      gyro_calib_sum[0] = 0.0f;
      gyro_calib_sum[1] = 0.0f;
      gyro_calib_sum[2] = 0.0f;
    }
    else
    {
      gyro_calib_sum[0] += gyro_data[0];
      gyro_calib_sum[1] += gyro_data[1];
      gyro_calib_sum[2] += gyro_data[2];
      gyro_calib_count++;
      if (gyro_calib_count >= kGyroCalibSamples)
      {
        gyro_calib_bias[0] =
            gyro_calib_sum[0] / static_cast<float>(gyro_calib_count);
        gyro_calib_bias[1] =
            gyro_calib_sum[1] / static_cast<float>(gyro_calib_count);
        gyro_calib_bias[2] =
            gyro_calib_sum[2] / static_cast<float>(gyro_calib_count);
        gyro_calibrated = true;
      }
    }

    dbg_acc_x = acc_data[0];
    dbg_acc_y = acc_data[1];
    dbg_acc_z = acc_data[2];
    dbg_gyro_x = gyro_data[0];
    dbg_gyro_y = gyro_data[1];
    dbg_gyro_z = gyro_data[2];
    dbg_imu_calibrated = 0u;
    dbg_imu_calib_count = gyro_calib_count;
    return;
  }

  /* 校准时序完成后，从陀螺数据中减去零偏 */
  gyro_data[0] -= gyro_calib_bias[0];
  gyro_data[1] -= gyro_calib_bias[1];
  gyro_data[2] -= gyro_calib_bias[2];

  dbg_acc_x = acc_data[0];
  dbg_acc_y = acc_data[1];
  dbg_acc_z = acc_data[2];
  dbg_gyro_x = gyro_data[0];
  dbg_gyro_y = gyro_data[1];
  dbg_gyro_z = gyro_data[2];
  dbg_imu_calibrated = 1u;
  dbg_imu_calib_count = kGyroCalibSamples;

  mahony_ptr->update(acc_data, gyro_data);
  mahony_ptr->getQuat(quat);
  Quat2Euler(quat, euler_angles);

  euler_angles[1] = -euler_angles[1];  // Pitch 保持当前符号
}

/**
 * @brief       四元数转欧拉角

 * @param        q: 四元数，[w, x, y, z]
 * @param        euler: 欧拉角，[yaw, pitch, roll]，单位：rad
 * @retval       None
 * @note        涉及四元数转欧拉角的部分请调用该函数
 */
static void Quat2Euler(float *quat, float *euler) {
  arm_atan2_f32(quat[0] * quat[1] + quat[2] * quat[3],
                quat[0] * quat[0] + quat[3] * quat[3] - 0.5f, euler + 2);
  float sinp = 2 * (quat[0] * quat[2] - quat[3] * quat[1]);
  if (sinp > 1.0f)
    sinp = 1.0;
  else if (sinp < -1.0f)
    sinp = -1.0f;
  euler[1] = asinf(sinp);
  arm_atan2_f32(quat[0] * quat[3] + quat[1] * quat[2],
                quat[0] * quat[0] + quat[1] * quat[1] - 0.5f, euler + 0);
}