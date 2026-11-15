/**
*******************************************************************************
* @file      :imu_task.hpp
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HOMEWORK_TASKS_IMU_TASK_HPP_
#define HOMEWORK_TASKS_IMU_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

extern float euler_angles[3];
/* Exported function prototypes ----------------------------------------------*/

void ImuInit();
void ImuUpdate();
#endif /* HOMEWORK_TASKS_IMU_TASK_HPP_ */