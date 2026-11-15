/**
*******************************************************************************
* @file      ：system.hpp
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
#ifndef __SYSTEM_USER_HPP__
#define __SYSTEM_USER_HPP__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "can.h"
#include "gpio.h"
#include "iwdg.h"
#include <stdint.h>

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

const float kCtrlPeriod = 0.001f;
extern uint32_t tick;

/* Quat2Euler 输出顺序为 [yaw, pitch, roll] */
#define YAW 0
#define PITCH 1
#define ROLL 2

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __SYSTEM_USER_HPP__ */