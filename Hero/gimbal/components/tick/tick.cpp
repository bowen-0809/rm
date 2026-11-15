

/**
 *******************************************************************************
 * @file      : tick.cpp
 * @brief     : 系统滴答定时器驱动源文件
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2023-10-31      ZhouShichan     1. 完成正式版
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成格式修改
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "tick.hpp"
#include "stm32f4xx_hal.h"

namespace hello_world
{
namespace tick
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

uint32_t GetTickUs(void)
{
  return (uint32_t)((HAL_GetTick() * HAL_GetTickFreq()) * 1000 +
                    SysTick->VAL * 1000 / SysTick->LOAD);
}

/**
 * @brief       返回当前时间
 * @retval       当前时间，单位：ms
 * @note        None
 */
uint32_t GetTickMs(void)
{
  return GetTickUs() / 1000;
}

/**
 * @brief       返回当前时间
 * @retval       当前时间，单位：s
 * @note        None
 */
uint32_t GetTickS(void)
{
  return GetTickMs() / 1000;
}

/**
 * @brief       微秒级延时
 * @param        us: 需要延时的时间，单位：us
 * @retval       None
 * @note        None
 */
void DelayUs(uint32_t us)
{
  uint32_t reload = SysTick->LOAD;
  uint32_t ticks = us * (SystemCoreClock / 1e6f);
  uint32_t t_last = SysTick->VAL;

  uint32_t t_now = 0;
  uint32_t t_cnt = 0;

  while (t_cnt < ticks) {
    t_now = SysTick->VAL;
    if (t_now != t_last) {
      if (t_now < t_last) {
        t_cnt += t_last - t_now;
      } else {
        t_cnt += reload - t_now + t_last;
      }
      t_last = t_now;
    }
  }
}

/* Private function definitions -----------------------------------------------*/
}  // namespace tick
}  // namespace hello_world