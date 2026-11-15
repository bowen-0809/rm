/**
 *******************************************************************************
 * @file      : tick.hpp
 * @brief     : 系统滴答定时器驱动头文件
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_TICK_TICK_HPP_
#define HW_COMPONENTS_BSP_TICK_TICK_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace hello_world
{
namespace tick
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       返回当前时间
 * @retval       当前时间，单位：us
 * @note        None
 */
uint32_t GetTickUs(void);

/**
 * @brief       返回当前时间
 * @retval       当前时间，单位：ms
 * @note        None
 */
uint32_t GetTickMs(void);

/**
 * @brief       返回当前时间
 * @retval       当前时间，单位：s
 * @note        None
 */
uint32_t GetTickS(void);

/**
 * @brief       返回当前时间
 * @retval       当前时间，单位：us
 * @note        None
 */
void DelayUs(uint32_t us);
}  // namespace tick
}  // namespace hello_world

#endif /* HW_COMPONENTS_BSP_TICK_TICK_HPP_ */
