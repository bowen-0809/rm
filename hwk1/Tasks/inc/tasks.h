/**
 * @file tasks.h
 * @brief 电控第一次作业 - 业务代码头文件
 *
 * 三题共用一个工程：
 *   第1题 GPIO  - PC13 推挽输出低电平，板载LED点亮
 *   第2题 定时器 - TIM2 更新中断 1ms，tick++ 并喂狗
 *   第3题 看门狗 - IWDG 2s 超时，不喂狗则复位
 */

#ifndef __TASKS_H__
#define __TASKS_H__

#include "main.h"

/* ========== 全局变量（Ozone 监视用） ========== */
extern volatile uint32_t tick;

/* ========== 初始化函数（main.c USER CODE 调用） ========== */
void Tasks_Init(void);

/* ========== 定时器中断回调（全工程仅此一份） ========== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* __TASKS_H__ */
