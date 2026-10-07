/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-10-07 06:40:35
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-10-07 09:48:40
 * @FilePath: \hwk1\Tasks\src\tasks.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file tasks.c
 * @brief 电控第一次作业 - 业务代码实现
 *
 * 硬件：STM32F103C6T6A 最小系统板
 *   - 板载LED：PC13（低电平点亮）
 *   - 定时器：TIM2，更新中断 1ms
 *   - 看门狗：IWDG，超时约 2s（LSI≈40kHz，分频64，Reload=1249）
 *
 * 三题切换方式：
 *   第1题 GPIO：        只编译，不喂狗。暂停调试器观察 LED 亮，2s 内看完否则复位。
 *   第2题 定时器+喂狗： 取消 KEEP_DOG_FEED 的注释，tick 每秒 +1000。
 *   第3题 不喂狗：      恢复 KEEP_DOG_FEED 为注释，tick 涨到 ~2000 后归零重复。
 */

#include "tasks.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/*全局变量：Ozone 里监视 tick*/
volatile uint32_t tick = 0;

/*
 * 三题切换：
 *   第2题（喂狗）：取消下一行注释 → 回调里会刷新 IWDG，tick 持续增长。
 *   第3题（不喂狗）：把下一行重新注释掉 → 2s 后复位，tick 回到 0。
 */
//#define KEEP_DOG_FEED

/*
 * 第1题 GPIO 初始化：PC13 输出低电平，板载灯点亮。
 * 在 CubeMX 里已把 PC13 配成 GPIO_Output（推挽，低速），
 * 这里只做电平设置，不在 while(1) 里翻转。
 */
void Tasks_Init(void)
{
    /* 第1题：PC13 写低电平 → 板载 LED 亮 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /* 启动 TIM2 更新中断（第2题开始使用） */
    HAL_TIM_Base_Start_IT(&htim2);
}

/*
 * 定时器更新中断回调 
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        tick++;

#ifdef KEEP_DOG_FEED
        /* 第2题：每 1ms 喂一次狗，芯片不会复位 */
        HAL_IWDG_Refresh(&hiwdg);
#endif
    }
}
