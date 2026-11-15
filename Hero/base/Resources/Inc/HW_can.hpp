#ifndef _HW_CAN_H_
#define _HW_CAN_H_
/* ------------------------------ Include ------------------------------ */
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "system_user.hpp"

#include "can.h"
/* ------------------------------ Macro Definition
 * ------------------------------ */

/* ------------------------------ Type Definition ------------------------------
 */

/* ------------------------------ Extern Global Variable
 * ------------------------------ */

/* ------------------------------ Function Declaration (used in other .c files)
 * ------------------------------ */

void CanFilter_Init(CAN_HandleTypeDef *hcan);

void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len);

/**
 * @brief       发送 0x200 四轮电流帧
 * @param       current: 4 个轮的电流指令，顺序为 ID1~ID4
 */
void CAN_SendWheelCurrents(const int16_t current[4]);

/**
 * @brief       发送底盘 -> 云台命令/心跳帧（0x7FE，100 Hz）
 */
void CAN_SendGimbalCmd(int16_t yaw_rate_mrad, int16_t pitch_mrad,
                       uint8_t enable, uint8_t estop, uint8_t counter,
                       uint8_t flags);

#endif
