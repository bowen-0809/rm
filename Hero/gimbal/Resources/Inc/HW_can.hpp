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

#endif
