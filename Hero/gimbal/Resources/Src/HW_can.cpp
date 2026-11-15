/**
 *******************************************************************************
 * @file      :HW_can.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"
#include "stdint.h"

#include <cstring>

#include "dm4310_drv.hpp"
#include "dm4340_drv.hpp"
#include "gimbal_cmd.hpp"
#include "iwdg.h"

extern Joint_Motor_t yaw_motor;
extern DM4340 pitch_motor;
extern volatile GimbalCmd gimbal_cmd;
extern volatile bool gimbal_cmd_valid;
extern volatile uint32_t gimbal_cmd_tick;
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1, rx_header2;
static uint8_t can1_rx_data[8], can2_rx_data[8];
uint32_t pTxMailbox;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief
 * @param        *hcan:
 * @retval       None
 * @note        None
 */
void CanFilter_Init(CAN_HandleTypeDef *hcan) {
  CAN_FilterTypeDef canfilter;

  canfilter.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilter.FilterScale = CAN_FILTERSCALE_16BIT;

  canfilter.FilterActivation = ENABLE;
  canfilter.SlaveStartFilterBank = 14;
  if (hcan == &hcan1) {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;

    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;
    canfilter.FilterBank = 0;
    canfilter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK) {
      Error_Handler();
    }
  } else if (hcan == &hcan2) {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO1;
    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;
    canfilter.FilterActivation = ENABLE;
    canfilter.FilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK) {
      Error_Handler();
    }
  }
}

uint32_t can_rec_times = 0;
uint32_t can_success_times = 0;
uint32_t can_receive_data = 0;

/* 云台 Pitch 反馈诊断：CAN2 任意帧计数 / 0x19 反馈计数 */
volatile uint32_t dbg_can2_rx_count = 0;
volatile uint32_t dbg_pitch_rx_count = 0;

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, can1_rx_data) ==
        HAL_OK) {
      if (rx_header1.StdId == 0x13) {
        /* Yaw DM4310 反馈 */
        dm4310_fbdata(&yaw_motor, can1_rx_data, 8);
      } else if (rx_header1.StdId == kGimbalCmdId) {
        /* 底盘命令 + 心跳：int16 为大端，和 DM 帧一致 */
        gimbal_cmd.yaw_rate_mrad = static_cast<int16_t>(
            (static_cast<uint16_t>(can1_rx_data[0]) << 8) | can1_rx_data[1]);
        gimbal_cmd.pitch_mrad = static_cast<int16_t>(
            (static_cast<uint16_t>(can1_rx_data[2]) << 8) | can1_rx_data[3]);
        gimbal_cmd.enable = can1_rx_data[4];
        gimbal_cmd.estop = can1_rx_data[5];
        gimbal_cmd.counter = can1_rx_data[6];
        gimbal_cmd.reserved = can1_rx_data[7];
        gimbal_cmd_valid = true;
        gimbal_cmd_tick = tick;
        HAL_IWDG_Refresh(&hiwdg);
      }
    }
  }
  HAL_CAN_ActivateNotification(
      hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2, can2_rx_data) ==
        HAL_OK) {
      dbg_can2_rx_count = dbg_can2_rx_count + 1u;
      if (rx_header2.StdId == 0x19) {
        /* Pitch DM4340 反馈 */
        dbg_pitch_rx_count = dbg_pitch_rx_count + 1u;
        pitch_motor.update(can2_rx_data);
      }
      /* 0x201~0x207 摩擦轮反馈，云台任务暂时不用 */
    }
  }
  HAL_CAN_ActivateNotification(
      hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief   向can总线发送数据，抄官方的
 * @param   hcan为CAN句柄
 * @param	msg为发送数组首地址
 * @param	id为发送报文
 * @param	len为发送数据长度（字节数）
 * @retval  none
 * @note    主控发送都是len=8字节，再加上帧间隔3位，理论上can总线1ms最多传输9帧
 **/
void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len) {
  CAN_TxHeaderTypeDef TxMessageHeader = {0};
  TxMessageHeader.StdId = id;
  TxMessageHeader.IDE = CAN_ID_STD;
  TxMessageHeader.RTR = CAN_RTR_DATA;
  TxMessageHeader.DLC = len;
  if (HAL_CAN_AddTxMessage(hcan, &TxMessageHeader, msg, &pTxMailbox) !=
      HAL_OK) {
  }
}
