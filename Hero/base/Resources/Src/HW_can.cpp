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

#include "m3508.hpp"

extern hello_world::motor::M3508 wheel[4];

/* Yaw 电机到云台关节的减速比：实测 180° 对应电机约 12.5 rad */
static constexpr float kYawJointGearRatio = 4.0f;
extern volatile float gimbal_yaw;
extern volatile uint8_t gimbal_yaw_valid;
extern volatile uint32_t gimbal_yaw_tick;

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
        /* DM4310 Yaw 反馈：PMAX = 12.5 rad */
        const uint16_t p_int = static_cast<uint16_t>(
            (static_cast<uint16_t>(can1_rx_data[1]) << 8) | can1_rx_data[2]);
        const float motor_yaw =
            static_cast<float>(p_int) * 25.0f / 65535.0f - 12.5f;
        /* 底盘跟随使用云台关节角度，不使用电机角度 */
        gimbal_yaw = motor_yaw / kYawJointGearRatio;
        gimbal_yaw_valid = 1u;
        gimbal_yaw_tick = tick;
      }
    }
  }
  HAL_CAN_ActivateNotification(
      hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {

  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2, can2_rx_data) ==
        HAL_OK) // 获得接收到的数据头和数据
    {
      switch (rx_header2.StdId) {
        case 0x201:  // 左前 ID1
          wheel[0].update(can2_rx_data);
          break;
        case 0x202:  // 左后 ID2
          wheel[1].update(can2_rx_data);
          break;
        case 0x203:  // 右后 ID3
          wheel[2].update(can2_rx_data);
          break;
        case 0x204:  // 右前 ID4
          wheel[3].update(can2_rx_data);
          break;
        default:
          break;
      }
    }
  }
  HAL_CAN_ActivateNotification(
      hcan, CAN_IT_RX_FIFO1_MSG_PENDING); // 再次使能FIFO1接收中断
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

void CAN_SendWheelCurrents(const int16_t current[4]) {
  uint8_t data[8];

  for (int i = 0; i < 4; ++i) {
    const uint16_t raw = static_cast<uint16_t>(current[i]);
    data[2 * i] = static_cast<uint8_t>(raw >> 8);
    data[2 * i + 1] = static_cast<uint8_t>(raw & 0xFF);
  }

  CAN_Send_Msg(&hcan2, data, 0x200, 8);
}

void CAN_SendGimbalCmd(int16_t yaw_rate_mrad, int16_t pitch_mrad,
                       uint8_t enable, uint8_t estop, uint8_t counter,
                       uint8_t flags) {
  uint8_t data[8];

  data[0] = static_cast<uint8_t>(static_cast<uint16_t>(yaw_rate_mrad) >> 8);
  data[1] = static_cast<uint8_t>(static_cast<uint16_t>(yaw_rate_mrad) & 0xFF);
  data[2] = static_cast<uint8_t>(static_cast<uint16_t>(pitch_mrad) >> 8);
  data[3] = static_cast<uint8_t>(static_cast<uint16_t>(pitch_mrad) & 0xFF);
  data[4] = enable;
  data[5] = estop;
  data[6] = counter;
  data[7] = flags;

  CAN_Send_Msg(&hcan1, data, 0x7FE, 8);
}
