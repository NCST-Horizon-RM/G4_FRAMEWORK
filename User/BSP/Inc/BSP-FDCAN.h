//
// Created by CaoKangqi on 2026/1/5.
//

#ifndef G4_FRAMEWORK_BSP_FDCAN_H
#define G4_FRAMEWORK_BSP_FDCAN_H

#include "fdcan.h"

// CAN接收统计数据结构
typedef struct
{
    uint32_t rx_count;          // 总接收消息数
    uint32_t fifo_full_count;   // FIFO满次数
    uint32_t msg_lost_count;    // 消息丢失次数
    uint32_t error_count;       // 读取错误次数
} CAN_Stats_t;

extern CAN_Stats_t can1_stats;
extern CAN_Stats_t can2_stats;
extern CAN_Stats_t can3_stats;

void CAN_App_Frame_Dispatch(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, uint8_t *data, uint32_t len);

typedef FDCAN_HandleTypeDef hcan_t;
void FDCAN_Config(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo);
uint32_t Bytes_To_DLC(uint32_t len);
uint8_t DLC_To_Bytes(uint32_t dlc);
extern uint8_t FDCAN_Send_Msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len);

#endif //G4_FRAMEWORK_BSP_FDCAN_H