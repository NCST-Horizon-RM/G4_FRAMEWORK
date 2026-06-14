//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef G4_FRAMEWORK_ALL_TASK_H
#define G4_FRAMEWORK_ALL_TASK_H

#include "All_Init.h"

typedef void (*CAN_Resolve_Func_t)(void *instance, uint8_t *data);

// 路由表条目结构体
typedef struct {
    FDCAN_GlobalTypeDef *instance; // 对应的硬件总线：FDCAN1 / FDCAN2 / FDCAN3
    uint32_t id;                   // 绑定的 CAN ID
    void *device_ptr;              // 对应的应用层变量指针
    CAN_Resolve_Func_t resolve;    // 对应的中层协议解析函数
} CAN_Rx_Route_t;

void CAN_App_Frame_Dispatch(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, uint8_t *data, uint32_t len);

extern uint16_t adc_dma_buffer[2];
void speed_solve(void);

void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif //G4_FRAMEWORK_ALL_TASK_H