//
// Created by CaoKangqi on 2026/6/10.
//

#ifndef G4_FRAMEWORK_BSP_UART_H
#define G4_FRAMEWORK_BSP_UART_H

#include "stm32g4xx_hal_uart.h"

HAL_StatusTypeDef UART_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

#endif //G4_FRAMEWORK_BSP_UART_H