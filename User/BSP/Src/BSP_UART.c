//
// Created by CaoKangqi on 2026/6/10.
//
#include "stm32g4xx_hal.h"
#include "BSP_UART.h"

/**
 * @brief   UART DMA 空闲中断接收
 * @param  huart: HAL串口句柄指针
 * @param  pData: 接收缓冲区指针
 * @param  Size:  预期的最大接收字节数
 * @return HAL_StatusTypeDef: HAL_OK 启动成功, HAL_ERROR 配置错误或句柄为空
 */
HAL_StatusTypeDef UART_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
    // 防止传入空指针导致 HardFault
    if (huart == NULL || pData == NULL || Size == 0) {
        return HAL_ERROR;
    }
    // 检查 DMA 句柄是否配置：防止 CubeMX 没开 DMA 导致死机
    if (huart->hdmarx == NULL) {
        return HAL_ERROR;
    }
    // 停止当前可能存在的传输
    HAL_UART_DMAStop(huart);
    // 彻底清除硬件错误标志
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    // 通过 RQR 寄存器请求丢弃当前接收行为
    huart->Instance->RQR |= USART_RQR_RXFRQ;
    // 读出 RDR 缓存
    volatile uint32_t tmp = huart->Instance->RDR;
    (void)tmp;
    // 重新启动 DMA 接收
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size) != HAL_OK) {
        return HAL_ERROR;
    }
    // 关闭 DMA 半传输中断（HAL 库启动后默认会开启，这里必须关闭）
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return HAL_OK;
}

__weak void UART_App_Rx_Callback(UART_HandleTypeDef *huart, uint16_t Size) {
    (void)huart; (void)Size;
}

__weak void UART_App_Error_Callback(UART_HandleTypeDef *huart) {
    (void)huart;
}

/*/**
 * @brief  HAL 库统一接收事件回调（空闲中断/正常满中断都会进这）
 #1#
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    UART_App_Rx_Callback(huart, Size);
}

/**
 * @brief  HAL 库统一错误回调
 #1#
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    UART_App_Error_Callback(huart);
}*/