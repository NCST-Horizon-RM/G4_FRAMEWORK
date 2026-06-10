//
// Created by CaoKangqi on 2026/2/13.
//
#include "All_Init.h"

//DBUS
uint8_t DBUS_RX_DATA[18];
DBUS_Typedef DBUS = { 0 };

uint8_t VT13_RX_DATA[21];
VT13_Typedef VT13 = { 0 };

CCM_DATA MOTOR_Typdef All_Motor;
CCM_DATA ROOT_STATUS_Typedef ROOT_Status;
ALL_POWER_RX All_Power = {0};

CONTAL_Typedef contal;

User_Data_T User_data;
uint8_t Referee_Rx_Buf[2][REFEREE_RXFRAME_LENGTH];

uint8_t rx_buffer[64];
SpeedData_t current_data;

UI_t h_ui;

uint32_t stm32_id[3];
void Get_UID(uint32_t *uid) {
    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();
}
void All_Init() {
    DWT_Init(170);
    Get_UID(stm32_id);

    UART_ReceiveToIdle_DMA(&huart3,DBUS_RX_DATA,18);//DBUS串口
    UART_ReceiveToIdle_DMA(&huart5, VT13_RX_DATA, 21);//图传链路串口
    UART_ReceiveToIdle_DMA(&huart1, Referee_Rx_Buf[0], REFEREE_RXFRAME_LENGTH);//裁判系统串口
    UART_ReceiveToIdle_DMA(&huart2, rx_buffer, 64);//上位机串口

    FDCAN_Config(&hfdcan1, FDCAN_RX_FIFO0);
    FDCAN_Config(&hfdcan2, FDCAN_RX_FIFO1);
    FDCAN_Config(&hfdcan3, FDCAN_RX_FIFO0);

    WS2812_Init();
    W25N01GV_Init();
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//IMU加热
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);//蜂鸣器

    HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_2);//PA1扳机
    htim5.Instance->CCR2 = 600;
    htim20.Instance->CCR2 = 100;
    HAL_Delay(500);
    htim20.Instance->CCR2 = 0;
    //HAL_TIM_PWM_Stop(&htim20, TIM_CHANNEL_2);
    Buzzer_Start();
}