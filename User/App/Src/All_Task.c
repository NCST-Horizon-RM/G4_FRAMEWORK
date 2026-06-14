//
// Created by CaoKangqi on 2026/1/19.
//
#include "All_Task.h"
#include <stdio.h>

CCM_FUNC void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        if (htim->Instance == TIM4) {
            System_Root(&ROOT_Status, &DBUS, &All_Motor, NULL);
        }
    if (htim->Instance == TIM6) {
        Update_Shoot_Det(current_data.speed_i2, current_data.speed_i3, &g_det);
    }
}
static uint8_t icm_tx_buf[15];//包含寄存器地址和14字节数据，预先填充寄存器地址以优化DMA读取
static uint8_t icm_rx_buf[15];//包含寄存器地址和14字节数据，预先填充寄存器地址以优化DMA读取
static uint8_t icm_raw_cache[14];//DMA读取完成后会先存放在这里，等待主任务处理转换为物理量
static TaskHandle_t xIMUTaskHandle = NULL;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 显式声明并初始化xHigherPriorityTaskWoken
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == ICM_DRDY_PIN) {
        ICM42688_StartRead_IntDMA(icm_tx_buf, icm_rx_buf);
        // 清除MCU EXTI挂起位
        __HAL_GPIO_EXTI_CLEAR_IT(ICM_DRDY_PIN);
        // 触发APP层任务
        if(xIMUTaskHandle != NULL) {
            xTaskNotifyFromISR(xIMUTaskHandle,
                              0,
                              eIncrement,
                              &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == ICM_SPI_HANDLE) {
        ICM_CS_PORT->BSRR = ICM_CS_PIN;
        memcpy(icm_raw_cache, &icm_rx_buf[1], sizeof(icm_raw_cache));
    }
}

static uint32_t INS_DWT_Count = 0; // DWT计数基准
static float imu_period_s = 0.0f;
void IMU_Task(void *argument)
{
    (void)argument;
    xIMUTaskHandle = xTaskGetCurrentTaskHandle();
    ICM42688_Init();
    // 预填充DMA缓冲区：寄存器地址+0xFF占位数据，优化后续DMA读取效率
    icm_tx_buf[0] = (REG_TEMP_DATA1 | 0x80);
    for (int i = 1; i < (int)sizeof(icm_tx_buf); i++) {
        icm_tx_buf[i] = 0xFF;
    }
    INS_DWT_Count = DWT->CYCCNT;
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        imu_period_s = DWT_GetDeltaT(&INS_DWT_Count);
        //ICM42688_Read_Fast(IMU_Data.gyro, IMU_Data.accel,&IMU_Data.temp);
        ICM42688_ResolveRaw(icm_raw_cache, IMU_Data.gyro, IMU_Data.accel,&IMU_Data.temp);
        IMU_Update_Task(imu_period_s);
        DWT_SysTimeUpdate();
    }
}

uint8_t flash_id[3] = {0};
void Motor_Task(void *argument)
{
    (void)argument;
    W25N01GV_Init();
    if (Chassis_Control_Init(&All_Motor) != DF_READY)
    {
        Error_Handler();
    }
    for(;;)
    {
        Chassis_Control_Task(&All_Motor);
        //W25N01GV_ReadID(flash_id);// ID 应该是 EF AA 21
        VOFA_justfloat(
            IMU_Data.pitch,
            hfdcan2.ErrorCode,
            All_Motor.DJI_6020_Steer[0].PID_S.Output,
            All_Motor.DJI_6020_Steer[1].DATA.Speed_now,
            All_Motor.DJI_6020_Steer[1].PID_S.Output,
            All_Motor.DJI_6020_Steer[2].DATA.Speed_now,
            All_Motor.DJI_6020_Steer[2].PID_S.Output,
            All_Motor.DM4310_Yaw.PID_P.Ref,
            IMU_Data.YawTotalAngle,All_Power.P_Chassis.power);
        osDelay(1);
    }
}

float current_temp = 0.0f;
uint32_t last_tick = 0;
void Test_Task(void *argument)
{
    (void)argument;
    Test_Init();
    ui_config_t ui_cfg = {
        .max_cap = 100.0f,
        .max_wr = 60.0f,
        .max_bullet = 30.0f,
        .max_shoot = 10.0f
    };
    UI_Init(&h_ui, &ui_cfg);
    Shoot_Control_Init();
    for(;;)
    {

        if (User_data.hurt_data.HP_deduction_reason != 1)
        {
            UI_UpdateHurtDirection(&h_ui, User_data.hurt_data.armor_id, 0.0f);
        }

        h_ui.yaw = IMU_Data.yaw;
        h_ui.pitch = IMU_Data.pitch;
        h_ui.cap = cap.get.Cap_Capacity;
        UI_OnLoop(&h_ui);
        UI_SendUartCmd(&h_ui);
        //Ctrl_Test_Task();
        Motor_Calibration_Task();

        //Test_Tx();
        /*if (HAL_GetTick() - last_tick >= 250) {
            last_tick = HAL_GetTick();
            current_temp = Thermocouple_Read_Temp(&huart2);
        }*/
        //VOFA_justfloat(current_temp,0,0,0,0,0,0,0,0,0);
        osDelay(1);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    uint8_t *pData = huart->pRxBuffPtr;
    if (huart->Instance == USART3){
        if (Size == 18){
            DBUS_Resolved(DBUS_RX_DATA, &DBUS);
            __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
        }
    }
    if (huart->Instance == UART5){
        if (Size == 21){
            VT13_Resolved(VT13_RX_DATA, &VT13);
            __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
        }
    }
    if (huart->Instance == USART1){
        uint8_t *next_buf = (pData == Referee_Rx_Buf[0]) ? Referee_Rx_Buf[1] : Referee_Rx_Buf[0];
        HAL_UARTEx_ReceiveToIdle_DMA(huart, next_buf, REFEREE_RXFRAME_LENGTH);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断
        Referee_System_Frame_Update(pData,Size);
    }
    if (huart->Instance == USART2) {
        if (Size >= sizeof(SpeedData_t))
        {
            for (uint32_t i = 0; i <= Size - sizeof(SpeedData_t); i++)
            {
                if (rx_buffer[i] == 0xAA && rx_buffer[i+1] == 0xBB)
                {
                    SpeedData_t *pPkg = (SpeedData_t *)&rx_buffer[i];
                    current_data.speed_i2 = pPkg->speed_i2;
                    current_data.speed_i3 = pPkg->speed_i3;
                    break;
                }
            }
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart){
    if (huart->Instance == USART3){
        UART_ReceiveToIdle_DMA(&huart3,DBUS_RX_DATA,18);
    }
    if (huart->Instance == UART5){
        UART_ReceiveToIdle_DMA(&huart5, VT13_RX_DATA, 21);
    }
    if (huart->Instance == USART1){
        UART_ReceiveToIdle_DMA(&huart1, Referee_Rx_Buf[0], REFEREE_RXFRAME_LENGTH);
    }
    if (huart->Instance == USART2) {
        UART_ReceiveToIdle_DMA(&huart2, rx_buffer, 64);
    }
}


static const CAN_Rx_Route_t CAN_Rx_Config_Table[] = {
    /* ----- FDCAN1 ----- */
    {FDCAN1, 0x201, &All_Motor.DJI_3508_Chassis[0], DJI_Motor_Resolve},
    {FDCAN1, 0x202, &All_Motor.DJI_3508_Chassis[1], DJI_Motor_Resolve},
    {FDCAN1, 0x203, &All_Motor.DJI_3508_Chassis[2], DJI_Motor_Resolve},
    {FDCAN1, 0x204, &All_Motor.DJI_3508_Chassis[3], DJI_Motor_Resolve},
    {FDCAN1, 0x206, &All_Motor.DJI_6020_Pitch,      DJI_Motor_Resolve},

    /* ----- FDCAN2 ----- */
    {FDCAN2, 0x301, &All_Motor.DM4310_Feed,         DM_1to4_Resolve},
    {FDCAN2, 0x202, &All_Motor.DJI_3508_Pull,       DJI_Motor_Resolve},
    {FDCAN2, 0x203, &All_Motor.DJI_3508_Yaw,        DJI_Motor_Resolve},
    {FDCAN2, 0x288, &cap,                    Power_Cap_Rx},// 超级电容

    /* ----- FDCAN3 ----- */
    {FDCAN3, 0x205, &All_Motor.DJI_6020_Steer[0],   DJI_Motor_Resolve},
    {FDCAN3, 0x206, &All_Motor.DJI_6020_Steer[1],   DJI_Motor_Resolve},
    {FDCAN3, 0x207, &All_Motor.DJI_6020_Steer[2],   DJI_Motor_Resolve},
    {FDCAN3, 0x208, &All_Motor.DJI_6020_Steer[3],   DJI_Motor_Resolve},
};

CCM_FUNC void CAN_App_Frame_Dispatch(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, uint8_t *data, uint32_t len)
{
    (void)len; // 对齐接口后，长度信息在包装函数内处理或忽略
    size_t table_size = sizeof(CAN_Rx_Config_Table) / sizeof(CAN_Rx_Route_t);
    for (size_t i = 0; i < table_size; i++)
    {
        if ((hfdcan->Instance == CAN_Rx_Config_Table[i].instance) &&
            (identifier == CAN_Rx_Config_Table[i].id))
        {
            if (CAN_Rx_Config_Table[i].resolve != NULL)
            {
                CAN_Rx_Config_Table[i].resolve(CAN_Rx_Config_Table[i].device_ptr, data);
            }
            return;
        }
    }
}