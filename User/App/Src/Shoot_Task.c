//
// Created by CaoKangqi on 2026/5/13.
//
#include "Shoot_Task.h"

#define TOTAL_SLOTS         6.0f
#define FEED_ZERO_OFFSET    1325.0f
#define COUNTS_PER_SHOT     (8192.0f / TOTAL_SLOTS)

DM4310_Feeder_t g_feed_motor = {0};
Calib_State_t   g_calib_state = CALIB_START;
Pull_State_t    g_pull_state = PULL_STATE_NORMAL;

float g_zero_offset_angle = 0.0f;
float g_mid_offset_angle  = 0.0f;
uint16_t g_pull_delay_counter = 0;
uint8_t  last_v = GPIO_PIN_SET;

void Shoot_Control_Init() {
    float PID_P_FEED[3] = {1.0f, 0.0f, 0.0f};
    float PID_S_FEED[3] = {0.4f, 0.0f, 0.0f};
    float PID_P_YAW[3]  = {1.6f, 0.0f, 0.0f};
    float PID_S_YAW[3]  = {8.0f, 0.01f, 0.0f};
    float PID_P_PULL[3] = {1.0f, 0.0f, 0.0f};
    float PID_S_PULL[3] = {7.0f, 0.0f, 0.0f};

    uint8_t mode = Integral_Limit | ErrorHandle;

    // Feed (DM4310)
    PID_Init(&All_Motor.DM4310_Feed.PID_P, 80, 30, PID_P_FEED, 0, 0, 0, 0, 0, mode);
    PID_Init(&All_Motor.DM4310_Feed.PID_S, 15, 10, PID_S_FEED, 0, 0, 0, 0, 0, mode);

    // Yaw (3508)
    PID_Init(&All_Motor.DJI_3508_Yaw.PID_P, 1000, 150, PID_P_YAW, 0, 0, 0, 0, 0, mode);
    PID_Init(&All_Motor.DJI_3508_Yaw.PID_S, 16384, 2000, PID_S_YAW, 0, 0, 0, 0, 0, mode);

    // Pull (3508)
    if (g_pull_state == PULL_STATE_RESET) {
        PID_Init(&All_Motor.DJI_3508_Pull.PID_P, 2500, 100, PID_P_PULL, 0, 0, 0, 0, 0, mode);
    }
    else {
        PID_Init(&All_Motor.DJI_3508_Pull.PID_P, 4000, 100, PID_P_PULL, 0, 0, 0, 0, 0, mode);
    }
    PID_Init(&All_Motor.DJI_3508_Pull.PID_S, 16384, 2000, PID_S_PULL, 0, 0, 0, 0, 0, mode);
}

void Ctrl_Shoot_Task() {
    if (All_Motor.DM4310_Feed.DATA.Angle_Infinite == 0.0f && All_Motor.DM4310_Feed.DATA.Speed_now == 0.0f) {
        return;
    }

    float current_pulse = All_Motor.DM4310_Feed.DATA.Angle_Infinite;

    if (!g_feed_motor.is_init) {
        g_feed_motor.target_pos_cnt = (int32_t)floorf((current_pulse - FEED_ZERO_OFFSET) / COUNTS_PER_SHOT);
        g_feed_motor.smooth_ref = current_pulse;
        g_feed_motor.is_init = true;
    }

    float final_target = FEED_ZERO_OFFSET + ((float)g_feed_motor.target_pos_cnt * COUNTS_PER_SHOT);

    if (g_feed_motor.smooth_ref > final_target) {
        g_feed_motor.smooth_ref -= 5.0f;
        if (g_feed_motor.smooth_ref < final_target) g_feed_motor.smooth_ref = final_target;
    } else {
        g_feed_motor.smooth_ref = final_target;
    }

    PID_Calculate(&All_Motor.DM4310_Feed.PID_P, current_pulse, g_feed_motor.smooth_ref);
    PID_Calculate(&All_Motor.DM4310_Feed.PID_S, All_Motor.DM4310_Feed.DATA.Speed_now, All_Motor.DM4310_Feed.PID_P.Output);
    DM_Motor_Send(&hfdcan2, 0x3FE, -All_Motor.DM4310_Feed.PID_S.Output, 0, 0, 0);

    All_Motor.DJI_3508_Yaw.PID_P.Ref -= DBUS.Remote.CH3 * 0.09f;
    All_Motor.DJI_3508_Yaw.PID_P.Ref = MATH_Limit_float(All_Motor.DJI_3508_Yaw.PID_P.Ref,
                                                        g_mid_offset_angle - 72550.0f,
                                                        g_mid_offset_angle + 72550.0f);
    PID_Calculate(&All_Motor.DJI_3508_Yaw.PID_P, All_Motor.DJI_3508_Yaw.DATA.Angle_Infinite, All_Motor.DJI_3508_Yaw.PID_P.Ref);
    PID_Calculate(&All_Motor.DJI_3508_Yaw.PID_S, All_Motor.DJI_3508_Yaw.DATA.Speed_now, All_Motor.DJI_3508_Yaw.PID_P.Output);

    GPIO_PinState current_switch_v = HAL_GPIO_ReadPin(Switch_GPIO_Port, Switch_Pin);

    switch (g_pull_state) {
        case PULL_STATE_NORMAL:
            htim5.Instance->CCR2 = 1200;
            All_Motor.DJI_3508_Pull.PID_P.Ref += 200.0f;
            if (current_switch_v == GPIO_PIN_RESET && last_v == GPIO_PIN_SET) {
                htim5.Instance->CCR2 = 600;
                g_pull_delay_counter = 0;
                g_pull_state = PULL_STATE_TRIGGERED;
            }
            break;

        case PULL_STATE_TRIGGERED:
            All_Motor.DJI_3508_Pull.PID_P.Ref = All_Motor.DJI_3508_Pull.DATA.Angle_Infinite;
            if (++g_pull_delay_counter >= 580) {
                All_Motor.DJI_3508_Pull.PID_P.Ref -= 1100000.0f;
                g_feed_motor.target_pos_cnt -= 1;
                g_pull_state = PULL_STATE_RESET;
            }
            break;

        case PULL_STATE_RESET:
            if (MATH_ABS_float(All_Motor.DJI_3508_Pull.DATA.Angle_Infinite - All_Motor.DJI_3508_Pull.PID_P.Ref) < 1000.0f
                && g_feed_motor.smooth_ref == final_target) {
                All_Motor.DJI_3508_Pull.PID_S.Iout = 0.0f;
                g_pull_state = PULL_STATE_STOPPED;
            }
            break;

        case PULL_STATE_STOPPED:
             static uint8_t last_s1 = 0;
            if (DBUS.Remote.S1 == 1 && last_s1 == 3) {
                htim5.Instance->CCR2 = 1200;
                g_pull_state = PULL_STATE_NORMAL;
            }
            last_s1 = DBUS.Remote.S1;
            break;
    }
    last_v = current_switch_v;

    float pull_output = 0.0f;
    if (g_pull_state != PULL_STATE_STOPPED) {
        PID_Calculate(&All_Motor.DJI_3508_Pull.PID_P, All_Motor.DJI_3508_Pull.DATA.Angle_Infinite, All_Motor.DJI_3508_Pull.PID_P.Ref);
        PID_Calculate(&All_Motor.DJI_3508_Pull.PID_S, All_Motor.DJI_3508_Pull.DATA.Speed_now, All_Motor.DJI_3508_Pull.PID_P.Output);
        pull_output = All_Motor.DJI_3508_Pull.PID_S.Output;
    }

    DJI_Motor_Send(&hfdcan2, 0x200, 0, pull_output, All_Motor.DJI_3508_Yaw.PID_S.Output, 0);
}

uint8_t Check_Motor_Reached_Limit(DJI_MOTOR_Typedef* motor, float target_speed, float stuck_current, uint16_t confirm_time) {
    static uint16_t limit_check_counter = 0;
    static float Last_Angle_Infinite = 0;

    float pos_delta = MATH_ABS_float(motor->DATA.Angle_Infinite - Last_Angle_Infinite);
    float current   = MATH_ABS_float(motor->DATA.current);
    float speed     = MATH_ABS_float(motor->DATA.Speed_now);

    if (pos_delta < 10.0f && current > stuck_current && speed < MATH_ABS_float(target_speed) * 0.2f) {
        if (++limit_check_counter >= confirm_time) {
            limit_check_counter = 0;
            return 1;
        }
    } else {
        limit_check_counter = 0;
    }
    Last_Angle_Infinite = motor->DATA.Angle_Infinite;
    return 0;
}

void Motor_Calibration_Task() {
    switch (g_calib_state) {
        case CALIB_START:
            //g_calib_state = CALIB_NORMAL;
            g_calib_state = CALIB_MOVING;
            break;

        case CALIB_MOVING:
            PID_Calculate(&All_Motor.DJI_3508_Yaw.PID_S, All_Motor.DJI_3508_Yaw.DATA.Speed_now, 500.0f);
            DJI_Motor_Send(&hfdcan2, 0x200, 0, 0, All_Motor.DJI_3508_Yaw.PID_S.Output, 0);

            if (Check_Motor_Reached_Limit(&All_Motor.DJI_3508_Yaw, 500.0f, 3000.0f, 80)) {
                g_calib_state = CALIB_DONE;
            }
            break;

        case CALIB_DONE:
            DJI_Motor_Send(&hfdcan2, 0x200, 0, 0, 0, 0); // 停转卸力
            All_Motor.DJI_3508_Yaw.PID_S.Iout = 0.0f;

            g_zero_offset_angle = All_Motor.DJI_3508_Yaw.DATA.Angle_Infinite;
            g_mid_offset_angle = g_zero_offset_angle - 72550.0f;

            // 给 Yaw 一个安全的初始 Ref，无缝衔接后续闭环
            All_Motor.DJI_3508_Yaw.PID_P.Ref = g_mid_offset_angle;

            g_calib_state = CALIB_NORMAL;
            break;

        case CALIB_NORMAL:
            Ctrl_Shoot_Task();
            break;
    }
}

Feeder_t g_feeder = {0};

#define COUNTS_PER_SHOT 36864.0f*2.5f*8/9

/*
void Shoot_Control_Init() {
    float PID_P_SHOOT[3] = {0.23f,0.0f,0.0f};
    float PID_S_SHOOT[3] = {15.0f,0.01f,0.0f};

    PID_Init(&All_Motor.DJI_2006_bo.PID_P,20000,4000,
        PID_P_SHOOT,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
    PID_Init(&All_Motor.DJI_2006_bo.PID_S,10000,2000,
        PID_S_SHOOT,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
}

void Ctrl_Shoot_Task() {
    if (DBUS.Remote.S2 != 1 && DBUS.Remote.S2 != 2) {
        DJI_Motor_Send(&hfdcan2, 0x200, 0, 0, 0, 0);
        return;
    }
    // 方向切换开关 1 默认方向，-1 整体反转方向
    const int32_t dir_sign = -1;

    static uint8_t last_s1 = 3;
    static uint32_t last_shot_time = 0;
    static float smooth_ref = 0.0f;
    static bool is_init = false;

    if (!is_init) {
        smooth_ref = All_Motor.DJI_2006_bo.DATA.Angle_Infinite;
        g_feeder.target_pos_cnt = (int32_t)(smooth_ref / COUNTS_PER_SHOT);
        is_init = true;
    }
    // 配置不同模式的“控制特性”
    float target_freq = 0.0f;
    bool use_smoothing = false;
    // 两个模式的正常转动方向完全相同，统一使用 dir_sign
    int32_t dir = 1 * dir_sign;

    if (DBUS.Remote.S2 == 1) {
        float raw_freq = 18.0f - (float)DBUS.Remote.Dial / 20.0f;
        target_freq = MATH_Limit_float(25.0f, 0.0f, raw_freq);
        use_smoothing = true;   // 模式一：开启平滑
    }
    else if (DBUS.Remote.S2 == 2) {
        float raw_freq = 10.0f - (float)DBUS.Remote.Dial / 30.0f;
        target_freq = MATH_Limit_float(25.0f, 0.0f, raw_freq);
        use_smoothing = false;  // 模式二：关闭平滑
    }
    // 统一频率限幅与时间判定
    uint32_t now = HAL_GetTick();
    float interval = 1000.0f / target_freq;

    // 统一正常射击触发判定
    if ((DBUS.Remote.S1 == 2 && last_s1 == 3) || (DBUS.Remote.S1 == 1)) {
        if (now - last_shot_time >= (uint32_t)interval) {
            g_feeder.target_pos_cnt += dir;
            last_shot_time = now;
        }
    }
    last_s1 = DBUS.Remote.S1;

    // 分模式处理卡弹（堵转）逻辑
    if (All_Motor.DJI_2006_bo.DATA.Stuck_Flag[1] == 1)
    {
        All_Motor.DJI_2006_bo.PID_S.Output = 0.0f;
        All_Motor.DJI_2006_bo.PID_S.Iout = 0.0f;
        float current_exact_pop = All_Motor.DJI_2006_bo.DATA.Angle_Infinite / COUNTS_PER_SHOT;

        if (dir > 0) {
            // 正转时卡弹（例如10.1）：必须去 11，使用 ceilf 向上取整
            g_feeder.target_pos_cnt = (int32_t)ceilf(current_exact_pop);

            // 如果正好卡在整数点（概率极低），强制向前再推一发，避免目标没有更新
            if ((float)g_feeder.target_pos_cnt - current_exact_pop < 0.01f) {
                g_feeder.target_pos_cnt += 1;
            }
        } else {
            // 反转时卡弹（例如 -10.1）：必须去 -11，使用 floorf 向下取整
            g_feeder.target_pos_cnt = (int32_t)floorf(current_exact_pop);

            // 如果正好卡在整数点，强制向前（负方向）再推一发
            if (current_exact_pop - (float)g_feeder.target_pos_cnt < 0.01f) {
                g_feeder.target_pos_cnt -= 1;
            }
        }
        // 让平滑参考值立刻对齐当前实际位置，这样下一帧它会顺着‘前进方向’平滑移动到新的整弹点
        smooth_ref = All_Motor.DJI_2006_bo.DATA.Angle_Infinite;
        // 处理完毕，手动清除堵转标志位
        All_Motor.DJI_2006_bo.DATA.Stuck_Flag[1] = 0;
    }
    // 统一目标计算与运动平滑控制 (Ramp 阶跃生成器)
    float final_target = (float)g_feeder.target_pos_cnt * COUNTS_PER_SHOT;

    if (use_smoothing) {
        float step = (target_freq * COUNTS_PER_SHOT) / 1000.0f;

        if (smooth_ref > final_target) {
            smooth_ref -= step;
            if (smooth_ref < final_target) smooth_ref = final_target;
        } else if (smooth_ref < final_target) {
            smooth_ref += step;
            if (smooth_ref > final_target) smooth_ref = final_target;
        }
    } else {
        // 模式二关闭平滑，目标值直接阶跃过去，爆发力最强
        smooth_ref = final_target;
    }

    // 7. 统一底层电机控制与 CAN 发送
    All_Motor.DJI_2006_bo.PID_P.Ref = smooth_ref;
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_P, All_Motor.DJI_2006_bo.DATA.Angle_Infinite, smooth_ref);
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_S, All_Motor.DJI_2006_bo.DATA.Speed_now, All_Motor.DJI_2006_bo.PID_P.Output);

    DJI_Motor_Stuck_Check(&All_Motor.DJI_2006_bo, 6000, 100, 100, 500);
    DJI_Motor_Send(&hfdcan2, 0x200, 0, 0, All_Motor.DJI_2006_bo.PID_S.Output, 0);
}
*/

ShootDet_t g_det = {0};

bool Update_Shoot_Det(float speed1, float speed2, ShootDet_t *det) {
    det->last_cnt = det->cnt;
    float val = (fabsf(speed1) + fabsf(speed2)) / 2.0f;
    if (!det->init) {
        det->base = val;
        det->last_val = val;
        det->max_drop_in_round = 0;
        det->cool_down_cnt = 0;
        det->init = true;
        return false;
    }
    float slope = det->last_val - val;
    det->last_val = val;
    if (val > det->base) {
        det->base = (K_UP * val) + (1.0f - K_UP) * det->base;
    } else {
        det->base = (K_DN * val) + (1.0f - K_DN) * det->base;
    }
    float drop = det->base - val;
    bool shoot_done = false;
    if (det->cool_down_cnt > 0) {
        det->cool_down_cnt--;
        det->armed = false;
        return false;
    }
    if (!det->armed) {
        if (drop > TH_FIRE && drop < TH_FIRE_MAX && slope > MIN_SLOPE && val > 4500) {
            det->armed = true;
            det->max_drop_in_round = drop;
            det->t_out = 0;
        }
    } else {
        det->t_out++;
        if (drop > det->max_drop_in_round) {
            det->max_drop_in_round = drop;
        }
        bool condition_relative = (drop < det->max_drop_in_round * (1.0f - RELATIVE_RECOVER));
        bool condition_absolute = (drop < TH_RST_SAFE);
        if (condition_relative || condition_absolute) {
            det->armed = false;
            det->cnt++;
            det->cool_down_cnt = COOL_DOWN_TICKS;
            det->max_drop_in_round = 0;
            shoot_done = true;
        }
        else if (det->t_out >= TIMEOUT_TICKS) {
            det->armed = false;
            det->max_drop_in_round = 0;
        }
    }
    return shoot_done;
}

void Heat_Freq_Ctrl(float kp, User_Data_T *user_data, Feeder_t *feeder, ShootDet_t *det, float speed1, float speed2) {
    static float heat = 0.0f;

    if (Update_Shoot_Det(speed1, speed2, det)) {
        heat += 10.0f;
    }

    float max_heat = (float)user_data->robot_status.shooter_barrel_heat_limit;
    heat = MATH_Limit_float(max_heat, 0.0f, heat);

    heat -= user_data->robot_status.shooter_barrel_cooling_value * 0.005f;
    if (heat < user_data->power_heat_data.shooter_17mm_barrel_heat) {
        heat = (float)user_data->power_heat_data.shooter_17mm_barrel_heat;
    }
    feeder->target_freq = kp * (max_heat - heat);
    feeder->target_freq = MATH_Limit_float(20.0f, 0.0f, feeder->target_freq);
}