//
// Created by CaoKangqi on 2026/5/13.
//

#ifndef G4_FRAMEWORK_SHOOT_TASK_H
#define G4_FRAMEWORK_SHOOT_TASK_H

#include "All_Init.h"

typedef enum {
    FEEDER_STOP = 0,
    FEEDER_SINGLE,
    FEEDER_BURST
} FeederMode_e;

typedef struct {
    FeederMode_e mode;
    int32_t target_pos_cnt;      // 目标弹槽索引 (绝对位置)
    uint8_t last_trigger_state;  // 记录上一次触发状态 (用于单发边缘检测)
    float   target_freq;
} Feeder_t;

extern Feeder_t g_feeder;
// 拨弹电机状态结构体
typedef struct {
    int32_t  target_pos_cnt;
    float    target_pos;
    float    smooth_ref;
    uint8_t  is_init;
    uint8_t  last_btn_state;
} DM4310_Feeder_t;

// 云台校准状态机
typedef enum {
    CALIB_START = 0,
    CALIB_MOVING,
    CALIB_DONE,
    CALIB_NORMAL
} Calib_State_t;

// 扳机电机状态机
typedef enum {
    PULL_STATE_NORMAL = 0,
    PULL_STATE_TRIGGERED,
    PULL_STATE_RESET,
    PULL_STATE_STOPPED
} Pull_State_t;

// 全局变量声明
extern DM4310_Feeder_t g_feed_motor;
extern Calib_State_t   g_calib_state;
extern Pull_State_t    g_pull_state;

// 函数声明
void Shoot_Control_Init(void);
void Ctrl_Shoot_Task(void);
void Motor_Calibration_Task(void);
uint8_t Check_Motor_Reached_Limit(DJI_MOTOR_Typedef* motor, float target_speed, float stuck_current, uint16_t confirm_time);

//这下面是射击检测的
#define K_UP             0.673f   // 上升系数
#define K_DN             0.142f   // 下降系数
#define TH_FIRE          200.0f   // 触发阈值
#define TH_FIRE_MAX      1200.0f  // 最大触发阈值
#define MIN_SLOPE        80.0f    // 最小斜率阈值
#define RELATIVE_RECOVER 0.25f    // 回升比例
#define TH_RST_SAFE      100.0f   // 复位阈值
#define TIMEOUT_TICKS    14       // 超时上限
#define COOL_DOWN_TICKS  2        // 冷却周期

// 重新定义的结构体
typedef struct {
    float base;              // 动态基准线
    float last_val;          // 记录上一次的转速，用于算斜率
    float max_drop_in_round; // 记录单次触发过程中的最大跌落深度
    bool  armed;             // 触发状态
    uint32_t cnt;            // 计数器
    uint32_t last_cnt;
    uint8_t  t_out;          // 超时计数器
    uint8_t  cool_down_cnt;  // 冷却计数器
    bool  init;              // 初始化标志
} ShootDet_t;

extern ShootDet_t g_det;
bool Update_Shoot_Det(float speed1, float speed2, ShootDet_t *det);
void Heat_Freq_Ctrl(float kp, User_Data_T *user_data, Feeder_t *feeder, ShootDet_t *det, float speed1, float speed2);
#endif //G4_FRAMEWORK_SHOOT_TASK_H