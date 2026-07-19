//
// Created by DS on 2026/3/9.
//

#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "main.h"
#include "cmsis_os.h"

// ================= 1. FreeRTOS 句柄声明 =================
extern osThreadId_t Task_ControlHandle;
extern osThreadId_t Task_DepthHandle;
extern osThreadId_t Task_CommsHandle;

extern osMessageQueueId_t Queues_CmdHandle;

extern osSemaphoreId_t Sem_IMU_ReadyHandle;
extern osSemaphoreId_t Sem_LoRa_RxHandle;

extern osMutexId_t Mutex_StateHandle;

// ================= 2. 数据结构定义 =================
// 机器人工作模式
typedef enum {
    MODE_IDLE,      // 怠速/锁定
    MODE_HOVER,     // 定深悬停
    MODE_AUTO,      // 直航 (锁定航向 + 前进推力)
    MODE_TURN,      // 转向到指定航向 (到达后自动切 HOVER)
    MODE_SQUARE     // 方形轨迹自动导航
} RobotMode_e;

// 上位机指令结构
typedef struct {
    uint8_t cmd_type;
    float   param1;
    float   param2;
    uint32_t duration;
} Command_t;

// 机器人全局状态 (这就是我们要保护的资源)
typedef struct {
    RobotMode_e current_mode;
    float current_depth;    // 当前深度 (m)
    float target_depth;     // 目标深度 (m)

    float current_yaw;      // 当前航向 (deg)
    float target_yaw;       // 目标航向 (deg)

    float current_pitch;    // 当前俯仰 (deg)
    float target_pitch;     // 目标俯仰 (deg), 通常 = 0 (保持水平)

    float current_roll;     // 当前横滚 (deg)
    float target_roll;      // 目标横滚 (deg), 通常 = 0 (保持水平)

    float target_surge;     // 目标前进推力 [-1.0, +1.0]

    // --- 速度估算 (IMU 加速度积分) ---
    float estimated_speed;  // 估算前进速度 (m/s), 由 accel_x 积分

    // --- 方形轨迹状态机 ---
    uint8_t  sq_leg;              // 当前边编号 0~3
    uint8_t  sq_phase;            // 0=直行, 1=转向
    float    sq_surge;            // 方形轨迹推力
    uint32_t sq_leg_duration_ms;  // 每条边的运行时间 (ms)
    float    sq_start_yaw;        // 方形轨迹起始航向 (deg)
    uint32_t sq_phase_start_tick; // 当前阶段起始时钟 (tick)
} RobotState_t;

// ================= 3. 全局变量声明 =================
extern RobotState_t g_robotState;

#endif //ROBOT_DEF_H