#include "app_robot.h"
#include "cmsis_os.h"
#include "robot_def.h"
#include "usart.h"
#include "string.h"
#include "analysis_data.h"
#include "MS5837.h"
#include "pid.h"
#include "mixer.h"
#include "motor_pwm.h"
#include <math.h>

// ================= LED 指示灯宏 =================
// 共阳极接法: RESET=亮, SET=灭
#define LED_RED_ON()      HAL_GPIO_WritePin(LED0_RED_GPIO_Port,  LED0_RED_Pin,  GPIO_PIN_RESET)
#define LED_RED_OFF()     HAL_GPIO_WritePin(LED0_RED_GPIO_Port,  LED0_RED_Pin,  GPIO_PIN_SET)
#define LED_RED_TOGGLE()  HAL_GPIO_TogglePin(LED0_RED_GPIO_Port, LED0_RED_Pin)

// ================= LORA 变量 =================
#define LORA_RX_BUF_SIZE 64
uint8_t lora_rx_buf[LORA_RX_BUF_SIZE];
volatile uint8_t lora_data_len = 0;    // ISR 写, Task 读, 必须 volatile

// --- LoRa TX ---
#define LORA_TX_BUF_SIZE 64
static uint8_t lora_tx_buf[LORA_TX_BUF_SIZE];

// LoRa 定点传输头: 目标地址(0x00 0x00) + 信道(0x17)
#define LORA_PC_ADDR_H  0x00
#define LORA_PC_ADDR_L  0x00
#define LORA_CHANNEL    0x17
#define TELEM_FUNC_ID   0x80
#define TELEM_INTERVAL_MS  500   // 回传间隔 500ms (2Hz)

// 回传 Payload
#pragma pack(push, 1)
typedef struct {
    uint8_t  mode;
    float    depth;
    float    yaw;
    float    pitch;
    float    roll;
    float    target_depth;
    float    target_yaw;
    float    speed;          // 估算前进速度 (m/s)
    float    surge;          // 当前前进推力
} TelemetryPayload_t;   // 33 字节
#pragma pack(pop)

// ================= IMU 变量 =================
#define IMU_RX_BUF_SIZE 256
uint8_t imu_rx_buf[IMU_RX_BUF_SIZE];
volatile uint8_t imu_data_len = 0;

// ================= PID 实例 =================
static PID_Controller_t pid_depth;
static PID_Controller_t pid_pitch;
static PID_Controller_t pid_roll;
static PID_Controller_t pid_yaw;

// ================= 混控实例 =================
static MixerCoeff_t mixer_coeff;
static MotorOutput_t motor_output;

// ============================================================
//  StartControlTask - 核心闭环控制 (Realtime 优先级)
// ============================================================
void StartControlTask(void *argument)
{
    Command_t recv_cmd;

    PID_Init(&pid_depth, 2.0f, 0.3f, 0.5f,  1.0f,  -1.0f,   0.5f,   0.01f);
    PID_Init(&pid_pitch, 1.5f, 0.1f, 0.3f,  0.5f,  -0.5f,   0.3f,   0.5f);
    PID_Init(&pid_roll,  1.5f, 0.1f, 0.3f,  0.5f,  -0.5f,   0.3f,   0.5f);
    PID_Init(&pid_yaw,   1.0f, 0.05f,0.2f,  0.8f,  -0.8f,   0.3f,   1.0f);

    Mixer_Init(&mixer_coeff, 0.5f, 0.5f, 0.5f, 0.5f);
    Motor_PWM_Init();

    // ESC arming: 中位 3 秒
    osDelay(3000);

    // IMU: 开启 IDLE 中断 + DMA
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6, imu_rx_buf, IMU_RX_BUF_SIZE);

    uint32_t last_tick = osKernelGetTickCount();

    for(;;)
    {
        // 0. 计算 dt (必须在 IMU 速度估算之前)
        uint32_t now_tick = osKernelGetTickCount();
        float dt = (float)(now_tick - last_tick) / 1000.0f;
        last_tick = now_tick;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

        // 1. 非阻塞检查指令队列
        if (osMessageQueueGet(Queues_CmdHandle, &recv_cmd, NULL, 0) == osOK)
        {
            osMutexAcquire(Mutex_StateHandle, osWaitForever);
            if (recv_cmd.cmd_type == 1) {
                // HOVER: 定深悬停
                g_robotState.current_mode = MODE_HOVER;
                g_robotState.target_depth = recv_cmd.param1;
                g_robotState.target_pitch = 0.0f;
                g_robotState.target_roll  = 0.0f;
                g_robotState.target_surge = 0.0f;
            } else if (recv_cmd.cmd_type == 2) {
                // AUTO: 直航 (锁定当前航向)
                g_robotState.current_mode = MODE_AUTO;
                g_robotState.target_surge = recv_cmd.param1;
                g_robotState.target_yaw   = g_robotState.current_yaw;
                g_robotState.target_pitch = 0.0f;
                g_robotState.target_roll  = 0.0f;
            } else if (recv_cmd.cmd_type == 4) {
                // TURN: 转向到指定航向
                g_robotState.current_mode = MODE_TURN;
                g_robotState.target_yaw   = recv_cmd.param1;
                g_robotState.target_surge = 0.0f;
                g_robotState.target_pitch = 0.0f;
                g_robotState.target_roll  = 0.0f;
            } else if (recv_cmd.cmd_type == 5) {
                // SQUARE: 方形轨迹自动导航
                g_robotState.current_mode       = MODE_SQUARE;
                g_robotState.sq_leg             = 0;
                g_robotState.sq_phase           = 0;  // 从直行开始
                g_robotState.sq_surge           = recv_cmd.param2;  // 推力
                g_robotState.sq_leg_duration_ms = (uint32_t)(recv_cmd.param1 * 1000.0f); // 边时长
                g_robotState.sq_start_yaw       = g_robotState.current_yaw;
                g_robotState.target_yaw         = g_robotState.current_yaw;
                g_robotState.target_surge       = recv_cmd.param2;
                g_robotState.target_pitch       = 0.0f;
                g_robotState.target_roll        = 0.0f;
                g_robotState.sq_phase_start_tick = osKernelGetTickCount();
            } else {
                // STOP / IDLE
                g_robotState.current_mode = MODE_IDLE;
                g_robotState.target_surge = 0.0f;
                g_robotState.estimated_speed = 0.0f;
            }
            osMutexRelease(Mutex_StateHandle);
        }

        // 2. 等待 IMU 信号量
        if (osSemaphoreAcquire(Sem_IMU_ReadyHandle, 200) == osOK)
        {
            int ret = analysis_data(imu_rx_buf, imu_data_len);
            if (ret == analysis_ok)
            {
                osMutexAcquire(Mutex_StateHandle, osWaitForever);
                g_robotState.current_yaw   = g_output_info.yaw;
                g_robotState.current_pitch = g_output_info.pitch;
                g_robotState.current_roll  = g_output_info.roll;

                // --- 速度估算: IMU 前向加速度积分 ---
                // accel_x 是机体坐标系前向加速度 (m/s^2)
                // 简单积分 + 指数衰减防漂移
                {
                    float ax = g_output_info.accel_x;
                    // 去除重力在前向的分量 (用 pitch 角补偿)
                    float pitch_rad = g_robotState.current_pitch * 0.01745329f; // deg->rad
                    float ax_corrected = ax + 9.81f * sinf(pitch_rad);
                    // 死区: 小加速度视为噪声
                    if (ax_corrected > -0.1f && ax_corrected < 0.1f) ax_corrected = 0.0f;
                    g_robotState.estimated_speed += ax_corrected * dt;
                    // 指数衰减: 防止长时间积分漂移
                    g_robotState.estimated_speed *= 0.98f;
                    // 限幅
                    if (g_robotState.estimated_speed >  3.0f) g_robotState.estimated_speed =  3.0f;
                    if (g_robotState.estimated_speed < -3.0f) g_robotState.estimated_speed = -3.0f;
                }
                osMutexRelease(Mutex_StateHandle);
            }
            memset(imu_rx_buf, 0, IMU_RX_BUF_SIZE);
            HAL_UART_Receive_DMA(&huart6, imu_rx_buf, IMU_RX_BUF_SIZE);
        }
        else
        {
            Motor_PWM_AllStop();
            PID_Reset(&pid_depth);
            PID_Reset(&pid_pitch);
            PID_Reset(&pid_roll);
            PID_Reset(&pid_yaw);
            continue;
        }

        // 3. PID + 混控
        osMutexAcquire(Mutex_StateHandle, osWaitForever);
        RobotMode_e mode = g_robotState.current_mode;
        float cur_depth = g_robotState.current_depth;
        float tgt_depth = g_robotState.target_depth;
        float cur_pitch = g_robotState.current_pitch;
        float tgt_pitch = g_robotState.target_pitch;
        float cur_roll  = g_robotState.current_roll;
        float tgt_roll  = g_robotState.target_roll;
        float cur_yaw   = g_robotState.current_yaw;
        float tgt_yaw   = g_robotState.target_yaw;
        float tgt_surge = g_robotState.target_surge;
        osMutexRelease(Mutex_StateHandle);

        if (mode == MODE_IDLE)
        {
            Motor_PWM_AllStop();
            PID_Reset(&pid_depth);
            PID_Reset(&pid_pitch);
            PID_Reset(&pid_roll);
            PID_Reset(&pid_yaw);
        }
        else
        {
            // ========== MODE_SQUARE 状态机 ==========
            if (mode == MODE_SQUARE)
            {
                osMutexAcquire(Mutex_StateHandle, osWaitForever);
                uint32_t elapsed = osKernelGetTickCount() - g_robotState.sq_phase_start_tick;

                if (g_robotState.sq_phase == 0)
                {
                    // --- 直行阶段 ---
                    tgt_surge = g_robotState.sq_surge;
                    if (elapsed >= g_robotState.sq_leg_duration_ms)
                    {
                        // 时间到 → 切换为转向阶段
                        g_robotState.sq_phase = 1;
                        g_robotState.sq_phase_start_tick = osKernelGetTickCount();
                        // 目标航向 += 90° (向右转, 用 Angle_Normalize 归一化)
                        g_robotState.target_yaw = Angle_Normalize(g_robotState.target_yaw + 90.0f);
                        tgt_yaw = g_robotState.target_yaw;
                        tgt_surge = 0.0f;  // 转向时停止前进
                    }
                }
                else
                {
                    // --- 转向阶段 ---
                    tgt_surge = 0.0f;
                    tgt_yaw = g_robotState.target_yaw;
                    float yaw_err_sq = Angle_Normalize(tgt_yaw - cur_yaw);
                    // 航向误差 < 3° 且至少过了 500ms → 转向完成
                    if ((yaw_err_sq > -3.0f && yaw_err_sq < 3.0f) && elapsed >= 500)
                    {
                        g_robotState.sq_leg++;
                        if (g_robotState.sq_leg >= 4)
                        {
                            // 4 条边走完 → 回到 IDLE
                            g_robotState.current_mode = MODE_IDLE;
                            g_robotState.target_surge = 0.0f;
                            osMutexRelease(Mutex_StateHandle);
                            Motor_PWM_AllStop();
                            PID_Reset(&pid_depth);
                            PID_Reset(&pid_pitch);
                            PID_Reset(&pid_roll);
                            PID_Reset(&pid_yaw);
                            continue;
                        }
                        // 进入下一条边的直行阶段
                        g_robotState.sq_phase = 0;
                        g_robotState.sq_phase_start_tick = osKernelGetTickCount();
                    }
                }
                osMutexRelease(Mutex_StateHandle);
            }

            // ========== MODE_TURN 自动回归 ==========
            if (mode == MODE_TURN)
            {
                float yaw_err_turn = Angle_Normalize(tgt_yaw - cur_yaw);
                if (yaw_err_turn > -3.0f && yaw_err_turn < 3.0f)
                {
                    // 航向到达目标 → 自动切换为 HOVER
                    osMutexAcquire(Mutex_StateHandle, osWaitForever);
                    g_robotState.current_mode = MODE_HOVER;
                    osMutexRelease(Mutex_StateHandle);
                }
            }

            // ========== PID + 混控计算 (所有非 IDLE 模式共用) ==========
            float heave        = PID_Compute(&pid_depth, tgt_depth, cur_depth, dt);
            float pitch_torque = PID_Compute(&pid_pitch, tgt_pitch, cur_pitch, dt);
            float roll_torque  = PID_Compute(&pid_roll,  tgt_roll,  cur_roll,  dt);
            float yaw_error    = Angle_Normalize(tgt_yaw - cur_yaw);
            float yaw_torque   = PID_Compute(&pid_yaw, yaw_error + cur_yaw, cur_yaw, dt);
            float surge = tgt_surge;
            if (mode == MODE_HOVER || mode == MODE_TURN) surge = 0.0f;

            Mixer_Compute(&mixer_coeff, heave, pitch_torque, roll_torque,
                          surge, yaw_torque, &motor_output);
            Motor_PWM_SetAll(&motor_output);
        }
    }
}

// ============================================================
//  StartDepthTask - 深度传感器 (Normal 优先级)
// ============================================================
void StartDepthTask(void *argument)
{
    MS5837Init();

    float atm_sum = 0;
    for (int i = 0; i < 5; i++)
    {
        MS5837Read();
        atm_sum += (float)MS5837_30BA_data.Pressure;
        osDelay(50);
    }
    MS5837_30BA_data.Atmosphere_Pressure = (int32_t)(atm_sum / 5.0f);

    for(;;)
    {
        float depth = MS5837_GetDepth();
        osMutexAcquire(Mutex_StateHandle, osWaitForever);
        g_robotState.current_depth = depth;
        osMutexRelease(Mutex_StateHandle);
        osDelay(50);
    }
}

// ============================================================
//  Parse_LoRa_Command - 解析指令, 返回 1=成功 0=失败
// ============================================================
static int Parse_LoRa_Command(uint8_t *buf, uint8_t len)
{
    if (len < 5) return 0;
    if (buf[0] != 0xAA || buf[1] != 0x55) return 0;

    uint8_t func_id     = buf[2];
    uint8_t payload_len = buf[3];

    if (len < (uint8_t)(4 + payload_len + 1)) return 0;

    uint8_t checksum = 0;
    for (int i = 0; i < 4 + payload_len; i++)
        checksum += buf[i];
    if (checksum != buf[4 + payload_len]) return 0;

    Command_t new_cmd = {0};

    switch (func_id)
    {
    case 0x01:  // HOVER: 定深悬停
        new_cmd.cmd_type = 1;
        if (payload_len >= 4) memcpy(&new_cmd.param1, &buf[4], 4);
        else new_cmd.param1 = 0.15f;
        break;
    case 0x02:  // AUTO: 直航
        new_cmd.cmd_type = 2;
        if (payload_len >= 4) memcpy(&new_cmd.param1, &buf[4], 4);
        break;
    case 0x03:  // STOP: 急停
        new_cmd.cmd_type = 0;
        break;
    case 0x04:  // TURN: 转向到指定航向
        new_cmd.cmd_type = 4;
        if (payload_len >= 4) memcpy(&new_cmd.param1, &buf[4], 4);  // target_yaw (deg)
        break;
    case 0x05:  // SQUARE: 方形轨迹
        new_cmd.cmd_type = 5;
        if (payload_len >= 4) memcpy(&new_cmd.param1, &buf[4], 4);  // 边时长 (秒)
        if (payload_len >= 8) memcpy(&new_cmd.param2, &buf[8], 4);  // 推力
        else new_cmd.param2 = 0.3f;  // 默认推力
        break;
    default:
        return 0;
    }

    osMessageQueuePut(Queues_CmdHandle, &new_cmd, 0, 0);
    return 1;
}

// ============================================================
//  LoRa_SendTelemetry - 构建并发送遥测帧到 PC
// ============================================================
static void LoRa_SendTelemetry(void)
{
    TelemetryPayload_t telem;

    osMutexAcquire(Mutex_StateHandle, osWaitForever);
    telem.mode         = (uint8_t)g_robotState.current_mode;
    telem.depth        = g_robotState.current_depth;
    telem.yaw          = g_robotState.current_yaw;
    telem.pitch        = g_robotState.current_pitch;
    telem.roll         = g_robotState.current_roll;
    telem.target_depth = g_robotState.target_depth;
    telem.target_yaw   = g_robotState.target_yaw;
    telem.speed        = g_robotState.estimated_speed;
    telem.surge        = g_robotState.target_surge;
    osMutexRelease(Mutex_StateHandle);

    uint8_t payload_len = sizeof(TelemetryPayload_t);
    uint8_t frame_len = 3 + 2 + 1 + 1 + payload_len + 1;
    if (frame_len > LORA_TX_BUF_SIZE) return;

    uint8_t pos = 0;
    lora_tx_buf[pos++] = LORA_PC_ADDR_H;
    lora_tx_buf[pos++] = LORA_PC_ADDR_L;
    lora_tx_buf[pos++] = LORA_CHANNEL;
    lora_tx_buf[pos++] = 0xAA;
    lora_tx_buf[pos++] = 0x55;
    lora_tx_buf[pos++] = TELEM_FUNC_ID;
    lora_tx_buf[pos++] = payload_len;
    memcpy(&lora_tx_buf[pos], &telem, payload_len);
    pos += payload_len;
    uint8_t checksum = 0;
    for (uint8_t i = 3; i < pos; i++)
        checksum += lora_tx_buf[i];
    lora_tx_buf[pos++] = checksum;

    if (huart3.gState == HAL_UART_STATE_READY)
    {
        HAL_UART_Transmit_DMA(&huart3, lora_tx_buf, pos);
    }
}

// ============================================================
//  StartCommTask - LoRa 通信 (接收指令 + 定时回传遥测)
//
//  LED 指示:
//    绿灯 500ms 闪烁      = FreeRTOS 心跳 (DefaultTask)
//    红灯每次遥测翻转      = 遥测正在发送 (正常 ~1Hz 闪)
//    红灯快闪 3 次         = 收到有效指令
//    红灯长亮 300ms        = 收到数据但解析失败
// ============================================================
void StartCommTask(void *argument)
{
    // ===== 启动指示: 红灯闪 2 次 =====
    for (int i = 0; i < 2; i++) {
        LED_RED_ON();  osDelay(150);
        LED_RED_OFF(); osDelay(150);
    }

    // 清除残留 IDLE 标志, 防止虚假中断
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3, lora_rx_buf, LORA_RX_BUF_SIZE);

    uint32_t last_telem_tick = osKernelGetTickCount();

    // 本地缓冲: 拷贝后解析, 避免竞态
    uint8_t local_buf[LORA_RX_BUF_SIZE];
    uint8_t local_len = 0;

    for(;;)
    {
        // 等待信号量, 超时 = 遥测间隔 → 保证定时发送
        osStatus_t status = osSemaphoreAcquire(Sem_LoRa_RxHandle, TELEM_INTERVAL_MS);

        if (status == osOK)
        {
            // --- IDLE 中断到达: 有数据 ---
            local_len = lora_data_len;
            if (local_len > LORA_RX_BUF_SIZE) local_len = LORA_RX_BUF_SIZE;
            if (local_len > 0)
                memcpy(local_buf, lora_rx_buf, local_len);
            lora_data_len = 0;

            // 重启 RX DMA
            memset(lora_rx_buf, 0, LORA_RX_BUF_SIZE);
            if (huart3.RxState == HAL_UART_STATE_READY)
                HAL_UART_Receive_DMA(&huart3, lora_rx_buf, LORA_RX_BUF_SIZE);

            // 从本地缓冲解析
            if (local_len > 0)
            {
                if (Parse_LoRa_Command(local_buf, local_len))
                {
                    // 指令成功: 红灯快闪 3 次
                    for (int i = 0; i < 3; i++) {
                        LED_RED_ON();  osDelay(80);
                        LED_RED_OFF(); osDelay(80);
                    }
                }
                else
                {
                    // 解析失败: 红灯亮 300ms
                    LED_RED_ON();  osDelay(300);  LED_RED_OFF();
                }
            }
        }

        // 定时发送遥测
        uint32_t now = osKernelGetTickCount();
        if ((now - last_telem_tick) >= TELEM_INTERVAL_MS)
        {
            LoRa_SendTelemetry();
            last_telem_tick = now;
            LED_RED_TOGGLE();
        }

        // 安全网: 确保 RX DMA 在运行
        if (huart3.RxState == HAL_UART_STATE_READY)
            HAL_UART_Receive_DMA(&huart3, lora_rx_buf, LORA_RX_BUF_SIZE);
    }
}