//
// Created by DS on 2026/3/9.
// PID 控制器实现
//

#include "pid.h"

// -------------------------------------------------------
void PID_Init(PID_Controller_t *pid,
              float kp, float ki, float kd,
              float out_max, float out_min,
              float integral_max, float deadband)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->output_max   = out_max;
    pid->output_min   = out_min;
    pid->integral_max = integral_max;
    pid->deadband     = deadband;

    pid->integral   = 0.0f;
    pid->last_error = 0.0f;
    pid->output     = 0.0f;
}

// -------------------------------------------------------
float PID_Compute(PID_Controller_t *pid, float target, float current, float dt)
{
    if (dt <= 0.0f) return pid->output; // 安全保护

    float error = target - current;

    // --- 死区处理 ---
    if (error > -pid->deadband && error < pid->deadband)
    {
        error = 0.0f;
    }

    // --- 积分累积 (带积分限幅) ---
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    // --- 微分 ---
    float derivative = (error - pid->last_error) / dt;

    // --- PID 输出 ---
    float output = pid->Kp * error
                 + pid->Ki * pid->integral
                 + pid->Kd * derivative;

    // --- 输出限幅 ---
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    // --- 保存状态 ---
    pid->last_error = error;
    pid->output     = output;

    return output;
}

// -------------------------------------------------------
void PID_Reset(PID_Controller_t *pid)
{
    pid->integral   = 0.0f;
    pid->last_error = 0.0f;
    pid->output     = 0.0f;
}

// -------------------------------------------------------
float Angle_Normalize(float angle)
{
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

