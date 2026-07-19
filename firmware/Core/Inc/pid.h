//
// Created by DS on 2026/3/9.
// PID 控制器 - 通用位置式 PID, 带积分限幅与输出限幅
//

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

// ================= PID 控制器结构体 =================
typedef struct {
    // --- 参数 ---
    float Kp;
    float Ki;
    float Kd;

    float output_max;       
    float output_min;     
    float integral_max;    
    float deadband;         

    // --- 内部状态 ---
    float integral;        
    float last_error;      
    float output;          
} PID_Controller_t;

// ================= API =================

/**
 * @brief  初始化 PID 控制器参数
 */
void PID_Init(PID_Controller_t *pid,
              float kp, float ki, float kd,
              float out_max, float out_min,
              float integral_max, float deadband);

/**
 * @brief  PID 计算 (位置式)
 * @param  pid     
 * @param  target 
 * @param  current 
 * @param  dt    
 * @return 
 */
float PID_Compute(PID_Controller_t *pid, float target, float current, float dt);

/**
 * @brief  重置 PID 内部状态 (积分清零, 误差清零)
 */
void PID_Reset(PID_Controller_t *pid);

/**
 * @brief  角度误差归一化到 [-180, +180]
 *        
 */
float Angle_Normalize(float angle);

#ifdef __cplusplus
}
#endif

#endif // PID_H

