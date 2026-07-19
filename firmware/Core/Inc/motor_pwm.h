//
// Created by DS on 2026/3/9.
// 电机 PWM 输出驱动
//
// 电调信号: 1000μs ~ 2000μs (50Hz / 20ms 周期)
//   1500μs = 停转 (中位)
//   1000μs = 全速反转
//   2000μs = 全速正转
//
// 硬件配置:
//   TIM2 CH1 (PA0) → VF
//   TIM2 CH2 (PA1) → VRL
//   TIM2 CH3 (PA2) → VRR
//   TIM2 CH4 (PA3) → HL
//   TIM3 CH1 (PA6) → HR
//
//   Prescaler = 84-1, ARR = 20000-1 → 50Hz, CCR 范围 1000~2000
//

#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "mixer.h"  // for MOT_COUNT, MotorIndex_e

// PWM 参数
#define PWM_NEUTRAL   1500   
#define PWM_MIN       1000   
#define PWM_MAX       2000   

/**
 * @brief  启动所有电机 PWM 通道, 初始输出中位 (1500μs)
 * @note  
 */
void Motor_PWM_Init(void);

/**
 * @brief 设置单个电机的推力
 * @param  index  
 * @param  thrust  
 *                  
 */
void Motor_PWM_Set(MotorIndex_e index, float thrust);

/**
 * @brief  将混控输出一次性写入所有电机
 * @param  out  混控计算结果
 */
void Motor_PWM_SetAll(const MotorOutput_t *out);

/**
 * @brief  所有电机输出中位 (停转)
 */
void Motor_PWM_AllStop(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_PWM_H

