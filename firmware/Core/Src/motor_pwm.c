//
// Created by DS on 2026/3/9.
// 电机 PWM 输出驱动实现

#include "motor_pwm.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} MotorPWMMap_t;

static const MotorPWMMap_t motor_map[MOT_COUNT] = {
    [MOT_VF]  = { &htim2, TIM_CHANNEL_2 },  // PA1 - 垂直前方
    [MOT_VRL] = { &htim2, TIM_CHANNEL_3 },  // PA2 - 垂直尾左
    [MOT_VRR] = { &htim2, TIM_CHANNEL_4 },  // PA3 - 垂直尾右
    [MOT_HL]  = { &htim3, TIM_CHANNEL_3 },  // PB0 - 水平左
    [MOT_HR]  = { &htim3, TIM_CHANNEL_4 },  // PB1 - 水平右
};

// === 辅助: 推力归一化值转 PWM CCR ===
static uint16_t Thrust_To_CCR(float thrust)
{
    // 限幅
    if (thrust >  1.0f) thrust =  1.0f;
    if (thrust < -1.0f) thrust = -1.0f;

    // [-1.0, +1.0] → [1000, 2000]
    return (uint16_t)(PWM_NEUTRAL + thrust * (PWM_MAX - PWM_NEUTRAL));
}

// -------------------------------------------------------
void Motor_PWM_Init(void)
{

    // 启动所有 PWM 通道, 初始输出中位
    for (int i = 0; i < MOT_COUNT; i++)
    {
        __HAL_TIM_SET_COMPARE(motor_map[i].htim, motor_map[i].channel, PWM_NEUTRAL);
        HAL_TIM_PWM_Start(motor_map[i].htim, motor_map[i].channel);
    }
}

// -------------------------------------------------------
void Motor_PWM_Set(MotorIndex_e index, float thrust)
{
    if (index >= MOT_COUNT) return;

    uint16_t ccr = Thrust_To_CCR(thrust);
    __HAL_TIM_SET_COMPARE(motor_map[index].htim, motor_map[index].channel, ccr);
}

// -------------------------------------------------------
void Motor_PWM_SetAll(const MotorOutput_t *out)
{
    for (int i = 0; i < MOT_COUNT; i++)
    {
        uint16_t ccr = Thrust_To_CCR(out->thrust[i]);
        __HAL_TIM_SET_COMPARE(motor_map[i].htim, motor_map[i].channel, ccr);
    }
}

// -------------------------------------------------------
void Motor_PWM_AllStop(void)
{
    for (int i = 0; i < MOT_COUNT; i++)
    {
        __HAL_TIM_SET_COMPARE(motor_map[i].htim, motor_map[i].channel, PWM_NEUTRAL);
    }
}

