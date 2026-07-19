//
// Created by DS on 2026/3/9.
// 推进器混控矩阵 - 5 推进器鱼雷型 AUV
//
// 推进器布局 (从上方俯视, X轴朝前):
//
//         VF (垂直, 前方中心)
//          |
//    HL ---+--- HR  (水平, 舱体中部左右)
//          |
//      VRL   VRR   (垂直, 尾部左右)
//
// 垂直组 [VF, VRL, VRR]: 控制 heave(升降) + pitch(俯仰) + roll(横滚)
// 水平组 [HL, HR]:        控制 surge(前进) + yaw(偏航)
//

#ifndef MIXER_H
#define MIXER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// ================= 电机索引枚举 =================
typedef enum {
    MOT_VF  = 0,  
    MOT_VRL = 1,  
    MOT_VRR = 2,   
    MOT_HL  = 3,   
    MOT_HR  = 4,  
    MOT_COUNT = 5
} MotorIndex_e;

// ================= 混控输出结构体 =================
typedef struct {
    float thrust[MOT_COUNT];  // 归一化推力 [-1.0, +1.0]
} MotorOutput_t;

// ================= 混控系数 (可调参) =================
// 这些系数反映了推进器到重心的力臂比例, 需要根据实际机械结构调整
typedef struct {
    float pitch_vf;   
    float pitch_vr;   
    float roll_vr;     
    float yaw_h;       
} MixerCoeff_t;

// ================= API =================

/**
 * @brief  初始化混控系数
 * @param  coeff  混控系数指针
 * @param  pitch_vf 
 * @param  pitch_vr   
 * @param  roll_vr    
 * @param  yaw_h      
 */
void Mixer_Init(MixerCoeff_t *coeff,
                float pitch_vf, float pitch_vr,
                float roll_vr, float yaw_h);

/**
 * @brief  混控计算
 *
 * @param  coeff         
 * @param  heave        
 * @param  pitch_torque  
 * @param  surge         
 * @param  yaw_torque   
 * @param  out          
 */
void Mixer_Compute(const MixerCoeff_t *coeff,
                   float heave, float pitch_torque, float roll_torque,
                   float surge, float yaw_torque,
                   MotorOutput_t *out);

/**
 * @brief  全部电机输出清零
 */
void Mixer_AllZero(MotorOutput_t *out);

#ifdef __cplusplus
}
#endif

#endif // MIXER_H

