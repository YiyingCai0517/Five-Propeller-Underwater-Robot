//
// Created by DS on 2026/3/9.
// 推进器混控矩阵实现
//
// 混控公式:
//   VF  = heave + pitch_vf * pitch_torque
//   VRL = heave - pitch_vr * pitch_torque + roll_vr * roll_torque
//   VRR = heave - pitch_vr * pitch_torque - roll_vr * roll_torque
//   HL  = surge + yaw_h * yaw_torque
//   HR  = surge - yaw_h * yaw_torque
//
// 混控后进行等比缩放限幅, 保证所有输出在 [-1.0, +1.0] 且方向不失真。
//

#include "mixer.h"

// 辅助: 取绝对值
static float fabsf_local(float x) { return x >= 0.0f ? x : -x; }

// -------------------------------------------------------
void Mixer_Init(MixerCoeff_t *coeff,
                float pitch_vf, float pitch_vr,
                float roll_vr, float yaw_h)
{
    coeff->pitch_vf = pitch_vf;
    coeff->pitch_vr = pitch_vr;
    coeff->roll_vr  = roll_vr;
    coeff->yaw_h    = yaw_h;
}

// -------------------------------------------------------
void Mixer_Compute(const MixerCoeff_t *coeff,
                   float heave, float pitch_torque, float roll_torque,
                   float surge, float yaw_torque,
                   MotorOutput_t *out)
{
    // === 垂直组混控 ===
    // VF 在前方中心线上: 只响应 heave 和 pitch, 不响应 roll (侧向力臂为0)
    out->thrust[MOT_VF]  = heave + coeff->pitch_vf * pitch_torque;

    // VRL 在尾部左侧: heave - pitch (后方力臂反向) + roll (左侧正向)
    out->thrust[MOT_VRL] = heave - coeff->pitch_vr * pitch_torque
                                 + coeff->roll_vr  * roll_torque;

    // VRR 在尾部右侧: heave - pitch (后方力臂反向) - roll (右侧反向)
    out->thrust[MOT_VRR] = heave - coeff->pitch_vr * pitch_torque
                                 - coeff->roll_vr  * roll_torque;

    // HL 左侧: 正 yaw_torque 时左推加推 → 机器向右转
    out->thrust[MOT_HL]  = surge + coeff->yaw_h * yaw_torque;

    // HR 右侧: 正 yaw_torque 时右推减推
    out->thrust[MOT_HR]  = surge - coeff->yaw_h * yaw_torque;

    // 找到绝对值最大的电机输出
    float max_abs = 0.0f;
    for (int i = 0; i < MOT_COUNT; i++)
    {
        float abs_val = fabsf_local(out->thrust[i]);
        if (abs_val > max_abs) max_abs = abs_val;
    }

    // 如果最大值超过 1.0, 所有输出等比缩小 (保持方向)
    if (max_abs > 1.0f)
    {
        float scale = 1.0f / max_abs;
        for (int i = 0; i < MOT_COUNT; i++)
        {
            out->thrust[i] *= scale;
        }
    }
}

// -------------------------------------------------------
void Mixer_AllZero(MotorOutput_t *out)
{
    for (int i = 0; i < MOT_COUNT; i++)
    {
        out->thrust[i] = 0.0f;
    }
}

