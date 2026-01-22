/**
 * @file algo_gravity_comp.cpp
 * @brief 示教器重力补偿算法实现
 * @author Ciallo
 * @date 2026-01-20
 */

#include "algo_gravity_comp.hpp"

namespace my_engineer {

// 计算补偿力矩 (输入编码器角度)
CAlgoGravityComp::SGravTorques CAlgoGravityComp::Calculate(
    float enc_pitch1, float enc_pitch2,
    float enc_roll, float enc_pitchEnd)
{
    // 编码器角度->DH角度
    float q2 = enc_pitch1   + DH_OFFSET_PITCH1;
    float q3 = enc_pitch2   + DH_OFFSET_PITCH2;
    float q4 = enc_roll     + DH_OFFSET_ROLL;
    float q5 = enc_pitchEnd + DH_OFFSET_PITCHEND;

    return CalculateDH(q2, q3, q4, q5);
}

// 计算补偿力矩 (输入DH角度)
CAlgoGravityComp::SGravTorques CAlgoGravityComp::CalculateDH(
    float q2, float q3, float q4, float q5)
{
    SGravTorques torques;

    if (!enabled_) {
        return torques;
    }

    // 预计算三角函数
    const float s2 = sinf(q2);
    const float c2 = cosf(q2);
    const float s4 = sinf(q4);
    const float c4 = cosf(q4);
    const float s5 = sinf(q5);
    const float c5 = cosf(q5);

    const float q23 = q2 + q3;
    const float s23 = sinf(q23);
    const float c23 = cosf(q23);

    // Yaw (不补偿)
    torques.tau_yaw = 0.0f;

    // Pitch1: tau2 = K2_1*c2 + K2_5*s2 + K2_2*s23 - K2_4*c23*s4 - K2_3*s23*s5 + K2_3*c23*c4*c5
    torques.tau_pitch1 = coeffs_.K2_1 * c2
                       + coeffs_.K2_5 * s2
                       + coeffs_.K2_2 * s23
                       - coeffs_.K2_4 * c23 * s4
                       - coeffs_.K2_3 * s23 * s5
                       + coeffs_.K2_3 * c23 * c4 * c5;

    // Pitch2: tau3 = K3_1*s23 - K3_3*c23*s4 - K3_2*s23*s5 + K3_2*c23*c4*c5
    torques.tau_pitch2 = coeffs_.K3_1 * s23
                       - coeffs_.K3_3 * c23 * s4
                       - coeffs_.K3_2 * s23 * s5
                       + coeffs_.K3_2 * c23 * c4 * c5;

    // Roll: tau4 = -K3_3*s23*c4 - K3_2*s23*s4*c5
    torques.tau_roll = -coeffs_.K3_3 * s23 * c4
                     - coeffs_.K3_2 * s23 * s4 * c5;

    // PitchEnd: tau5 = K5_1*c23*c5 - K5_1*s23*c4*s5
    torques.tau_pitchEnd = coeffs_.K5_1 * c23 * c5
                         - coeffs_.K5_1 * s23 * c4 * s5;

    // 补偿比例:这里是用来根据实际情况后期调整的，请跟前面的补偿系数区分
    torques.tau_pitch1   *= scale_;
    torques.tau_pitch2   *= scale_;
    torques.tau_roll     *= scale_;
    torques.tau_pitchEnd *= scale_;

    return torques;
}

} // namespace my_engineer
