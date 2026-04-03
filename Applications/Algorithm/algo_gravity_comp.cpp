/**
 * @file algo_gravity_comp.cpp
 * @brief 示教器重力补偿与虚拟阻尼算法实现
 * @author Ciallo
 * @date 2026-01-20
 * @note 2026-02-08: 新增虚拟阻尼功能
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

    // 填充调试信息
    lastDebug_.q2 = q2;
    lastDebug_.q3 = q3;
    lastDebug_.q4 = q4;
    lastDebug_.q5 = q5;
    lastDebug_.c2 = c2;
    lastDebug_.c23 = c23;
    lastDebug_.s23 = s23;
    lastDebug_.c4 = c4;
    lastDebug_.c5 = c5;
    lastDebug_.s5 = s5;

    // Yaw (不补偿)
    torques.tau_yaw = 0.0f;

    // Pitch1 (τ2): MATLAB推导公式 2026-02-04 (修正DH参数后)
    // tau_p1 = K2_c2*c2 + K2_c23*c23 + K2_end*(c5*c23*c4 - s5*s23)
    lastDebug_.p1_self = coeffs_.K2_c2 * c2;
    lastDebug_.p1_link = coeffs_.K2_c23 * c23;
    lastDebug_.p1_end = coeffs_.K2_end * (c5 * c23 * c4 - s5 * s23);
    lastDebug_.p1_total = lastDebug_.p1_self + lastDebug_.p1_link + lastDebug_.p1_end;
    torques.tau_pitch1 = lastDebug_.p1_total;

    // Pitch2 (τ3): MATLAB推导公式 2026-02-04 (修正DH参数后)
    // tau_p2 = K3_c23*c23 + K3_s23*s23 + K3_end*(c5*c23*c4 - s5*s23)
    lastDebug_.p2_link = coeffs_.K3_c23 * c23 + coeffs_.K3_s23 * s23;
    lastDebug_.p2_end = coeffs_.K3_end * (c5 * c23 * c4 - s5 * s23);
    lastDebug_.p2_total = lastDebug_.p2_link + lastDebug_.p2_end;
    torques.tau_pitch2 = lastDebug_.p2_total;

    // Roll (τ4): 修正后Roll轴补偿较小，暂设为0
    torques.tau_roll = 0.0f;

    // PitchEnd (τ5): MATLAB推导公式 2026-02-04 (修正DH参数后)
    // tau_pitchend = K5_1*(c5*c23 - s5*s23*c4)
    // 修正：当 Roll 接近 ±90° 时，PitchEnd 轴变成 Yaw 方向，不再抵抗重力
    // 因此整个重力补偿需要乘以 c4（Roll 的余弦），Roll=0° 时 c4=1，Roll=90° 时 c4=0
    lastDebug_.pEnd_grav = coeffs_.K5_1 * (c5 * c23) * c4;
    lastDebug_.pEnd_roll = coeffs_.K5_1 * (-s5 * s23 * c4);
    lastDebug_.pEnd_total = lastDebug_.pEnd_grav + lastDebug_.pEnd_roll;
    torques.tau_pitchEnd = lastDebug_.pEnd_total;

    // 补偿比例:这里是用来根据实际情况后期调整的
    torques.tau_pitch1   *= scale_;
    torques.tau_pitch2   *= scale_;
    torques.tau_roll     *= scale_;
    torques.tau_pitchEnd *= scale_;

    return torques;
}

/**
 * @brief 计算虚拟阻尼力矩（仅PitchEnd）
 * @param rawVelocity 原始速度反馈 (rad/s)，已由调用方转换
 * @return 阻尼力矩 (N·m)，方向与速度相反
 * @note 使用滑动平均滤波 + 方向一致性检查，避免延迟导致的抖动
 */
float CAlgoGravityComp::CalculateDamping_PitchEnd(float rawVelocity) {
    // 未使能时返回0
    if (!dampingEnabled_) {
        lastDebug_.damping_vel_raw = rawVelocity;
        lastDebug_.damping_vel_filtered = 0.0f;
        lastDebug_.damping_torque = 0.0f;
        return 0.0f;
    }

    // 1. 更新滑动窗口缓冲区
    velocityBuffer_pitchEnd_[bufferIndex_pitchEnd_] = rawVelocity;
    bufferIndex_pitchEnd_ = (bufferIndex_pitchEnd_ + 1) % VELOCITY_BUFFER_SIZE;

    // 2. 计算滑动平均
    float sum = 0.0f;
    for (int i = 0; i < VELOCITY_BUFFER_SIZE; ++i) {
        sum += velocityBuffer_pitchEnd_[i];
    }
    float avgVelocity = sum / static_cast<float>(VELOCITY_BUFFER_SIZE);

    // 3. 方向一致性检查：当前速度和平均速度方向必须一致才输出阻尼
    //    这可以防止速度突变时产生错误方向的阻尼力矩
    float effectiveVelocity = 0.0f;
    bool sameDirection = (rawVelocity * avgVelocity) >= 0.0f;  // 同号或有一个为0

    if (sameDirection && fabsf(avgVelocity) > dampingParams_.deadzone) {
        // 使用当前速度而非滤波速度，减少延迟
        // 但幅值用平均值，减少噪声
        effectiveVelocity = (fabsf(rawVelocity) < fabsf(avgVelocity)) ? rawVelocity : avgVelocity;
    }

    // 4. 计算阻尼力矩: τ = -B × v
    float dampingTorque = -dampingParams_.B_pitchEnd * effectiveVelocity;

    // 5. 力矩限幅，防止过大的阻尼力矩
    dampingTorque = std::clamp(dampingTorque, -dampingParams_.maxTorque, dampingParams_.maxTorque);

    // 6. 更新上一次速度（用于下次方向检查）
    lastVelocity_pitchEnd_ = rawVelocity;

    // 7. 填充调试信息
    lastDebug_.damping_vel_raw = rawVelocity;
    lastDebug_.damping_vel_filtered = avgVelocity;
    lastDebug_.damping_torque = dampingTorque;

    return dampingTorque;
}

/**
 * @brief 计算虚拟阻尼力矩（Roll）
 * @param rawVelocity 原始速度反馈 (rad/s)，已由调用方转换
 * @return 阻尼力矩 (N·m)，方向与速度相反
 * @note 使用滑动平均滤波 + 方向一致性检查，避免延迟导致的抖动
 */
float CAlgoGravityComp::CalculateDamping_Roll(float rawVelocity) {
    // 未使能时返回0
    if (!dampingEnabled_) {
        lastDebug_.damping_roll_vel_raw = rawVelocity;
        lastDebug_.damping_roll_vel_filtered = 0.0f;
        lastDebug_.damping_roll_torque = 0.0f;
        return 0.0f;
    }

    // 1. 更新滑动窗口缓冲区
    velocityBuffer_roll_[bufferIndex_roll_] = rawVelocity;
    bufferIndex_roll_ = (bufferIndex_roll_ + 1) % VELOCITY_BUFFER_SIZE;

    // 2. 计算滑动平均
    float sum = 0.0f;
    for (int i = 0; i < VELOCITY_BUFFER_SIZE; ++i) {
        sum += velocityBuffer_roll_[i];
    }
    float avgVelocity = sum / static_cast<float>(VELOCITY_BUFFER_SIZE);

    // 3. 方向一致性检查：当前速度和平均速度方向必须一致才输出阻尼
    float effectiveVelocity = 0.0f;
    bool sameDirection = (rawVelocity * avgVelocity) >= 0.0f;

    if (sameDirection && fabsf(avgVelocity) > dampingParams_.deadzone) {
        effectiveVelocity = (fabsf(rawVelocity) < fabsf(avgVelocity)) ? rawVelocity : avgVelocity;
    }

    // 4. 计算阻尼力矩: τ = -B × v
    float dampingTorque = -dampingParams_.B_roll * effectiveVelocity;

    // 5. 力矩限幅
    dampingTorque = std::clamp(dampingTorque, -dampingParams_.maxTorque, dampingParams_.maxTorque);

    // 6. 更新上一次速度
    lastVelocity_roll_ = rawVelocity;

    // 7. 填充调试信息
    lastDebug_.damping_roll_vel_raw = rawVelocity;
    lastDebug_.damping_roll_vel_filtered = avgVelocity;
    lastDebug_.damping_roll_torque = dampingTorque;

    return dampingTorque;
}

} // namespace my_engineer
