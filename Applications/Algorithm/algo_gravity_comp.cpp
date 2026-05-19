/**
 * @file algo_gravity_comp.cpp
 * @brief 示教器重力补偿与虚拟阻尼算法实现 (6轴版)
 * @author Ciallo
 * @date 2026-01-20
 * @note 2026-04-18: 6轴重构 — P3/PitchEnd独立建模, Roll仅阻尼
 *
 * DH: q2=Pitch1, q3=Pitch2, q4=P3, q5=Roll, q6=PitchEnd
 * 累积角: q23=q2+q3, q234=q2+q3+q4
 */

#include "algo_gravity_comp.hpp"

extern "C" {
volatile float dbg_gc_q2 = 0.0f;
volatile float dbg_gc_q3 = 0.0f;
volatile float dbg_gc_q4 = 0.0f;
volatile float dbg_gc_q5 = 0.0f;
volatile float dbg_gc_q6 = 0.0f;

volatile float dbg_gc_s2 = 0.0f;
volatile float dbg_gc_c2 = 0.0f;
volatile float dbg_gc_s23 = 0.0f;
volatile float dbg_gc_c23 = 0.0f;
volatile float dbg_gc_s234 = 0.0f;
volatile float dbg_gc_c234 = 0.0f;
volatile float dbg_gc_s5 = 0.0f;
volatile float dbg_gc_c5 = 0.0f;
volatile float dbg_gc_s6 = 0.0f;
volatile float dbg_gc_c6 = 0.0f;

volatile float dbg_gc_tau_pitch1_raw = 0.0f;
volatile float dbg_gc_tau_pitch2_raw = 0.0f;
volatile float dbg_gc_tau_pitch3_raw = 0.0f;
volatile float dbg_gc_tau_pitchEnd_raw = 0.0f;
volatile float dbg_gc_tau_pitch1_scaled = 0.0f;
volatile float dbg_gc_tau_pitch2_scaled = 0.0f;
volatile float dbg_gc_tau_pitch3_scaled = 0.0f;
volatile float dbg_gc_tau_pitchEnd_scaled = 0.0f;
}

namespace my_engineer {

CAlgoGravityComp::SGravTorques CAlgoGravityComp::Calculate(
    float enc_pitch1, float enc_pitch2,
    float enc_pitch3, float enc_roll, float enc_pitchEnd)
{
    const float q2 = enc_pitch1   + DH_OFFSET_PITCH1;
    const float q3 = enc_pitch2   + DH_OFFSET_PITCH2;
    const float q4 = enc_pitch3   + DH_OFFSET_PITCH3;
    const float q5 = enc_roll     + DH_OFFSET_ROLL;
    const float q6 = enc_pitchEnd + DH_OFFSET_PITCHEND;

    return CalculateDH(q2, q3, q4, q5, q6);
}

CAlgoGravityComp::SGravTorques CAlgoGravityComp::CalculateDH(
    float q2, float q3, float q4, float q5, float q6)
{
    SGravTorques torques;

    if (!enabled_) {
        return torques;
    }

    // Precompute trig values (10 calls)
    const float s2 = sinf(q2);
    const float c2 = cosf(q2);

    const float q23 = q2 + q3;
    const float s23 = sinf(q23);
    const float c23 = cosf(q23);

    const float q234 = q23 + q4;
    const float s234 = sinf(q234);
    const float c234 = cosf(q234);

    const float s5 = sinf(q5);
    const float c5 = cosf(q5);
    const float s6 = sinf(q6);
    const float c6 = cosf(q6);

    // Debug: store inputs
    lastDebug_.q2 = q2; lastDebug_.q3 = q3; lastDebug_.q4 = q4;
    lastDebug_.q5 = q5; lastDebug_.q6 = q6;
    lastDebug_.s2 = s2; lastDebug_.c2 = c2;
    lastDebug_.s23 = s23; lastDebug_.c23 = c23;
    lastDebug_.s234 = s234; lastDebug_.c234 = c234;
    lastDebug_.s5 = s5; lastDebug_.c5 = c5;
    lastDebug_.s6 = s6; lastDebug_.c6 = c6;

    dbg_gc_q2 = q2; dbg_gc_q3 = q3; dbg_gc_q4 = q4;
    dbg_gc_q5 = q5; dbg_gc_q6 = q6;
    dbg_gc_s2 = s2; dbg_gc_c2 = c2;
    dbg_gc_s23 = s23; dbg_gc_c23 = c23;
    dbg_gc_s234 = s234; dbg_gc_c234 = c234;
    dbg_gc_s5 = s5; dbg_gc_c5 = c5;
    dbg_gc_s6 = s6; dbg_gc_c6 = c6;

    // Common cross products
    const float s5c234   = s5 * c234;
    const float c6s234   = c6 * s234;
    const float s6s234   = s6 * s234;
    const float c5c6     = c5 * c6;
    const float c5s6     = c5 * s6;
    const float c5c6c234 = c5c6 * c234;
    const float c5s6c234 = c5s6 * c234;

    // tau1 (Yaw) = 0
    torques.tau_yaw = 0.0f;

    // ================================================================
    // tau2 (Pitch1) — 11 terms
    // ================================================================
    lastDebug_.p1_base =
        coeffs_.K2_c2 * c2 +
        coeffs_.K2_s2 * s2;
    lastDebug_.p1_link =
        coeffs_.K2_c23 * c23 +
        coeffs_.K2_s23 * s23;
    lastDebug_.p1_wrist =
        coeffs_.K2_c234 * c234 +
        coeffs_.K2_s234 * s234;
    lastDebug_.p1_roll =
        coeffs_.K2_s5c234 * s5c234;
    lastDebug_.p1_end =
        coeffs_.K2_c6s234 * c6s234 +
        coeffs_.K2_s6s234 * s6s234;
    lastDebug_.p1_cross =
        coeffs_.K2_c5c6c234 * c5c6c234 +
        coeffs_.K2_c5s6c234 * c5s6c234;
    lastDebug_.p1_total =
        lastDebug_.p1_base + lastDebug_.p1_link + lastDebug_.p1_wrist +
        lastDebug_.p1_roll + lastDebug_.p1_end + lastDebug_.p1_cross;
    torques.tau_pitch1 = lastDebug_.p1_total;

    // ================================================================
    // tau3 (Pitch2) — 9 terms
    // ================================================================
    lastDebug_.p2_link =
        coeffs_.K3_c23 * c23 +
        coeffs_.K3_s23 * s23;
    lastDebug_.p2_wrist =
        coeffs_.K3_c234 * c234 +
        coeffs_.K3_s234 * s234;
    lastDebug_.p2_roll =
        coeffs_.K3_s5c234 * s5c234;
    lastDebug_.p2_end =
        coeffs_.K3_c6s234 * c6s234 +
        coeffs_.K3_s6s234 * s6s234;
    lastDebug_.p2_cross =
        coeffs_.K3_c5c6c234 * c5c6c234 +
        coeffs_.K3_c5s6c234 * c5s6c234;
    lastDebug_.p2_total =
        lastDebug_.p2_link + lastDebug_.p2_wrist +
        lastDebug_.p2_roll + lastDebug_.p2_end + lastDebug_.p2_cross;
    torques.tau_pitch2 = lastDebug_.p2_total;

    // ================================================================
    // tau4 (P3) — 7 terms
    // ================================================================
    lastDebug_.p3_wrist =
        coeffs_.K4_c234 * c234 +
        coeffs_.K4_s234 * s234;
    lastDebug_.p3_roll =
        coeffs_.K4_s5c234 * s5c234;
    lastDebug_.p3_end =
        coeffs_.K4_c6s234 * c6s234 +
        coeffs_.K4_s6s234 * s6s234;
    lastDebug_.p3_cross =
        coeffs_.K4_c5c6c234 * c5c6c234 +
        coeffs_.K4_c5s6c234 * c5s6c234;
    lastDebug_.p3_total =
        lastDebug_.p3_wrist + lastDebug_.p3_roll +
        lastDebug_.p3_end + lastDebug_.p3_cross;
    torques.tau_pitch3 = lastDebug_.p3_total;

    // ================================================================
    // tau5 (Roll) — 无重力补偿
    // ================================================================
    torques.tau_roll = 0.0f;

    // ================================================================
    // tau6 (PitchEnd) — 5 terms
    // ================================================================
    const float c6c234   = c6 * c234;
    const float s6c234   = s6 * c234;
    const float c5c6s234 = c5c6 * s234;
    const float c5s6s234 = c5s6 * s234;
    const float s5s6s234 = s5 * s6 * s234;

    lastDebug_.pEnd_direct =
        coeffs_.K6_c6c234 * c6c234 +
        coeffs_.K6_s6c234 * s6c234;
    lastDebug_.pEnd_roll =
        coeffs_.K6_c5c6s234 * c5c6s234 +
        coeffs_.K6_c5s6s234 * c5s6s234;
    lastDebug_.pEnd_cross =
        coeffs_.K6_s5s6s234 * s5s6s234;
    lastDebug_.pEnd_total =
        lastDebug_.pEnd_direct + lastDebug_.pEnd_roll + lastDebug_.pEnd_cross;
    torques.tau_pitchEnd = lastDebug_.pEnd_total;

    dbg_gc_tau_pitch1_raw = torques.tau_pitch1;
    dbg_gc_tau_pitch2_raw = torques.tau_pitch2;
    dbg_gc_tau_pitch3_raw = torques.tau_pitch3;
    dbg_gc_tau_pitchEnd_raw = torques.tau_pitchEnd;

    // Apply global scale
    torques.tau_pitch1   *= scale_;
    torques.tau_pitch2   *= scale_;
    torques.tau_pitch3   *= scale_;
    torques.tau_pitchEnd *= scale_;

    dbg_gc_tau_pitch1_scaled = torques.tau_pitch1;
    dbg_gc_tau_pitch2_scaled = torques.tau_pitch2;
    dbg_gc_tau_pitch3_scaled = torques.tau_pitch3;
    dbg_gc_tau_pitchEnd_scaled = torques.tau_pitchEnd;

    return torques;
}

/**
 * @brief 计算虚拟阻尼力矩（仅PitchEnd）
 * @param rawVelocity 原始速度反馈 (rad/s)，已由调用方转换
 * @return 阻尼力矩 (N·m)，方向与速度相反
 * @note 使用滑动平均滤波 + 方向一致性检查，避免延迟导致的抖动
 */
float CAlgoGravityComp::CalculateDamping_PitchEnd(float rawVelocity) {
    if (!dampingEnabled_) {
        lastDebug_.damping_vel_raw = rawVelocity;
        lastDebug_.damping_vel_filtered = 0.0f;
        lastDebug_.damping_torque = 0.0f;
        return 0.0f;
    }

    velocityBuffer_pitchEnd_[bufferIndex_pitchEnd_] = rawVelocity;
    bufferIndex_pitchEnd_ = (bufferIndex_pitchEnd_ + 1) % VELOCITY_BUFFER_SIZE;

    float sum = 0.0f;
    for (int i = 0; i < VELOCITY_BUFFER_SIZE; ++i) {
        sum += velocityBuffer_pitchEnd_[i];
    }
    const float avgVelocity = sum / static_cast<float>(VELOCITY_BUFFER_SIZE);

    float effectiveVelocity = 0.0f;
    const bool sameDirection = (rawVelocity * avgVelocity) >= 0.0f;

    if (sameDirection && fabsf(avgVelocity) > dampingParams_.deadzone) {
        effectiveVelocity = (fabsf(rawVelocity) < fabsf(avgVelocity)) ? rawVelocity : avgVelocity;
    }

    float dampingTorque = -dampingParams_.B_pitchEnd * effectiveVelocity;
    dampingTorque = std::clamp(dampingTorque, -dampingParams_.maxTorque, dampingParams_.maxTorque);

    lastVelocity_pitchEnd_ = rawVelocity;

    lastDebug_.damping_vel_raw = rawVelocity;
    lastDebug_.damping_vel_filtered = avgVelocity;
    lastDebug_.damping_torque = dampingTorque;

    return dampingTorque;
}

float CAlgoGravityComp::CalculateDamping_Roll(float rawVelocity) {
    if (!dampingEnabled_) {
        lastDebug_.damping_roll_vel_raw = rawVelocity;
        lastDebug_.damping_roll_vel_filtered = 0.0f;
        lastDebug_.damping_roll_torque = 0.0f;
        return 0.0f;
    }

    velocityBuffer_roll_[bufferIndex_roll_] = rawVelocity;
    bufferIndex_roll_ = (bufferIndex_roll_ + 1) % VELOCITY_BUFFER_SIZE;

    float sum = 0.0f;
    for (int i = 0; i < VELOCITY_BUFFER_SIZE; ++i) {
        sum += velocityBuffer_roll_[i];
    }
    const float avgVelocity = sum / static_cast<float>(VELOCITY_BUFFER_SIZE);

    float effectiveVelocity = 0.0f;
    const bool sameDirection = (rawVelocity * avgVelocity) >= 0.0f;

    if (sameDirection && fabsf(avgVelocity) > dampingParams_.deadzone) {
        effectiveVelocity = (fabsf(rawVelocity) < fabsf(avgVelocity)) ? rawVelocity : avgVelocity;
    }

    float dampingTorque = -dampingParams_.B_roll * effectiveVelocity;
    dampingTorque = std::clamp(dampingTorque, -dampingParams_.maxTorque, dampingParams_.maxTorque);

    lastVelocity_roll_ = rawVelocity;

    lastDebug_.damping_roll_vel_raw = rawVelocity;
    lastDebug_.damping_roll_vel_filtered = avgVelocity;
    lastDebug_.damping_roll_torque = dampingTorque;

    return dampingTorque;
}

} // namespace my_engineer
