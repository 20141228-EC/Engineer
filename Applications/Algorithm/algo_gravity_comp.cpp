/**
 * @file algo_gravity_comp.cpp
 * @brief 示教器重力补偿算法实现
 * @author Ciallo
 * @date 2026-06-25
 */

#include "algo_gravity_comp.hpp"

namespace my_engineer {

///< 度转弧度
static inline float deg2rad(float deg) { return deg * 0.017453292519943295769f; }

/**
 * @brief 关节力矩 -> 电机指令
 * DM_MIT: 力矩经减速比换算到电机侧
 * DJI_CURRENT: 力矩 -> 电流 -> raw
 */
float SMotorConversion::JointTorqueToCommand(float tau_Nm) const {
    const float directedTau = tau_Nm * direction;

    switch (type) {
        case EMotorType::DM_MIT: {
            if (reduction == 0.0f) return 0.0f;
            return directedTau / reduction;  ///< 关节力矩除以减速比 = 电机侧力矩
        }
        case EMotorType::DJI_CURRENT: {
            const float rawPerAmp =
                (amp_to_raw > 0.0f) ? amp_to_raw : GRAV_KT_MOTOR_RAW_PER_AMP;
            const float denom = kt * reduction;
            if (denom == 0.0f) return 0.0f;
            return directedTau / denom * rawPerAmp;
        }
        case EMotorType::CUSTOM:
        default:
            return 0.0f;
    }
}

/**
 * @brief 电机指令 -> 关节力矩 (反算, 辨识用)
 */
float SMotorConversion::CommandToJointTorque(float command) const {
    switch (type) {
        case EMotorType::DM_MIT: {
            return command * reduction * direction;
        }
        case EMotorType::DJI_CURRENT: {
            const float rawPerAmp =
                (amp_to_raw > 0.0f) ? amp_to_raw : GRAV_KT_MOTOR_RAW_PER_AMP;
            if (rawPerAmp == 0.0f) return 0.0f;
            return command / rawPerAmp * kt * reduction * direction;
        }
        case EMotorType::CUSTOM:
        default:
            return 0.0f;
    }
}

EAppStatus CAlgoGravityComp::InitComponent(const SGravParam& param) {
    param_ = param;
    ramp_ = 0.0f;
    mode_ = CGravityCompMode::NONE;
    return APP_OK;
}

void CAlgoGravityComp::Reset() {
    mode_ = CGravityCompMode::NONE;
    ramp_ = 0.0f;
}

bool CAlgoGravityComp::IsStateValid_(const SGravState& state) const {
    return state.pitch1_deg >= param_.ws_pitch1_min && state.pitch1_deg <= param_.ws_pitch1_max
        && state.pitch2_deg >= param_.ws_pitch2_min && state.pitch2_deg <= param_.ws_pitch2_max
        && state.roll_deg >= param_.ws_roll_min && state.roll_deg <= param_.ws_roll_max;
}

/**
 * @brief 计算重力补偿前馈
 */
SGravOutput CAlgoGravityComp::Calc(const SGravState& state) {
    SGravOutput out;

    // ramp 渐变: 非 NONE 模式且在工作空间内才上升, 否则下降
    const bool stateValid = IsStateValid_(state);
    if (mode_ != CGravityCompMode::NONE && stateValid) {
        ramp_ = std::clamp(ramp_ + param_.ramp_alpha, 0.0f, 1.0f);
    } else {
        ramp_ = std::clamp(ramp_ - param_.ramp_alpha, 0.0f, 1.0f);
    }
    if (ramp_ <= 0.0f) return out;  ///< ramp 归零直接返回空输出

    // 角度转 rad 并减零点偏移
    const float q2 = deg2rad(state.pitch1_deg - param_.pitch1_zero_deg);
    const float q3 = deg2rad(state.pitch2_deg);  ///< P2 用相对角, 不减零点
    const float q5 = deg2rad(state.roll_deg - param_.roll_zero_deg);
    const float q6 = deg2rad(state.pitch_end_deg - param_.pitch_end_zero_deg);

    // 单关节 sin/cos
    const float s2 = sinf(q2), c2 = cosf(q2);
    const float s3 = sinf(q3), c3 = cosf(q3);
    const float s5 = sinf(q5), c5 = cosf(q5);
    const float s6 = sinf(q6), c6 = cosf(q6);

    // 累积角 sin/cos 
    const float s23 = s2 * c3 + c2 * s3, c23 = c2 * c3 - s2 * s3;
    const float s56 = s5 * c6 + c5 * s6, c56 = c5 * c6 - s5 * s6;

    // P2 相对 P1 姿态调制项: sin/cos(q3-q2)
    const float sr = s3 * c2 - c3 * s2;  ///< sin(q3-q2)
    const float cr = c3 * c2 + s3 * s2;  ///< cos(q3-q2)

    // ---- q3 二阶项 ----
    const float s2q3 = sinf(2.0f * q3);
    const float c2q3 = cosf(2.0f * q3);

    // ---- tau_p1: 1, s2, c2, s3, c3, s2s3, s2c3, c2s3, c2c3, sin(2q3), cos(2q3), s56, c56 ----
    const auto& p1 = param_.pitch1_coeff;
    const float tau_p1 =
        p1[0]
        + p1[1] * s2 + p1[2] * c2
        + p1[3] * s3 + p1[4] * c3
        + p1[5] * s2 * s3 + p1[6] * s2 * c3 + p1[7] * c2 * s3 + p1[8] * c2 * c3
        + p1[9] * s2q3 + p1[10] * c2q3
        + p1[11] * s56 + p1[12] * c56;

    // ---- tau_p2: 1, s2, c2, s3, c3, s2s3, s2c3, c2s3, c2c3, sin(2q3), cos(2q3), s56, c56 ----
    const auto& p2 = param_.pitch2_coeff;
    const float tau_p2 =
        p2[0]
        + p2[1] * s2 + p2[2] * c2
        + p2[3] * s3 + p2[4] * c3
        + p2[5] * s2 * s3 + p2[6] * s2 * c3 + p2[7] * c2 * s3 + p2[8] * c2 * c3
        + p2[9] * s2q3 + p2[10] * c2q3
        + p2[11] * s56 + p2[12] * c56;

    // ---- tau_roll: 1, s5, c5, s6*c5, c6*s5 ----
    const auto& rl = param_.roll_coeff;
    const float tau_roll =
        rl[0]
        + rl[1] * s5 + rl[2] * c5
        + rl[3] * s6 * c5
        + rl[4] * c6 * s5;

    // ---- tau_pe: 1, s6, c6, c5*c6, cr*s6, s5*c6 ----
    const auto& pe = param_.pitch_end_coeff;
    const float tau_pe =
        pe[0]
        + pe[1] * s6      + pe[2] * c6
        + pe[3] * c5 * c6
        + pe[4] * cr * s6
        + pe[5] * s5 * c6;

    // 关节侧重力矩
    out.pitch1_tau = tau_p1;
    out.pitch2_tau = tau_p2;
    out.roll_tau = tau_roll;
    out.pitch_end_tau = tau_pe;

    // p2 软边界 pushback
    const float over_p2 = state.pitch2_deg - param_.ws_pitch2_soft_max;
    const float p2_pushback = (over_p2 > 0.0f)
        ? param_.pitch2_pushback_gain * over_p2
        : 0.0f;
    if (over_p2 > 0.0f) {
        out.pitch2_tau -= p2_pushback;
    }

    // roll 轴线束补偿
    const float roll_pushback = 0.0f;

    // 观察模式: 只算力矩不输出前馈指令
    if (mode_ == CGravityCompMode::OBSERVE) return out;

    // 电机指令转换
    float ff_p1 = tau_p1;
    float ff_p2 = tau_p2 - p2_pushback;
    float ff_roll = tau_roll - roll_pushback;
    float ff_pe = tau_pe;
    out.pitch1_ff = std::clamp(
        param_.pitch1_motor.JointTorqueToCommand(ff_p1) * ramp_,
        -param_.pitch1_ff_limit, param_.pitch1_ff_limit);
    out.pitch2_ff = std::clamp(
        param_.pitch2_motor.JointTorqueToCommand(ff_p2) * ramp_,
        -param_.pitch2_ff_limit, param_.pitch2_ff_limit);
    out.roll_ff = std::clamp(
        param_.roll_motor.JointTorqueToCommand(ff_roll) * ramp_,
        -param_.roll_ff_limit, param_.roll_ff_limit);
    out.pitch_end_ff = std::clamp(
        param_.pitch_end_motor.JointTorqueToCommand(ff_pe) * ramp_,
        -param_.pitch_end_ff_limit, param_.pitch_end_ff_limit);

    return out;
}

} // namespace my_engineer
