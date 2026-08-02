/**
 * @file algo_grav_comp.cpp
 * @author ciallo (1002046597@qq.com)
 * @brief 七轴机械臂重力补偿算法实现
 * @version 1.0
 * @date 2026-06-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "algo_grav_comp.hpp"
#include <algorithm>
#include <cmath>

#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)

namespace my_engineer {

/**
 * @brief 将关节侧重力力矩转换为对应电机指令
 */
float_t SMotorConversion::JointTorqueToCommand(float_t tau_Nm) const {
    const float_t directedTau = tau_Nm * direction;

    switch (type) {
        case EMotorType::MG8010_I36V2:
        case EMotorType::MG6012_I36V3:
        case EMotorType::MG5010_I36V3: {
            const float_t rawPerAmp =
                (amp_to_raw > 0.0f) ? amp_to_raw : GRAV_KT_MOTOR_RAW_PER_AMP;// 如果手动配置了转换的电流和A的转换系数，否则就使用默认的raw/A转换系数
            const float_t denom = kt * reduction;
            if (denom == 0.0f) return 0.0f;
            return directedTau / denom * rawPerAmp;
        }
        case EMotorType::DM_MIT:
            if (reduction == 0.0f) return 0.0f;
            return directedTau / reduction;
        default:
            return 0.0f;
    }
}

/**
 * @brief 将电机指令反算为关节侧重力力矩
 */
float_t SMotorConversion::CommandToJointTorque(float_t command) const {
    switch (type) {
        case EMotorType::MG8010_I36V2:
        case EMotorType::MG6012_I36V3:
        case EMotorType::MG5010_I36V3: {
            const float_t rawPerAmp =
                (amp_to_raw > 0.0f) ? amp_to_raw : GRAV_KT_MOTOR_RAW_PER_AMP;
            if (rawPerAmp == 0.0f) return 0.0f;
            return command / rawPerAmp * kt * reduction * direction;
        }
        case EMotorType::DM_MIT:
            return command * reduction * direction;
        default:
            return 0.0f;
    }
}

/**
 * @brief 初始化重力补偿器
 */
EAppStatus CAlgoArmGravityComp::Init(const SArmGravityParam& param) {
    param_ = param;
    ramp_ = 0.0f;
    enabled_ = false;
    return APP_OK;
}

/**
 * @brief 启停重力补偿
 */
void CAlgoArmGravityComp::SetEnable(bool enable) {
    enabled_ = enable;
}

/**
 * @brief 设置观察模式
 */
void CAlgoArmGravityComp::SetObserveMode(bool observe) {
    observe_ = observe;
}

/**
 * @brief 重置重力补偿器状态
 */
void CAlgoArmGravityComp::Reset() {
    enabled_ = false;
    ramp_ = 0.0f;
}

/**
 * @brief 检查关节角度是否在安全工作空间内
 */
bool CAlgoArmGravityComp::IsStateValid_(const SArmGravityState& state) const {
    return state.pitch1_deg >= param_.ws_pitch1_min && state.pitch1_deg <= param_.ws_pitch1_max
        && state.pitch2_deg >= param_.ws_pitch2_min && state.pitch2_deg <= param_.ws_pitch2_max
        && state.pitch3_deg >= param_.ws_pitch3_min && state.pitch3_deg <= param_.ws_pitch3_max
        && state.roll_deg >= param_.ws_roll_min && state.roll_deg <= param_.ws_roll_max;
}

/**
 * @brief 计算重力补偿前馈量
 */
SArmGravityOutput CAlgoArmGravityComp::Calc(const SArmGravityState& state) {
    SArmGravityOutput out;

    const bool stateValid = IsStateValid_(state);

    // 超出安全工作空间则强制ramp下降，防止力矩突变
    if (enabled_ && stateValid) {
        ramp_ = std::clamp(ramp_ + param_.ramp_alpha, 0.0f, 1.0f);
    } else {
        ramp_ = std::clamp(ramp_ - param_.ramp_alpha, 0.0f, 1.0f);
    }

    if (ramp_ <= 0.0f) return out;

    // 角度转rad，q2~q7对应符号推导中的关节编号
    const float_t q2 = deg2rad(state.pitch1_deg - param_.pitch1_zero_deg);
    const float_t q3 = deg2rad(state.pitch2_deg - param_.pitch2_zero_deg);
    const float_t q4 = deg2rad(state.pitch3_deg - param_.pitch3_zero_deg);
    const float_t q5 = deg2rad(state.roll_deg);
    const float_t q6 = deg2rad(state.end_roll_deg);
    const float_t q7 = deg2rad(state.end_pitch_deg);

    // 各关节 sin/cos
    const float_t s2 = sinf(q2), c2 = cosf(q2);
    const float_t s3 = sinf(q3), c3 = cosf(q3);
    const float_t s4 = sinf(q4), c4 = cosf(q4);
    const float_t s5 = sinf(q5), c5 = cosf(q5);
    const float_t s6 = sinf(q6), c6 = cosf(q6);
    const float_t s7 = sinf(q7), c7 = cosf(q7);

    // 组合角 sin/cos 
    const float_t s23  = s2*c3 + c2*s3,  c23  = c2*c3 - s2*s3;
    const float_t s234 = s23*c4 + c23*s4, c234 = c23*c4 - s23*s4;
    const float_t s2345 = s234*c5 + c234*s5, c2345 = c234*c5 - s234*s5;
    const float_t s56  = s5*c6 + c5*s6,  c56  = c5*c6 - s5*s6;
    const float_t s23456 = s2345*c6 + c2345*s6, c23456 = c2345*c6 - s2345*s6;
    const float_t s567  = s56*c7 + c56*s7,   c567  = c56*c7 - s56*s7;
    const float_t s234567 = s23456*c7 + c23456*s7, c234567 = c23456*c7 - s23456*s7;

    // 每关节独立拟合
    const auto& pc1 = param_.pitch1_tau_coeff;
    const float_t tau_p1 =
        pc1[0]
        + pc1[1] * s2  + pc1[2] * c2
        + pc1[3] * s23 + pc1[4] * c23
        + pc1[5] * s234 + pc1[6] * c234
        + pc1[7] * s2345 + pc1[8] * c2345
        + pc1[9] * s23456 + pc1[10] * c23456
        + pc1[11] * s234567 + pc1[12] * c234567;

    const auto& pc2 = param_.pitch2_tau_coeff;
    const float_t tau_p2 =
        pc2[0]
        + pc2[1] * s23 + pc2[2] * c23
        + pc2[3] * s234 + pc2[4] * c234
        + pc2[5] * s2345 + pc2[6] * c2345
        + pc2[7] * s23456 + pc2[8] * c23456
        + pc2[9] * s234567 + pc2[10] * c234567;

    const auto& pc3 = param_.pitch3_tau_coeff;
    const float_t tau_p3 =
        pc3[0]
        + pc3[1] * s234 + pc3[2] * c234
        + pc3[3] * s2345 + pc3[4] * c2345
        + pc3[5] * s23456 + pc3[6] * c23456
        + pc3[7] * s234567 + pc3[8] * c234567;

    const auto& rc = param_.roll_tau_coeff;
    const float_t tau_rl =
        rc[0]
        + rc[1] * s5  + rc[2] * c5
        + rc[3] * s56 + rc[4] * c56
        + rc[5] * s567 + rc[6] * c567
        + rc[7] * s2345 + rc[8] * c2345;

    out.pitch1_tau = tau_p1;
    out.pitch2_tau = tau_p2;
    out.pitch3_tau = tau_p3;
    out.roll_tau = tau_rl;

    out.pitch1_current_ff = std::clamp(
        param_.pitch1_motor.JointTorqueToCommand(tau_p1) * ramp_,
        -param_.pitch1_ff_limit, param_.pitch1_ff_limit);

    out.pitch2_current_ff = std::clamp(
        param_.pitch2_motor.JointTorqueToCommand(tau_p2) * ramp_,
        -param_.pitch2_ff_limit, param_.pitch2_ff_limit);

    out.roll_tau_ff = std::clamp(
        param_.roll_motor.JointTorqueToCommand(tau_rl) * ramp_,
        -param_.roll_tau_limit, param_.roll_tau_limit);

    out.pitch3_current_ff = std::clamp(
        param_.pitch3_motor.JointTorqueToCommand(tau_p3) * ramp_,
        -param_.pitch3_ff_limit, param_.pitch3_ff_limit);

    return out;
}

} // namespace my_engineer
