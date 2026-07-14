/**
 * @file algo_grav_comp.cpp
 * @author ciallo (1002046597@qq.com)
 * @brief 六轴机械臂重力补偿算法实现
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
EAppStatus CAlgoGravityComp::InitComponent(const SGravParam& param) {
    param_ = param;
    ramp_ = 0.0f;
    mode_ = CGravityCompMode::NONE;
    return APP_OK;
}

/**
 * @brief 设置模式
 */
void CAlgoGravityComp::SetMode(CGravityCompMode mode) {
    if (mode_ != mode) ramp_ = 0.0f;
    mode_ = mode;
}

/**
 * @brief 重置重力补偿器状态
 */
void CAlgoGravityComp::Reset() {
    mode_ = CGravityCompMode::NONE;
    ramp_ = 0.0f;
}

/**
 * @brief 检查关节角度是否在安全工作空间内
 */
bool CAlgoGravityComp::IsStateValid_(const SGravState& state) const {
    return state.pitch1_deg >= param_.ws_pitch1_min && state.pitch1_deg <= param_.ws_pitch1_max
        && state.pitch2_deg >= param_.ws_pitch2_min && state.pitch2_deg <= param_.ws_pitch2_max
        && state.roll_deg >= param_.ws_roll_min && state.roll_deg <= param_.ws_roll_max;
}

/**
 * @brief 计算重力补偿前馈量
 */
SGravOutput CAlgoGravityComp::Calc(const SGravState& state) {
    SGravOutput out;

    const bool stateValid = IsStateValid_(state);

    // 超出安全工作空间则强制ramp下降，防止力矩突变
    if (mode_ != CGravityCompMode::NONE && stateValid) {
        ramp_ = std::clamp(ramp_ + param_.ramp_alpha, 0.0f, 1.0f);
    } else {
        ramp_ = std::clamp(ramp_ - param_.ramp_alpha, 0.0f, 1.0f);
    }

    if (ramp_ <= 0.0f) return out;

    // 角度转rad，减去各自零点偏移
    // 关节链: q2(pitch1) -> q3(pitch2) -> q5(roll) -> q6(end_roll) -> q7(end_pitch)
    const float_t q2 = deg2rad(state.pitch1_deg - param_.pitch1_zero_deg);
    const float_t q3 = deg2rad(state.pitch2_deg);
    const float_t q5 = deg2rad(state.roll_deg);
    const float_t q6 = deg2rad(state.end_roll_deg);
    const float_t q7 = deg2rad(state.end_pitch_deg);

    // 各关节 sin/cos
    const float_t s2 = sinf(q2), c2 = cosf(q2);
    const float_t s3 = sinf(q3), c3 = cosf(q3);
    const float_t s5 = sinf(q5), c5 = cosf(q5);
    const float_t s6 = sinf(q6), c6 = cosf(q6);
    const float_t s7 = sinf(q7), c7 = cosf(q7);

    // 累计角 sin/cos
    const float_t s23  = s2*c3 + c2*s3,   c23  = c2*c3 - s2*s3;
    const float_t s56  = s5*c6 + c5*s6,   c56  = c5*c6 - s5*s6;
    const float_t s67  = s6*c7 + c6*s7,   c67  = c6*c7 - s6*s7;
    const float_t s235 = s23*c5 + c23*s5, c235 = c23*c5 - s23*s5;
    const float_t s567 = s56*c7 + c56*s7, c567 = c56*c7 - s56*s7;
    const float_t s2356 = s235*c6 + c235*s6, c2356 = c235*c6 - s235*s6;
    const float_t s23567 = s2356*c7 + c2356*s7, c23567 = c2356*c7 - s2356*s7;

    // roll ：sin(q3-q2), cos(q3-q2)
    const float_t sr = s3*c2 - c3*s2;                   ///< sin(q3-q2)
    const float_t cr = c3*c2 + s3*s2;                   ///< cos(q3-q2)
    const float_t a7 = s7;                              ///< sin(q7)

    // 每关节独立拟合
    const auto& pc1 = param_.pitch1_tau_coeff;
    const float_t tau_p1 =
        pc1[0]
        + pc1[1] * s2  + pc1[2] * c2
        + pc1[3] * s23 + pc1[4] * c23
        + pc1[5] * s235 + pc1[6] * c235
        + pc1[7] * s2356 + pc1[8] * c2356
        + pc1[9] * s23567 + pc1[10] * c23567;

    const auto& pc2 = param_.pitch2_tau_coeff;
    const float_t tau_p2 =
        pc2[0]
        + pc2[1] * s23 + pc2[2] * c23;

    const auto& rc = param_.roll_tau_coeff;
    const float_t tau_rl =
        rc[0]
        + rc[1] * s5
        + rc[2] * c5
        + rc[3]  * a7 * sr * s5
        + rc[4]  * a7 * cr * s5
        + rc[5]  * a7 * sr * c5
        + rc[6]  * a7 * cr * c5
        + rc[7]  * c7 * sr * s5
        + rc[8]  * c7 * cr * s5
        + rc[9]  * c7 * sr * c5
        + rc[10] * c7 * cr * c5;

    const auto& er = param_.end_roll_tau_coeff;
    const float_t tau_er =
        er[0]
        + er[1] * s6 + er[2] * c6
        + er[3] * s67 + er[4] * c67;

    const auto& ep = param_.end_pitch_tau_coeff;
    const float_t tau_ep =
        ep[0]
        + ep[1] * s7 + ep[2] * c7
        + ep[3] * c5 * c7
        + ep[4] * cr * s7
        + ep[5] * s5 * c7;

    out.pitch1_tau = tau_p1;
    out.pitch2_tau = tau_p2;
    out.roll_tau = tau_rl;
    out.end_roll_tau = tau_er;
    out.end_pitch_tau = tau_ep;

    out.pitch1_current_ff = std::clamp(
        param_.pitch1_motor.JointTorqueToCommand(tau_p1) * ramp_,
        -param_.pitch1_ff_limit, param_.pitch1_ff_limit);

    out.pitch2_current_ff = std::clamp(
        param_.pitch2_motor.JointTorqueToCommand(tau_p2) * ramp_,
        -param_.pitch2_ff_limit, param_.pitch2_ff_limit);

    out.roll_tau_ff = std::clamp(
        param_.roll_motor.JointTorqueToCommand(tau_rl) * ramp_,
        -param_.roll_tau_limit, param_.roll_tau_limit);

    out.end_roll_tau_ff = std::clamp(
        param_.end_roll_motor.JointTorqueToCommand(tau_er) * ramp_,
        -param_.end_roll_tau_limit, param_.end_roll_tau_limit);

    out.end_pitch_tau_ff = std::clamp(
        param_.end_pitch_motor.JointTorqueToCommand(tau_ep) * ramp_,
        -param_.end_pitch_tau_limit, param_.end_pitch_tau_limit);

    return out;
}

} // namespace my_engineer
