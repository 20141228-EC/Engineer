/**
 * @file algo_gravity_comp.hpp
 * @brief 示教器重力补偿/虚拟阻尼算法 (6轴版)
 * @author Ciallo
 * @date 2026-01-20
 * @note 2026-04-18: 6轴重构
 *
 * DH关节映射: q1=Yaw, q2=Pitch1, q3=Pitch2, q4=P3, q5=Roll, q6=PitchEnd
 * 累积角: q23 = q2+q3, q234 = q2+q3+q4
 * Roll(q5)不参与重力补偿力矩输出, 但角度参与DH链计算
 */

#ifndef ALGO_GRAVITY_COMP_HPP
#define ALGO_GRAVITY_COMP_HPP

#include "Configuration.hpp"

#include <algorithm>
#include <cmath>

namespace my_engineer {

class CAlgoGravityComp {
public:
    // DH offsets are handled by the caller.
    static constexpr float DH_OFFSET_PITCH1   = 0.0f;
    static constexpr float DH_OFFSET_PITCH2   = 0.0f;
    static constexpr float DH_OFFSET_PITCH3   = 0.0f;
    static constexpr float DH_OFFSET_ROLL     = 0.0f;
    static constexpr float DH_OFFSET_PITCHEND = 0.0f;

    // 重力参数辨识
    // q23  = q2 + q3
    // q234 = q2 + q3 + q4
    // q5 = Roll, q6 = PitchEnd
    struct SGravCoeffs {
        // --- tau2 (Pitch1): 2026-05-01 ---
        float K2_c2       = -3.6082996410f;
        float K2_s2       = +15.9327075389f;
        float K2_c23      = +1.6263201073f;
        float K2_s23      = -2.3271380224f;
        float K2_c234     = +2.0781132049f;
        float K2_s234     = -6.1506700961f;
        float K2_s5c234   =  0.0f;
        float K2_c6s234   =  0.0f;
        float K2_s6s234   =  0.0f;
        float K2_c5c6c234 =  0.0f;
        float K2_c5s6c234 =  0.0f;

        // --- tau3 (Pitch2): 2026-05-01 ---
        float K3_c23      = -1.1979024907f;
        float K3_s23      = +4.6158350837f;
        float K3_c234     = -1.8418685078f;
        float K3_s234     = +1.2213724081f;
        float K3_s5c234   =  0.0f;
        float K3_c6s234   =  0.0f;
        float K3_s6s234   =  0.0f;
        float K3_c5c6c234 =  0.0f;
        float K3_c5s6c234 =  0.0f;


        // --- tau4 (P3): 2 base terms, 83-pt fit 2026-05-01, R^2=0.260 ---
        float K4_c234     = -0.1007196729f;
        float K4_s234     = +3.1042652063f;
        float K4_s5c234   =  0.0f;
        float K4_c6s234   =  0.0f;
        float K4_s6s234   =  0.0f;
        float K4_c5c6c234 =  0.0f;
        float K4_c5s6c234 =  0.0f;

        // --- tau6 (PitchEnd): 5 terms ---
        float K6_c6c234   = 0.17945227971f;
        float K6_s6c234   = -0.07545484125f;
        float K6_c5c6s234 = -0.07545484125f;
        float K6_c5s6s234 = -0.17945227971f;
        float K6_s5s6s234 = 0.07545484125f;
    };

    struct SDampingParams {
        float B_pitch1   = 0.0f;
        float B_pitch2   = 0.0f;
        float B_pitch3   = 0.0f;
        float B_roll     = 0.007f;
        float B_pitchEnd = 0.0065f;

        float deadzone   = 0.2f;
        float maxTorque  = 0.18f;
    };

    struct SGravTorques {
        float tau_yaw      = 0.0f;
        float tau_pitch1   = 0.0f;
        float tau_pitch2   = 0.0f;
        float tau_pitch3   = 0.0f;
        float tau_roll     = 0.0f;  // Roll无重力补偿，保留字段用于阻尼叠加
        float tau_pitchEnd = 0.0f;
    };

    struct SGravDebug {
        // Input DH angles
        float q2 = 0.0f, q3 = 0.0f, q4 = 0.0f, q5 = 0.0f, q6 = 0.0f;

        // Precomputed trig
        float s2 = 0.0f, c2 = 0.0f;
        float s23 = 0.0f, c23 = 0.0f;
        float s234 = 0.0f, c234 = 0.0f;
        float s5 = 0.0f, c5 = 0.0f;
        float s6 = 0.0f, c6 = 0.0f;

        // Pitch1 sub-totals
        float p1_base = 0.0f;
        float p1_link = 0.0f;
        float p1_wrist = 0.0f;
        float p1_roll = 0.0f;
        float p1_end = 0.0f;
        float p1_cross = 0.0f;
        float p1_total = 0.0f;

        // Pitch2 sub-totals
        float p2_link = 0.0f;
        float p2_wrist = 0.0f;
        float p2_roll = 0.0f;
        float p2_end = 0.0f;
        float p2_cross = 0.0f;
        float p2_total = 0.0f;

        // P3 sub-totals
        float p3_wrist = 0.0f;
        float p3_roll = 0.0f;
        float p3_end = 0.0f;
        float p3_cross = 0.0f;
        float p3_total = 0.0f;

        // PitchEnd sub-totals
        float pEnd_direct = 0.0f;
        float pEnd_roll = 0.0f;
        float pEnd_cross = 0.0f;
        float pEnd_total = 0.0f;

        // Damping
        float damping_vel_raw = 0.0f;
        float damping_vel_filtered = 0.0f;
        float damping_torque = 0.0f;

        float damping_roll_vel_raw = 0.0f;
        float damping_roll_vel_filtered = 0.0f;
        float damping_roll_torque = 0.0f;
    };

    volatile SGravDebug lastDebug_;

    CAlgoGravityComp() = default;

    void Init() {
        coeffs_ = SGravCoeffs{};
        dampingParams_ = SDampingParams{};
        enabled_ = true;
        dampingEnabled_ = false;
        scale_ = 1.0f;
        ResetDampingFilters();
    }

    void Init(const SGravCoeffs& coeffs) {
        coeffs_ = coeffs;
        dampingParams_ = SDampingParams{};
        enabled_ = true;
        dampingEnabled_ = false;
        scale_ = 1.0f;
        ResetDampingFilters();
    }

    /// 计算补偿力矩，输入编码器角度(rad)
    SGravTorques Calculate(float enc_pitch1, float enc_pitch2,
                           float enc_pitch3, float enc_roll, float enc_pitchEnd);

    /// 计算补偿力矩，输入DH角度(rad)
    SGravTorques CalculateDH(float q2, float q3, float q4, float q5, float q6);

    float CalculateDamping_PitchEnd(float rawVelocity);

    /**
     * @brief 计算虚拟阻尼力矩（Roll）
     * @param rawVelocity 原始速度反馈 (rad/s)
     * @return 阻尼力矩 (N·m)
     * @note 内部自动进行低通滤波和死区处理
     */
    float CalculateDamping_Roll(float rawVelocity);

    ///< 设置/获取补偿使能
    void SetEnabled(bool enabled) { enabled_ = enabled; }
    bool IsEnabled() const { return enabled_; }

    ///< 设置/获取虚拟阻尼使能
    void SetDampingEnabled(bool enabled) { dampingEnabled_ = enabled; }
    bool IsDampingEnabled() const { return dampingEnabled_; }

    ///< 设置/获取补偿比例(0.0~2.0)
    void SetScale(float scale) { scale_ = std::clamp(scale, 0.0f, 2.0f); }
    float GetScale() const { return scale_; }

    ///< 更新/获取补偿系数
    void UpdateCoeffs(const SGravCoeffs& coeffs) { coeffs_ = coeffs; }
    const SGravCoeffs& GetCoeffs() const { return coeffs_; }

    ///< 更新/获取虚拟阻尼参数
    void UpdateDampingParams(const SDampingParams& params) { dampingParams_ = params; }
    const SDampingParams& GetDampingParams() const { return dampingParams_; }

    ///< 重置滤波器状态（模式切换时调用）
    void ResetDampingFilters() {
        for (int i = 0; i < VELOCITY_BUFFER_SIZE; ++i) {
            velocityBuffer_pitchEnd_[i] = 0.0f;
            velocityBuffer_roll_[i] = 0.0f;
        }
        bufferIndex_pitchEnd_ = 0;
        bufferIndex_roll_ = 0;
        lastVelocity_pitchEnd_ = 0.0f;
        lastVelocity_roll_ = 0.0f;
    }

private:
    SGravCoeffs coeffs_;
    SDampingParams dampingParams_;
    bool enabled_ = true;
    bool dampingEnabled_ = true;
    float scale_ = 1.0f;

    static constexpr int VELOCITY_BUFFER_SIZE = 4;

    float velocityBuffer_pitchEnd_[VELOCITY_BUFFER_SIZE] = {0};
    int bufferIndex_pitchEnd_ = 0;
    float lastVelocity_pitchEnd_ = 0.0f;

    float velocityBuffer_roll_[VELOCITY_BUFFER_SIZE] = {0};
    int bufferIndex_roll_ = 0;
    float lastVelocity_roll_ = 0.0f;
};

} // namespace my_engineer

#endif // ALGO_GRAVITY_COMP_HPP
