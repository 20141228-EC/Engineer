/**
 * @file algo_gravity_comp.hpp
 * @brief 示教器重力补偿 + 虚拟阻尼算法
 * @author Ciallo
 * @date 2026-01-20
 * @note 2026-02-08: 新增虚拟阻尼功能
 */

#ifndef ALGO_GRAVITY_COMP_HPP
#define ALGO_GRAVITY_COMP_HPP

#include "Configuration.hpp"
#include <cmath>
#include <algorithm>

namespace my_engineer {

/**
 * @brief 重力补偿  虚拟阻尼算法类
 */
class CAlgoGravityComp {
public:
    // DH偏移常量 (已移至mod_controller.hpp的GRAV_COMP_OFFSET统一管理)
    // 这里全部设为0，偏移由调用方处理
    static constexpr float DH_OFFSET_PITCH1   = 0.0f;
    static constexpr float DH_OFFSET_PITCH2   = 0.0f;
    static constexpr float DH_OFFSET_ROLL     = 0.0f;
    static constexpr float DH_OFFSET_PITCHEND = 0.0f;

    // 重力补偿系数：基于MATLAB符号推导
    // τ2 = K2_c2*c2 + K2_c23*c23 + K2_s23*s23 + K2_end*(c5*c23*c4 - s5*s23)
    // τ3 = K3_c23*c23 + K3_s23*s23 + K3_end*(c5*c23*c4 - s5*s23)
    // τ5 = K5_1*(c5*c23 - s5*s23*c4)
    struct SGravCoeffs {
        // Pitch1 系数 (修正后)
        float K2_c2  = 1.758f;      // cos(q2)   Link1自身重力矩
        float K2_c23 = 0.6f;      // cos(q2+q3)  Link2对P1的影响
        float K2_s23 = 0.0f;        // sin(q2+q3)
        float K2_end = -0.095f;    // 末端对P1的耦合

        // Pitch2 系数 (修正后)
        float K3_c23 = 1.0f;     // cos(q2+q3)  质心沿着连杆轴线方向的分量
        float K3_s23 = 0.55f;    // sin(q2+q3)  增大以补偿q23≈70-80°区间的力矩不足
        float K3_end = -0.095f;    // 末端对P2的耦合

        // PitchEnd 系数 (修正后)
        float K5_1 = 0.0985f;         // 实测校准的末端重力矩系数，已包含机械结构和安装位置的综合影响
    };

    /**
     * @brief 虚拟阻尼参数结构体
     * @note 阻尼力矩 τ_damping = -B × filtered_velocity
     */
    struct SDampingParams {
        float B_pitch1   = 0.0f;    // Pitch1 阻尼系数 (N·m·s/rad)
        float B_pitch2   = 0.0f;    // Pitch2 阻尼系数
        float B_roll     = 0.007f;  // Roll 阻尼系数（DM3510，与PitchEnd相同）
        float B_pitchEnd = 0.0065f;  // PitchEnd 阻尼系数     （DM3510阻尼系数最大不超过0.007否则震荡）

        float deadzone   = 0.2f;    // 速度死区 (rad/s)
        float maxTorque  = 0.18f;   // 阻尼力矩上限 (N·m)，防止过大
    };

    // 输出力矩 (N·m) - 包含重力补偿 + 虚拟阻尼
    struct SGravTorques {
        float tau_yaw = 0.0f;
        float tau_pitch1 = 0.0f;
        float tau_pitch2 = 0.0f;
        float tau_roll = 0.0f;
        float tau_pitchEnd = 0.0f;
    };

    // 调试输出：力矩分量分解
    struct SGravDebug {
        // 输入角度 (rad)
        float q2 = 0.0f, q3 = 0.0f, q4 = 0.0f, q5 = 0.0f;
        // 三角函数值
        float c2 = 0.0f, c23 = 0.0f, s23 = 0.0f, c4 = 0.0f, c5 = 0.0f, s5 = 0.0f;
        // P1 各分量 (原始值，未乘scale_)
        float p1_self = 0.0f;     // K2_c2*c2 (Link1自身重力矩)
        float p1_link = 0.0f;     // K2_c23*c23 (Link2对P1的影响)
        float p1_end = 0.0f;      // K2_end*(c5*c23*c4 - s5*s23)
        float p1_total = 0.0f;    // 总和
        // P2 各分量 (原始值，未乘scale_)
        float p2_link = 0.0f;     // K3_c23*c23 + K3_s23*s23
        float p2_end = 0.0f;      // K3_end*(c5*c23*c4 - s5*s23)
        float p2_total = 0.0f;    // 总和
        // PitchEnd 分量 (原始值，未乘scale_)
        float pEnd_grav = 0.0f;   // K5_1*c5*c23 (纯重力方向分量)
        float pEnd_roll = 0.0f;   // K5_1*(-s5*s23*c4) (Roll耦合分量)
        float pEnd_total = 0.0f;  // 总和
        // 虚拟阻尼调试信息 - PitchEnd
        float damping_vel_raw = 0.0f;      // 原始速度 (rad/s)
        float damping_vel_filtered = 0.0f; // 滤波后速度
        float damping_torque = 0.0f;       // 阻尼力矩
        // 虚拟阻尼调试信息 - Roll
        float damping_roll_vel_raw = 0.0f;
        float damping_roll_vel_filtered = 0.0f;
        float damping_roll_torque = 0.0f;
    };

    /// 最近一次计算的调试信息（可通过调试器查看）
    SGravDebug lastDebug_;

    CAlgoGravityComp() = default;

    ///< 初始化补偿算法，使用默认系数
    void Init() {
        coeffs_ = SGravCoeffs{};
        dampingParams_ = SDampingParams{};
        enabled_ = true;
        dampingEnabled_ = false;  // 禁用虚拟阻尼补偿（保留计算代码用于调试）
        scale_ = 1.0f;
        // 重置滤波器状态
        ResetDampingFilters();
    }

    ///< 初始化补偿算法，传入自定义系数
    void Init(const SGravCoeffs& coeffs) {
        coeffs_ = coeffs;
        dampingParams_ = SDampingParams{};
        enabled_ = true;
        dampingEnabled_ = false;  // 禁用虚拟阻尼补偿（保留计算代码用于调试）
        scale_ = 1.0f;
        // 重置滤波器状态
        ResetDampingFilters();
    }

    ///< 计算补偿力矩，输入编码器角度(rad)
    SGravTorques Calculate(float enc_pitch1, float enc_pitch2,
                           float enc_roll, float enc_pitchEnd);

    ///< 计算补偿力矩，输入DH角度(rad)
    SGravTorques CalculateDH(float q2, float q3, float q4, float q5);

    /**
     * @brief 计算虚拟阻尼力矩（PitchEnd）
     * @param rawVelocity 原始速度反馈 (rad/s)
     * @return 阻尼力矩 (N·m)
     * @note 内部自动进行低通滤波和死区处理
     */
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

    // 滑动窗口滤波器（替代一阶低通）
    static constexpr int VELOCITY_BUFFER_SIZE = 4;  // 4点滑动平均
    // PitchEnd 滤波器
    float velocityBuffer_pitchEnd_[VELOCITY_BUFFER_SIZE] = {0};
    int bufferIndex_pitchEnd_ = 0;
    float lastVelocity_pitchEnd_ = 0.0f;  // 用于方向一致性检查
    // Roll 滤波器
    float velocityBuffer_roll_[VELOCITY_BUFFER_SIZE] = {0};
    int bufferIndex_roll_ = 0;
    float lastVelocity_roll_ = 0.0f;
};

} // namespace my_engineer

#endif // ALGO_GRAVITY_COMP_HPP
