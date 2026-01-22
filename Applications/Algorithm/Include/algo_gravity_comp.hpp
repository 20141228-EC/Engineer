/**
 * @file algo_gravity_comp.hpp
 * @brief 示教器重力补偿算法
 * @author Ciallo
 * @date 2026-01-20
 */

#ifndef ALGO_GRAVITY_COMP_HPP
#define ALGO_GRAVITY_COMP_HPP

#include "Configuration.hpp"
#include <cmath>
#include <algorithm>

namespace my_engineer {

/**
 * @brief 重力补偿算法类
 */
class CAlgoGravityComp {
public:
    // DH偏移常量 (编码器角度 + offset = DH角度)
    static constexpr float DH_OFFSET_PITCH1   = 3.14159265f;   // π
    static constexpr float DH_OFFSET_PITCH2   = 0.0f;
    static constexpr float DH_OFFSET_ROLL     = 1.57079633f;   // π/2
    static constexpr float DH_OFFSET_PITCHEND = -1.57079633f;  // -π/2

    // 重力补偿系数：基于连杆质量和质心位置的MATLAB推导
    struct SGravCoeffs {
        float K2_1 = 1.612f;      // Pitch1: cos(q2)
        float K2_2 = 0.4199f;     // Pitch1: sin(q2+q3)
        float K2_3 = 0.3296f;     // Pitch1: 末端影响
        float K2_4 = 0.03877f;    // Pitch1: Roll影响
        float K2_5 = 0.009967f;   // Pitch1: sin(q2)
        float K3_1 = 0.4199f;     // Pitch2: sin(q2+q3)
        float K3_2 = 0.3296f;     // Pitch2: 末端影响
        float K3_3 = 0.03877f;    // Pitch2: Roll影响
        float K5_1 = 0.3296f;     // PitchEnd
    };

    // 输出力矩 (N·m)
    struct SGravTorques {
        float tau_yaw = 0.0f;
        float tau_pitch1 = 0.0f;
        float tau_pitch2 = 0.0f;
        float tau_roll = 0.0f;
        float tau_pitchEnd = 0.0f;
    };

    CAlgoGravityComp() = default;

    ///< 初始化补偿算法，可传入自定义系数
    void Init(const SGravCoeffs& coeffs = SGravCoeffs{}) {
        coeffs_ = coeffs;
        enabled_ = true;
        scale_ = 1.0f;
    }

    ///< 计算补偿力矩，输入编码器角度(rad)
    SGravTorques Calculate(float enc_pitch1, float enc_pitch2,
                           float enc_roll, float enc_pitchEnd);

    ///< 计算补偿力矩，输入DH角度(rad)
    SGravTorques CalculateDH(float q2, float q3, float q4, float q5);

    ///< 设置/获取补偿使能
    void SetEnabled(bool enabled) { enabled_ = enabled; }
    bool IsEnabled() const { return enabled_; }

    ///< 设置/获取补偿比例(0.0~2.0)
    void SetScale(float scale) { scale_ = std::clamp(scale, 0.0f, 2.0f); }
    float GetScale() const { return scale_; }

    ///< 更新/获取补偿系数
    void UpdateCoeffs(const SGravCoeffs& coeffs) { coeffs_ = coeffs; }
    const SGravCoeffs& GetCoeffs() const { return coeffs_; }

private:
    SGravCoeffs coeffs_;
    bool enabled_ = true;
    float scale_ = 1.0f;
};

} // namespace my_engineer

#endif // ALGO_GRAVITY_COMP_HPP
