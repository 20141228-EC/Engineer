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

#ifndef ALGO_GRAV_COMP_HPP
#define ALGO_GRAV_COMP_HPP

#include "Configuration.hpp"
#include <array>

// 电机的转换常量
#define GRAV_KT_MOTOR_AMP_PER_RAW  (33.0f / 2048.0f)   ///< raw转安培
#define GRAV_KT_MOTOR_RAW_PER_AMP (2048.0f / 33.0f)    ///< 安培转raw

namespace my_engineer {

/**
 * @brief 电机类型枚举
 *
 */
enum class EMotorType {
    CUSTOM = 0,       ///< 自定义参数
    MG8010_I36V2,     ///< LK MG8010 i36V2
    MG6012_I36V3,     ///< LK MG6012 i36V3
    MG5010_I36V3,     ///< LK MG5010 i36V3
    DM_MIT,           ///< DM MIT模式 
};

/**
 * @brief 力矩电流转换
 */
struct SMotorConversion {
    EMotorType type = EMotorType::CUSTOM; ///< 电机类型
    float_t kt = 0.0f;         ///< 电机力矩系数 (N·m/A)
    float_t reduction = 36.0f;  ///< 减速比
    float_t amp_to_raw = 0.0f; ///< 物理电流
    float_t direction = 1.0f;  ///< 指令方向

    float_t JointTorqueToCommand(float_t tau_Nm) const;
    float_t CommandToJointTorque(float_t command) const;

};

/**
 * @brief 重力补偿输入
 *
 */
struct SArmGravityState {
    float_t pitch1_deg = 0.0f;    ///< 大Pitch角度 (deg)
    float_t pitch2_deg = 0.0f;    ///< 小Pitch角度 (deg)
    float_t pitch3_deg = 0.0f;    ///< Pitch3角度 (deg)
    float_t roll_deg = 0.0f;      ///< Roll角度 (deg)
    float_t end_roll_deg = 0.0f;  ///< 末端Roll角度 (deg)
    float_t end_pitch_deg = 0.0f; ///< 末端Pitch角度 (deg)
};

/**
 * @brief 重力补偿输出
 *
 */
struct SArmGravityOutput {
    float_t pitch1_current_ff = 0.0f; ///< Pitch1 raw电流前馈指令
    float_t pitch2_current_ff = 0.0f; ///< Pitch2 raw电流前馈指令
    float_t pitch3_current_ff = 0.0f; ///< Pitch3 raw电流前馈指令
    float_t roll_tau_ff = 0.0f;       ///< Roll力矩前馈

    float_t pitch1_tau = 0.0f; ///< Pitch1关节力矩
    float_t pitch2_tau = 0.0f; ///< Pitch2关节力矩
    float_t pitch3_tau = 0.0f; ///< Pitch3关节力矩
    float_t roll_tau = 0.0f;   ///< Roll关节力矩
};

/**
 * @brief 重力补偿参数
 *
 */
struct SArmGravityParam {
    // 关节零点偏移 (deg)
    float_t pitch1_zero_deg = 0.0f; ///< Pitch1零点偏移
    float_t pitch2_zero_deg = 0.0f; ///< Pitch2零点偏移
    float_t pitch3_zero_deg = 0.0f; ///< Pitch3零点偏移

    // 电机转换参数
    SMotorConversion pitch1_motor; ///< Pitch1电机 
    SMotorConversion pitch2_motor; ///< Pitch2电机 
    SMotorConversion pitch3_motor; ///< Pitch3电机
    SMotorConversion roll_motor;   ///< Roll电机

    // 安全限幅
    float_t pitch1_ff_limit = 600.0f; ///< Pitch1前馈限幅
    float_t pitch2_ff_limit = 600.0f; ///< Pitch2前馈限幅
    float_t pitch3_ff_limit = 600.0f; ///< Pitch3前馈限幅
    float_t roll_tau_limit = 10.f;     ///< Roll力矩限幅 (N·m)

    // 平滑过渡
    float_t ramp_alpha = 0.05f; ///< ramp渐变步长

    // 安全工作空间
    float_t ws_pitch1_min = 0.0f;  float_t ws_pitch1_max = 0.0f;
    float_t ws_pitch2_min = 0.0f;  float_t ws_pitch2_max = 0.0f;
    float_t ws_pitch3_min = 0.0f;  float_t ws_pitch3_max = 0.0f;
    float_t ws_roll_min = 0.0f;    float_t ws_roll_max = 0.0f;

    // 独立拟合重力补偿系数
    // pitch1: [0]bias [1]s2 [2]c2 [3]s23 [4]c23 [5]s234 [6]c234
    //          [7]s2345 [8]c2345 [9]s23456 [10]c23456 [11]s234567 [12]c234567
    std::array<float_t, 13> pitch1_tau_coeff{
        +40.4849251976f, -40.9293076763f, -112.1948276741f,
        +7.2018328190f, +1.9877549850f,
        -6.8156598366f, -2.4649341652f,
        +0.0871209563f, +0.0016252824f,
        +0.0785253976f, -0.0066250491f,
        +0.1331025291f, +0.1426351559f
    };

    // pitch2: [0]bias [1]s23 [2]c23 [3]s234 [4]c234
    //          [5]s2345 [6]c2345 [7]s23456 [8]c23456 [9]s234567 [10]c234567
    std::array<float_t, 11> pitch2_tau_coeff{
        -10.8185882520f, +1.6010217332f, -2.8517580314f,
        -3.9041855156f, +0.1984722830f,
        +0.0508573872f, -0.0990814165f,
        +0.0559589669f, -0.0959280091f,
        -0.0020400095f, -0.0465064848f
    };

    // pitch3: [0]bias [1]s234 [2]c234 [3]s2345 [4]c2345
    //          [5]s23456 [6]c23456 [7]s234567 [8]c234567
    std::array<float_t, 9> pitch3_tau_coeff{
        -22.0734511100f, -1.0440942961f, -0.4588638257f,
        -0.0847867568f, -0.0596231584f,
        -0.0789105974f, -0.0641781642f,
        -0.0917139275f, -0.0612910432f
    };

    // roll: [0]bias [1]s5 [2]c5 [3]s56 [4]c56
    //        [5]s567 [6]c567 [7]s2345 [8]c2345
    std::array<float_t, 9> roll_tau_coeff{
        +0.0657524852f, +0.0862109903f, -0.0430637002f,
        +0.0844546626f, -0.0426298421f,
        +0.0882353563f, -0.0489346043f,
        +0.0038879135f, -0.0726710026f
    };
};

/**
 * @brief 重力补偿算法类
 *
 */
class CAlgoArmGravityComp {
public:
    /**
     * @brief 初始化重力补偿器
     *
     * @param param 重力补偿参数
     * @return EAppStatus
     */
    EAppStatus Init(const SArmGravityParam& param);

    /**
     * @brief 计算重力补偿前馈量
     *
     * @param state 当前关节角度
     * @return SArmGravityOutput 各关节前馈量
     */
    SArmGravityOutput Calc(const SArmGravityState& state);

    /**
     * @brief 启停重力补偿
     * @param enable true启用, false禁用
     */
    void SetEnable(bool enable);

    /**
     * @brief 设置观察模式: 只计算力矩不输出电流指令
     * @param observe true观察模式, false正常输出模式
     */
    void SetObserveMode(bool observe);

    /**
     * @brief 重置补偿器状态
     */
    void Reset();

    const SArmGravityParam& GetParam() const { return param_; }
    bool IsObserveMode() const { return observe_; }

private:

    SArmGravityParam param_{};    // 补偿参数
    float_t ramp_ = 0.0f;    // ramp渐变值 (0~1)
    bool enabled_ = false;  // 启停标志
    bool observe_ = false;  // debug模式,观察补偿的力矩但是不输出力

    /**
     * @brief 检查关节角度是否在安全工作空间内
     *
     * @param state 当前关节角度
     * @return true 在安全范围内
     */
    bool IsStateValid_(const SArmGravityState& state) const;
};

} // namespace my_engineer

#endif // ALGO_GRAV_COMP_HPP
