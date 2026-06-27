/**
 * @file algo_grav_comp.hpp
 * @author ciallo (1002046597@qq.com)
 * @brief 六轴机械臂重力补偿算法
 * @version 1.0
 * @date 2026-06-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ALGO_GRAV_COMP_HPP
#define ALGO_GRAV_COMP_HPP

#include "Configuration.hpp"
#include <array>

#define GRAV_KT_MOTOR_AMP_PER_RAW  (33.0f / 2048.0f)   ///< raw转安培
// 电机的转换常量(来自瓴控的数据手册，根据不同的电机具体自己查)
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
    float_t roll_tau_ff = 0.0f;       ///< Roll力矩前馈

    float_t pitch1_tau = 0.0f; ///< Pitch1关节力矩
    float_t pitch2_tau = 0.0f; ///< Pitch2关节力矩
    float_t roll_tau = 0.0f;   ///< Roll关节力矩

    float_t end_roll_tau_ff = 0.0f;   ///< 末端Roll力矩前馈
    float_t end_pitch_tau_ff = 0.0f;  ///< 末端Pitch力矩前馈
    float_t end_roll_tau = 0.0f;      ///< 末端Roll关节力矩
    float_t end_pitch_tau = 0.0f;     ///< 末端Pitch关节力矩
};

/**
 * @brief 重力补偿参数
 *
 */
struct SArmGravityParam {
    // 关节零点偏移 (deg)，物理零点相对重力零点的偏移
    float_t pitch1_zero_deg = 0.0f;    ///< Pitch1零点偏移
    float_t pitch2_zero_deg = 0.0f;    ///< Pitch2零点偏移
    float_t roll_zero_deg = 0.0f;      ///< Roll零点偏移
    float_t end_roll_zero_deg = 0.0f;  ///< 末端Roll零点偏移
    float_t end_pitch_zero_deg = 0.0f; ///< 末端Pitch零点偏移

    // 电机转换参数
    SMotorConversion pitch1_motor;    ///< Pitch1电机
    SMotorConversion pitch2_motor;    ///< Pitch2电机
    SMotorConversion roll_motor;      ///< Roll电机
    SMotorConversion end_roll_motor;  ///< 末端Roll电机
    SMotorConversion end_pitch_motor; ///< 末端Pitch电机

    // 安全限幅
    float_t pitch1_ff_limit = 1000.0f;  ///< Pitch1前馈限幅
    float_t pitch2_ff_limit = 1000.0f;  ///< Pitch2前馈限幅
    float_t roll_tau_limit = 10.0f;     ///< Roll力矩限幅 (N·m)
    float_t end_roll_tau_limit = 5.0f;  ///< 末端Roll力矩限幅 (N·m)
    float_t end_pitch_tau_limit = 5.0f; ///< 末端Pitch力矩限幅 (N·m)

    // 平滑过渡
    float_t ramp_alpha = 0.05f; ///< ramp渐变步长

    // 安全工作空间
    float_t ws_pitch1_min = 0.0f;  float_t ws_pitch1_max = 0.0f;
    float_t ws_pitch2_min = 0.0f;  float_t ws_pitch2_max = 0.0f;
    float_t ws_roll_min = 0.0f;    float_t ws_roll_max = 0.0f;

    // pitch1: 1, s2, c2, s23, c23, s235, c235, s2356, c2356, s23567, c23567
    std::array<float_t, 11> pitch1_tau_coeff{
        -7.0393752f, -110.3010323f, 27.9130821f, 9.8493382f, 0.7143019f,
        -1.6631875f, -3.6380182f, -3.9075982f, -4.2533189f, -2.8495385f, 1.3570302f
    };

    // pitch2: 1, s23, c23
    std::array<float_t, 3> pitch2_tau_coeff{
        -6.2971922f, -1.3754504f, -0.0072020f
    };

    // roll: 1, s5, c5,
    // a7*sr*s5, a7*cr*s5, a7*sr*c5, a7*cr*c5,
    std::array<float_t, 11> roll_tau_coeff{
        -0.0103628f,
         0.0000000f,  0.0000000f,
        -0.3050435f, -1.1356033f, -0.0006946f,  0.0664156f,
         0.0000000f,  0.0000000f,  0.0000000f,  0.0000000f
    };

    // end_roll: 1, s6, c6, s67, c67
    std::array<float_t, 5> end_roll_tau_coeff{
        -0.0276435f, 0.0133025f, 0.0009081f, 0.0016892f, -0.0157651f
    };

    // end_pitch: 1, s7, c7, c5*c7, cr*s7, s5*c7
    std::array<float_t, 6> end_pitch_tau_coeff{
        +0.0479348f,
        -0.3221968f, +0.0118103f,
        +0.9539213f,
        +1.1628844f,
        +0.1584943f
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
