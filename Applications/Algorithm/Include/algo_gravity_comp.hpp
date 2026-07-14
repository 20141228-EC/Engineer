/**
 * @file algo_gravity_comp.hpp
 * @brief 示教器重力补偿算法 
 * @author Ciallo
 * @date 2026-06-25
 */

#ifndef ALGO_GRAVITY_COMP_HPP
#define ALGO_GRAVITY_COMP_HPP

#include "Configuration.hpp"

#include <array>
#include <algorithm>
#include <cmath>

namespace my_engineer {

///< DM 电机 raw/A 默认转换系数 
#define GRAV_KT_MOTOR_RAW_PER_AMP (2048.0f / 33.0f)

/**
 * @brief 电机类型枚举
 */
enum class EMotorType {
    CUSTOM = 0,      ///< 自定义参数
    DM_MIT,          ///< DM 电机 MIT 模式 
    DJI_CURRENT,     ///< DJI 电机电流模式
};

/**
 * @brief 关节力矩与电机指令的转换抽象
 * 替代旧版散落在调用处的 /GEAR_RATIO 硬编码
 */
struct SMotorConversion {
    EMotorType type = EMotorType::CUSTOM; ///< 电机类型
    float kt = 0.0f;                      ///< 电机力矩系数 
    float reduction = 1.0f;               ///< 减速比 
    float amp_to_raw = 0.0f;              ///< 电流转 raw 系数, 0 表示用默认值
    float direction = 1.0f;               ///< 指令方向系数

    ///< 关节力矩 (N·m) -> 电机指令 
    float JointTorqueToCommand(float tau_Nm) const;
    ///< 电机指令 -> 关节力矩 
    float CommandToJointTorque(float command) const;
};

/**
 * @brief 重力补偿输入: 当前各关节物理角度 (deg)
 */
struct SGravState {
    float pitch1_deg = 0.0f;    ///< Pitch1 角度
    float pitch2_deg = 0.0f;    ///< Pitch2 角度
    float roll_deg = 0.0f;      ///< Roll 角度
    float pitch_end_deg = 0.0f; ///< PitchEnd 角度
};

/**
 * @brief 重力补偿输出
 * tau 为关节侧重力矩, ff 为转换后的电机前馈指令
 */
struct SGravOutput {
    float pitch1_tau = 0.0f; ///< Pitch1 关节力矩
    float pitch2_tau = 0.0f; ///< Pitch2 关节力矩
    float roll_tau = 0.0f;   ///< Roll 关节力矩
    float pitch_end_tau = 0.0f; ///< PitchEnd 关节力矩

    float pitch1_ff = 0.0f; ///< Pitch1 电机前馈指令
    float pitch2_ff = 0.0f; ///< Pitch2 电机前馈指令
    float roll_ff = 0.0f;   ///< Roll 电机前馈指令
    float pitch_end_ff = 0.0f; ///< PitchEnd 电机前馈指令
};

/**
 * @brief 重力补偿参数
 */
struct SGravParam {
    ///< 关节零点偏移 (deg): 物理零点相对重力零点的偏移
    float pitch1_zero_deg = 0.0f;
    float pitch2_zero_deg = 0.0f;
    float roll_zero_deg = 0.0f;
    float pitch_end_zero_deg = 0.0f;

    ///< 电机转换参数
    SMotorConversion pitch1_motor;
    SMotorConversion pitch2_motor;
    SMotorConversion roll_motor;
    SMotorConversion pitch_end_motor;

    ///< 前馈限幅
    float pitch1_ff_limit = 1000.0f;
    float pitch2_ff_limit = 1000.0f;
    float roll_ff_limit = 1.0f;
    float pitch_end_ff_limit = 1.0f;

    ///< ramp 渐变步长 (每周期增加的比例, 0~1)
    float ramp_alpha = 0.05f;

    ///< 安全工作空间 (deg), 超出则强制 ramp 下降防止力矩突变
    float ws_pitch1_min = -180.0f;  float ws_pitch1_max = 180.0f;
    float ws_pitch2_min = -180.0f;  float ws_pitch2_max = 180.0f;
    float ws_roll_min = -180.0f;    float ws_roll_max = 180.0f;

    ///< p2 软边界
    float ws_pitch2_soft_max = 150.0f;    ///< p2 软件限位
    float pitch2_pushback_gain = 0.5f;    ///< 推回增益系数 , 每超出 1度 叠加 0.5 Nm 反向力

    ///< roll 软边界 
    float ws_roll_soft_max = 120.0f;      ///< roll 软件限位 
    float roll_pushback_fixed = 0.009f;     ///< 固定反向力 , 抵消线束拉力

    // pitch1: 1, s2, c2, s3, c3, s2s3, s2c3, c2s3, c2c3, sin(2q3), cos(2q3), s56, c56
    std::array<float, 13> pitch1_coeff{
        -2.033396f, -0.827100f, 12.080461f, 1.557426f, -3.123607f, -4.038908f, 1.719241f, 9.572811f, -3.194318f, -1.567572f, 2.273198f, -0.007622f, -0.016689f  ///< 已辨识 (fit_gravity.py)
    };

    // pitch2: 1, s2, c2, s3, c3, s2s3, s2c3, c2s3, c2c3, sin(2q3), cos(2q3), s56, c56
    std::array<float, 13> pitch2_coeff{
        -1.361942f, 2.139958f, 0.320018f, 1.166561f, 0.849708f, 3.719811f, 0.189491f, -2.599879f, 5.397766f, 0.802420f, -0.043013f, 0.008976f, 0.124503f  ///< 已辨识 (fit_gravity.py)
    };

    // roll: 1, s5, c5, s6*c5, c6*s5
    std::array<float, 5> roll_coeff{
        0.003913f, 0.019082f, 0.013435f, -0.000705f, -0.002611f 
    };

    // pitch_end: 1, s6, c6, c5*c6, cr*s6, s5*c6
    std::array<float, 6> pitch_end_coeff{
        0.000839f, 0.002008f, -0.003767f, 0.009292f, 0.006722f, -0.000310f 
    };
};

/**
 * @brief 5轴重力补偿算法类
 */
class CAlgoGravityComp {
public:
    ///< 重力补偿模式
    enum class CGravityCompMode {
        NONE = 0,     ///< 关闭 (ramp 下降, 无输出)
        ENABLE,       ///< 正常补偿 (ramp 上升, 输出 ff)
        OBSERVE,      ///< 观察模式 (ramp 上升, 只算 tau 不输出 ff, 调试用)
    };

    ///< 初始化, 传入参数
    EAppStatus InitComponent(const SGravParam& param);

    ///< 计算重力补偿前馈, 输入各关节角度 (deg)
    SGravOutput Calc(const SGravState& state);

    ///< 设置模式 (模式切换时 ramp 自动归零)
    void SetMode(CGravityCompMode mode) {
        if (mode_ != mode) ramp_ = 0.0f;
        mode_ = mode;
    }
    CGravityCompMode GetMode() const { return mode_; }

    ///< 重置补偿器状态 (ramp 归零, 模式切换时调用)
    void Reset();

    ///< 获取参数 (可现场修改 K 系数等)
    const SGravParam& GetParam() const { return param_; }
    SGravParam& GetParam() { return param_; }

private:
    SGravParam param_{};       ///< 补偿参数
    float ramp_ = 0.0f;        ///< ramp 渐变值 (0~1)
    CGravityCompMode mode_ = CGravityCompMode::NONE;   ///< 当前模式

    ///< 检查关节角度是否在安全工作空间内

    bool IsStateValid_(const SGravState& state) const;
};

} // namespace my_engineer

#endif // ALGO_GRAVITY_COMP_HPP
