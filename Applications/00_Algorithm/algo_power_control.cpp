/**
 * @file algo_power_control.cpp
 * @author yh
 * @brief 底盘单电机功率限制
 * @version 1.0
 * @date 2025-11-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "algo_power_control.hpp"
#include <cmath>

namespace my_engineer {
    
ChassisPowerController chassis_power_controller;

/**
 * @brief 单电机功率控制（底盘控制时循环调用4次，传入0~3分别对应LF/RF/LB/RB）
 * @param chassis 底盘实例指针
 * @param motorIndex 电机索引（0:LF, 1:RF, 2:LB, 3:RB）
 */
void ChassisPowerController::control(CModChassis* chassis, uint8_t motorIndex){
    // 检查参数有效性
    if(chassis == nullptr || motorIndex >= 4) return;
    
    // 获取总功率限制
    uint16_t max_power_limit = get_chassis_max_power();
    float chassis_max_power = static_cast<float>(max_power_limit);

    // 获取轮组和当前电机数据
    auto& wheelset = chassis->GetWheelset();
    const CDevMtr* motor = wheelset.motor[motorIndex];
    if(motor == nullptr || !motor->IsMotorOnline()){
        return; // 电机离线，不处理
    }

    // 读取电机当前状态（转速 + PID输出转矩）
    float speed_rpm = static_cast<float>(motor->motorData[CDevMtr::DATA_SPEED]);
    float torque_cmd = static_cast<float>(wheelset.pidSpdCtrl.out[motorIndex]);

    // 计算当前电机的原始功率（正向功率才参与限制）
    float motor_power = torque_cmd * kTorqueCoeff * speed_rpm
                      + k1 * torque_cmd * torque_cmd
                      + k2 * speed_rpm * speed_rpm
                      + kConstant;
    if(motor_power <= 0 || motor_power < 1e-6f){
        return; // 负功率或极小功率不限制
    }

    // 若单电机功率超限，按比例缩放（可扩展为多电机总功率限制）
    if(motor_power > chassis_max_power){
        float power_scale = chassis_max_power / motor_power;

        // 二次方程求解：k1*T² + b*T + c = 0（T为目标转矩）
        float b = kTorqueCoeff * speed_rpm;
        float c = k2 * speed_rpm * speed_rpm - (motor_power * power_scale) + kConstant;
        float derta = b * b - 4 * k1 * c;

        if(derta >= 0){ // 确保有实根
            float sqrt_derta = std::sqrt(derta);
            float original_torque = torque_cmd; // 保留原始方向
            float target_torque = 0.0f;

            // 根据原始转矩方向选择合适的根（保持方向一致）
            target_torque = (original_torque > 0) 
                          ? (-b + sqrt_derta) / (2 * k1) 
                          : (-b - sqrt_derta) / (2 * k1);

            // 输出限幅（避免超出电机能力）
            if(target_torque > kMotorOutputMax){
                wheelset.pidSpdCtrl.out[motorIndex] = kMotorOutputMax;
            }
            else if(target_torque < -kMotorOutputMax){
                wheelset.pidSpdCtrl.out[motorIndex] = -kMotorOutputMax;
            }
            else {
                wheelset.pidSpdCtrl.out[motorIndex] = static_cast<int16_t>(target_torque);
            }
        }
    }
}

} // namespace my_engineer
