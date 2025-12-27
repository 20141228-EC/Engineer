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

namespace my_engineer {
EAppStatus CAlgoPowerControl::InitPowerControl(const SAlgoInitParamPower* pStructInitParam){
    if(pStructInitParam == nullptr)return APP_ERROR;
    if(PowerStatus == APP_BUSY)return APP_ERROR;
    
    // 拷贝参数到私有成员（参数封装，外部仅通过结构体修改）
    auto &param = *pStructInitParam;
    kDefaultMaxPower = param.kDefaultMaxPower;
    kTorqueCoeff = param.kTorqueCoeff;
    k1 = param.k1;
    k2 = param.k2;
    kConstant = param.kConstant;
    kMotorOutputMax = param.kMotorOutputMax;
    
    // 初始化后重置算法状态
    ResetPowerControl();
    PowerStatus = EAppStatus::APP_OK;

    return APP_OK;
}
/**
 * @brief 实时更新功率限制（核心接口）
 */
 int16_t CAlgoPowerControl::UpdatePowerLimit(float currentSpeedRpm, int16_t currentTorque){
    if(PowerStatus != APP_OK)return currentTorque;

    // 转换为浮点型用于计算
    float torqueFloat = static_cast<float>(currentTorque);
    // 计算当前电机功率
    float currentPower = CalcMotorPower(currentSpeedRpm, torqueFloat);

    if(currentPower <= 0.0f || currentPower <= static_cast<float>(kDefaultMaxPower) + 1e-6f){
        // 未超功率，直接返回原始转矩
        return currentTorque;
    }
    // 计算目标功率
    float targetPower = static_cast<float>(kDefaultMaxPower);
    // 求解限制以后的转矩
    int8_t torqueSign = (currentTorque >= 0) ? 1 : -1;
    float targetTorque = SolveTargetTorque(currentSpeedRpm, targetPower, torqueSign);
    // 限幅输出转矩
    targetTorque = std::clamp(targetTorque, -static_cast<float>(kMotorOutputMax), static_cast<float>(kMotorOutputMax));

    return static_cast<int16_t>(targetTorque);
 }

 /**
 * @brief 重置功率限制算法
 */
EAppStatus CAlgoPowerControl::ResetPowerControl() {
    if (PowerStatus == EAppStatus::APP_BUSY) return APP_ERROR;
    
    PowerStatus = EAppStatus::APP_RESET; 
    return APP_OK;
}
/**
 * @brief 计算电机当前功率（内部辅助函数）
 */
 float CAlgoPowerControl::CalcMotorPower(float speedRpm, float torque) {
    // 功率公式：P = T*Kt*n + k1*T² + k2*n² + P0
    // T=转矩，Kt=转矩系数，n=转速，k1/k2=损耗系数，P0=固定损耗
    return torque * kTorqueCoeff * speedRpm
         + k1 * torque * torque
         + k2 * speedRpm * speedRpm
         + kConstant;
 }
 /**
 * @brief 求解目标转矩（二次方程：k1*T² + b*T + c = 0）
 */
 float CAlgoPowerControl::SolveTargetTorque(float speedRpm, float targetPower, int8_t originalTorqueSign){
    // // 二次方程系数：k1*T² + b*T + c = 0
    float coeff_k1 = k1;
    float coeff_b = kTorqueCoeff * speedRpm;
    float coeff_c = k2 * speedRpm * speedRpm + kConstant - targetPower;
    
    float derta = coeff_b * coeff_b - 4.0f * coeff_k1 * coeff_c;
    if(derta < 0.0f){
        // 无实数解，返回0转矩
        return 0.0f;
    }
    
    float sqrtDerta = std::sqrt(derta);
    float targetTorque = 0.0f;

    // 处理k1=0的情况（退化为一次方程）
    if (std::fabs(coeff_k1) < 1e-9f) {
        if (std::fabs(coeff_b) < 1e-9f) { // 无有效解，返回0
            return 0.0f;
        }
        targetTorque = -coeff_c / coeff_b;
    } else {
        // 有两个解，选择符合原始转矩符号的解
        targetTorque = (-coeff_b + (originalTorqueSign >= 0 ? sqrtDerta : -sqrtDerta)) / (2.0f * coeff_k1);
    }
    return targetTorque;
 }
} // namespace my_engineer
