/**
 * @file mod_chassis.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 底盘模块
 * @version 1.0
 * @date 2025-12-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "mod_chassis.hpp"
#include "RTT_DEBUG.h"
#include "sys_referee.hpp"
#include <cmath>
#include <cstring>

float wheel_power_lf = 0.0f;
float wheel_power_rf = 0.0f;
float wheel_power_lb = 0.0f;
float wheel_power_rb = 0.0f;
float powermeter = 0.0f;
float steer_power_lf = 0.0f;
float steer_power_rf = 0.0f;
float steer_power_lb = 0.0f;
float steer_power_rb = 0.0f;
float wheel_torque_lf = 0.0f;
float wheel_torque_rf = 0.0f;
float wheel_torque_lb = 0.0f;
float wheel_torque_rb = 0.0f;
float power_total = 0.0f;
float power_demand_total = 0.0f;
float power_cmd_total = 0.0f;
float power_budget = 0.0f;
float power_buffer_est = 0.0f;
float power_feedback_est = 0.0f;
float power_measure_used = 0.0f;
float power_guard_budget = 0.0f;
float steer_angle_lf_deg = 0.0f;
float steer_angle_rf_deg = 0.0f;
float steer_angle_lb_deg = 0.0f;
float steer_angle_rb_deg = 0.0f;
float_t crawler_torque_l = 0.0f;
float_t crawler_torque_r = 0.0f;
float raw_torque_LL = 0.0f;
float raw_torque_LR = 0.0f;
float actual_torque_LL = 0.0f;
float actual_torque_LR = 0.0f;
float_t raw_speed_LL = 0.0f;
int16_t max_power = 0.0f;
float_t power_demand_steer_total = 0.f;
float_t power_demand_wheel_total = 0.f;

namespace my_engineer {

CModChassis *pChassis_test = nullptr;

/**
 * @brief 初始化底盘模块
 * 
 * @param param
 * @return EAppStatus 
 */
EAppStatus CModChassis::InitModule(SModInitParam_Base &param){
    // 检查param是否正确
    if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    // 类型转换
    auto chassisParam = static_cast<SModInitParam_Chassis &>(param);
    moduleID = chassisParam.moduleID;

    //获取算法指针
    filter = static_cast<CAlgo_IMU_Ave*>(AlgoIDMap.at(chassisParam.FilterID));
    if(!filter){
        return APP_ERROR;
    }

    // 初始化底盘轮组
    comWheelset_.InitComponent(param);

    chassisMaxPower_ = chassisParam.chassisMaxPower;            // 保存底盘总功率限制
    dynamicPowerBudget_ = static_cast<float>(chassisMaxPower_);
    softPowerBudget_ = dynamicPowerBudget_ * kSoftLimitRatio_;
    power_budget = dynamicPowerBudget_;

    // 初始化4个电机的功率限制实例
    powerCtrlLF_.InitPowerControl(&chassisParam.powerParamLF);  // 左前电机功率初始化
    powerCtrlRF_.InitPowerControl(&chassisParam.powerParamRF);  // 右前电机功率初始化
    powerCtrlLB_.InitPowerControl(&chassisParam.powerParamLB);  // 左后电机功率初始化
    powerCtrlRB_.InitPowerControl(&chassisParam.powerParamRB);  // 右后电机功率初始化
    powerCtrlSteerLF_.InitPowerControl(&chassisParam.steerPowerParamLF);  // 左前舵向电机功率初始化
    powerCtrlSteerRF_.InitPowerControl(&chassisParam.steerPowerParamRF);  // 右前舵向电机功率初始化
    powerCtrlSteerLB_.InitPowerControl(&chassisParam.steerPowerParamLB);  // 左后舵向电机功率初始化
    powerCtrlSteerRB_.InitPowerControl(&chassisParam.steerPowerParamRB);  // 右后舵向电机功率初始化

    // 初始化功率计CAN接收节点（读取0x516反馈值）
    if (chassisParam.powerMeterCanID != EInterfaceID::INF_NULL && chassisParam.powerMeterStdID != 0U) {
        powerMeterRxNode_.InitRxNode(chassisParam.powerMeterCanID,
                                     chassisParam.powerMeterStdID,
                                     CInfCAN::ECanFrameType::DATA,
                                     chassisParam.powerMeterFrameDlc);
        powerMeterLastTimestamp_ = 0;
    }

    // 创建任务并注册模块
    CreateModuleTask_();
    RegisterModule_();

    // test
    pChassis_test = this;

    Module_FSMFlag_ = FSM_RESET;
    moduleStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 计算需求功率
 * @param 轮组实例
 * 
 */
float CModChassis::CalcTotalDemandPower(const CComWheelset& wheelset, float wheelDemand[4], float steerDemand[4]){
    float totalDemand = 0.0f;

    wheelDemand[CComWheelset::LF] = std::max(0.0f, powerCtrlLF_.CalcMotorPower(
        static_cast<float>(wheelset.motor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::LF])));
    wheelDemand[CComWheelset::RF] = std::max(0.0f, powerCtrlRF_.CalcMotorPower(
        static_cast<float>(wheelset.motor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::RF])));
    wheelDemand[CComWheelset::LB] = std::max(0.0f, powerCtrlLB_.CalcMotorPower(
        static_cast<float>(wheelset.motor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::LB])));
    wheelDemand[CComWheelset::RB] = std::max(0.0f, powerCtrlRB_.CalcMotorPower(
        static_cast<float>(wheelset.motor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::RB])));

    steerDemand[CComWheelset::LF] = std::max(0.0f, powerCtrlSteerLF_.CalcMotorPower(
        static_cast<float>(wheelset.steerMotor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.mtrSteerOutputBuffer[CComWheelset::LF])));
    steerDemand[CComWheelset::RF] = std::max(0.0f, powerCtrlSteerRF_.CalcMotorPower(
        static_cast<float>(wheelset.steerMotor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.mtrSteerOutputBuffer[CComWheelset::RF])));
    steerDemand[CComWheelset::LB] = std::max(0.0f, powerCtrlSteerLB_.CalcMotorPower(
        static_cast<float>(wheelset.steerMotor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.mtrSteerOutputBuffer[CComWheelset::LB])));
    steerDemand[CComWheelset::RB] = std::max(0.0f, powerCtrlSteerRB_.CalcMotorPower(
        static_cast<float>(wheelset.steerMotor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.mtrSteerOutputBuffer[CComWheelset::RB])));

    for (int i = 0; i < 4; i++) {
        totalDemand += wheelDemand[i] + steerDemand[i];
    }

    return totalDemand;
}

void CModChassis::AllocDynamicPower(const CComWheelset& wheelset, float targetWheelPower[4], float targetSteerPower[4]) {
    float wheelDemand[4] = {0.0f};
    float steerDemand[4] = {0.0f};
    CalcTotalDemandPower(wheelset, wheelDemand, steerDemand);

    // 舵向电机直接给需求功率，不做限制
    for (int i = 0; i < 4; i++) {
        targetSteerPower[i] = steerDemand[i];
    }

    // 总功率限制115W，减去舵向电机功率
    constexpr float kTotalWheelPower = 115.0f;
    float totalSteerDemand = 0.0f;
    for (int i = 0; i < 4; i++) totalSteerDemand += steerDemand[i];

    float remainingWheelPower = kTotalWheelPower - totalSteerDemand;    // 减去舵向后剩余功率
    if (remainingWheelPower < 0.0f) remainingWheelPower = 0.0f;

    // 按比例分配给轮向电机
    float totalWheelDemand = 0.0f;
    for (int i = 0; i < 4; i++) totalWheelDemand += wheelDemand[i];

    if (totalWheelDemand > 1e-6f) {
        float wheelScale = remainingWheelPower / totalWheelDemand;
        for (int i = 0; i < 4; i++) {
            targetWheelPower[i] = wheelDemand[i] * wheelScale;      // 对每个轮毂作比例缩放
        }
    } else {
        for (int i = 0; i < 4; i++) targetWheelPower[i] = 0.0f;
    }
}

void CModChassis::UpdatePowerBudget_() {
    if (SysReferee.refereeInfo.robot.robotMaxPower > 0) {
        chassisMaxPower_ = static_cast<uint16_t>(SysReferee.refereeInfo.robot.robotMaxPower);
        max_power = chassisMaxPower_;
    }

    const uint32_t now = HAL_GetTick();
    const bool powerMeterOnline = (powerMeterLastTimestamp_ != 0U)
        && ((now - powerMeterLastTimestamp_) <= kPowerMeterOfflineTimeoutMs_);
    const float measuredPower = powerMeterOnline ? std::max(0.0f, powermeter) : std::max(0.0f, feedbackMeasuredPower_);
    power_measure_used = measuredPower;

    dynamicPowerBudget_ = static_cast<float>(chassisMaxPower_);
    softPowerBudget_ = dynamicPowerBudget_ * kSoftLimitRatio_;
    power_budget = dynamicPowerBudget_;
    power_buffer_est = 0.0f;
    lastMeasuredPower_ = measuredPower;
}

/**
 * @brief 更新处理
 * 
 * @return EAppStatus 
 */
void CModChassis::UpdateHandler_() {

    // 检查模块状态
    if (moduleStatus == APP_RESET) return;

    // ------------------ 读取功率计 ------------------
    if (powerMeterRxNode_.timestamp > powerMeterLastTimestamp_ &&
        powerMeterRxNode_.dataBuffer.size() >= sizeof(float)) {
        std::memcpy(&powermeter, powerMeterRxNode_.dataBuffer.data(), sizeof(float));
        powerMeterLastTimestamp_ = powerMeterRxNode_.timestamp;
    }

    // 根据电机速度和发送电流计算预测功率
    auto calcFeedbackPower = [](CAlgoPowerControl &powerCtrl, CDevMtr *motor) -> float {
        if (!motor) return 0.0f;
        return std::max(0.0f, powerCtrl.CalcMotorPower(
            static_cast<float>(motor->motorData[CDevMtr::DATA_SPEED]),
            static_cast<float>(motor->motorData[CDevMtr::DATA_CURRENT])
        ));
    };

    // 总预测功率
    const float feedbackPowerRaw =
        calcFeedbackPower(powerCtrlLF_, comWheelset_.motor[CComWheelset::LF]) +
        calcFeedbackPower(powerCtrlRF_, comWheelset_.motor[CComWheelset::RF]) +
        calcFeedbackPower(powerCtrlLB_, comWheelset_.motor[CComWheelset::LB]) +
        calcFeedbackPower(powerCtrlRB_, comWheelset_.motor[CComWheelset::RB]) +
        calcFeedbackPower(powerCtrlSteerLF_, comWheelset_.steerMotor[CComWheelset::LF]) +
        calcFeedbackPower(powerCtrlSteerRF_, comWheelset_.steerMotor[CComWheelset::RF]) +
        calcFeedbackPower(powerCtrlSteerLB_, comWheelset_.steerMotor[CComWheelset::LB]) +
        calcFeedbackPower(powerCtrlSteerRB_, comWheelset_.steerMotor[CComWheelset::RB]);

    feedbackMeasuredPowerRaw_ = feedbackPowerRaw;
    // 做一个低通滤波 避免预测功率抖动而频繁限制
    constexpr float kFeedbackPowerLpfAlpha = 0.18f;
    feedbackMeasuredPower_ += kFeedbackPowerLpfAlpha * (feedbackPowerRaw - feedbackMeasuredPower_);
    power_feedback_est = feedbackMeasuredPower_;    // 低通滤波后的预测功率

    // ------------------ 更新功率预算 ------------------
    UpdatePowerBudget_();

    // ------------------ 更新底盘轮组 ------------------
    comWheelset_.UpdateComponent();

    // ------------------ 动态功率分配 ------------------
    float dynamicTargetWheelPower[4] = {0.0f};
    float dynamicTargetSteerPower[4] = {0.0f};
    AllocDynamicPower(comWheelset_, dynamicTargetWheelPower, dynamicTargetSteerPower);

    auto toPowerUInt = [](float power) -> uint16_t {
        return static_cast<uint16_t>(std::lround(std::max(0.0f, power)));
    };

    // ------------------ 设置轮向电机功率上限 ------------------
    powerCtrlLF_.SetDefaultMaxPower(toPowerUInt(dynamicTargetWheelPower[CComWheelset::LF]));
    powerCtrlRF_.SetDefaultMaxPower(toPowerUInt(dynamicTargetWheelPower[CComWheelset::RF]));
    powerCtrlLB_.SetDefaultMaxPower(toPowerUInt(dynamicTargetWheelPower[CComWheelset::LB]));
    powerCtrlRB_.SetDefaultMaxPower(toPowerUInt(dynamicTargetWheelPower[CComWheelset::RB]));

    // // 舵向电机不限制功率，不再设置限制
    // for (int i = 0; i < 4; i++) {
    //     comWheelset_.mtrSteerOutputBuffer[i] = static_cast<int16_t>(dynamicTargetSteerPower[i]);
    // }

    // ------------------ 轮向电机限幅 ------------------
    int16_t limitedTorque[4];
    for (int i = 0; i < 4; i++) {
        float wheelSpeed = static_cast<float>(comWheelset_.motor[i]->motorData[CDevMtr::DATA_SPEED]);
        limitedTorque[i] = (&powerCtrlLF_ + i)->UpdatePowerLimit(wheelSpeed, comWheelset_.mtrOutputBuffer[i]);
        comWheelset_.mtrOutputBuffer[i] = limitedTorque[i];  // 更新输出缓冲区
    }

    // debug变量开始
    constexpr float kWheelPowerLpfAlpha = 0.08f;
    float wheelPowerRaw[4] = {
        powerCtrlLF_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]),
                                    static_cast<float>(comWheelset_.mtrOutputBuffer[CComWheelset::LF])),
        powerCtrlRF_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]),
                                    static_cast<float>(comWheelset_.mtrOutputBuffer[CComWheelset::RF])),
        powerCtrlLB_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]),
                                    static_cast<float>(comWheelset_.mtrOutputBuffer[CComWheelset::LB])),
        powerCtrlRB_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]),
                                    static_cast<float>(comWheelset_.mtrOutputBuffer[CComWheelset::RB]))
    };

    // 这里同样作滤波处理
    wheel_power_lf += kWheelPowerLpfAlpha * (wheelPowerRaw[0] - wheel_power_lf);
    wheel_power_rf += kWheelPowerLpfAlpha * (wheelPowerRaw[1] - wheel_power_rf);
    wheel_power_lb += kWheelPowerLpfAlpha * (wheelPowerRaw[2] - wheel_power_lb);
    wheel_power_rb += kWheelPowerLpfAlpha * (wheelPowerRaw[3] - wheel_power_rb);

    // 舵向功率直接取预测值
    steer_power_lf = dynamicTargetSteerPower[CComWheelset::LF];
    steer_power_rf = dynamicTargetSteerPower[CComWheelset::RF];
    steer_power_lb = dynamicTargetSteerPower[CComWheelset::LB];
    steer_power_rb = dynamicTargetSteerPower[CComWheelset::RB];

    // ------------------ 功率统计 ------------------
    power_demand_steer_total = steer_power_lf + steer_power_rf + steer_power_lb + steer_power_rb;
    power_demand_wheel_total = wheelPowerRaw[0] + wheelPowerRaw[1] + wheelPowerRaw[2] + wheelPowerRaw[3];
    power_demand_total = wheelPowerRaw[0] + wheelPowerRaw[1] + wheelPowerRaw[2] + wheelPowerRaw[3] +
                         steer_power_lf + steer_power_rf + steer_power_lb + steer_power_rb;

    // debug变量结束

    // ------------------ 填充 CAN 发送缓冲区 ------------------
    for (int i = 0; i < 4; i++) {
        CDevMtrDJI::FillCanTxBuffer(comWheelset_.motor[i],
                                    comWheelset_.mtrCanTxNode[i]->dataBuffer,
                                    comWheelset_.mtrOutputBuffer[i]);
        CDevMtrDJI::FillCanTxBuffer(comWheelset_.steerMotor[i],
                                    comWheelset_.mtrSteerCanTxNode[i]->dataBuffer,
                                    comWheelset_.mtrSteerOutputBuffer[i]);
    }

    // ------------------ 最终总功率命令 ------------------
    power_cmd_total = 0.0f;
    for (int i = 0; i < 4; i++) {
        power_cmd_total += powerCtrlLF_.CalcMotorPower(
            static_cast<float>(comWheelset_.motor[i]->motorData[CDevMtr::DATA_SPEED]),
            static_cast<float>(comWheelset_.motor[i]->motorData[CDevMtr::DATA_CURRENT])
        );
    }
    power_total = power_cmd_total;
}

/**
 * @brief 心跳处理
 * 
 * @return EAppStatus 
 */
void CModChassis::HeartbeatHandler_(){

}

/**
 * @brief 创建模块任务
 * 
 * @return EAppStatus 
 */
EAppStatus CModChassis::CreateModuleTask_(){
    
    // 任务已存在，删除任务
    if(moduleTaskHandle != nullptr) vTaskDelete(moduleTaskHandle);

    // 创建任务
    xTaskCreate(StartChassisModuleTask, "Chassis Module Task", 
                512, this, proc_ModuleTaskPriority, 
                &moduleTaskHandle);
    
    return APP_OK;
}

/**
 * @brief 限制底盘模块的控制命令大小
 * 
 * @details 会在StartChassisModuleTask中被调用
 */
EAppStatus CModChassis::RestrictChassisCommand_() {

    // 检查模块状态
    if (moduleStatus == APP_RESET) {
        chassisCmd = SChassisCmd();
        return APP_ERROR;
    }

    // 平面速度圆限幅：避免斜向输入时合速度超过100%
    {
        const float planarMag = std::sqrt(chassisCmd.speed_X * chassisCmd.speed_X +
                                          chassisCmd.speed_Y * chassisCmd.speed_Y);
        if (planarMag > 440.0f) {
            const float scale = 440.0f / planarMag;
            chassisCmd.speed_X *= scale;
            chassisCmd.speed_Y *= scale;
        }
    }

    // 自动控制启用，则不继续做限制
    if (chassisCmd.isAutoCtrl) return APP_OK;

    return APP_OK;
}

} // namespace my_engineer
