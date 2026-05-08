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

/**
 * @brief 动态功率分配
 * 
 * 
 */
void CModChassis::AllocDynamicPower(const CComWheelset& wheelset, float targetWheelPower[4], float targetSteerPower[4]) {
    float wheelDemand[4] = {0.0f};
    float steerDemand[4] = {0.0f};
    CalcTotalDemandPower(wheelset, wheelDemand, steerDemand);

    float wheelDemandTotal = 0.0f;
    float steerDemandTotal = 0.0f;
    for (int i = 0; i < 4; i++) {
        wheelDemandTotal += wheelDemand[i];
        steerDemandTotal += steerDemand[i];
    }

    constexpr float eps = 1e-6f;
    float wheelLimitCoe = 1.0f;
    float steerLimitCoe = 1.0f;

    // 轮向电机分配105W，内部动态分配
    if (wheelDemandTotal > 105.0f + eps) {
        wheelLimitCoe = 105.0f / wheelDemandTotal;
    }

    // 舵向电机分配10W，内部动态分配
    if (steerDemandTotal > 10.0f + eps) {
        steerLimitCoe = 10.0f / steerDemandTotal;
    }

    for (int i = 0; i < 4; i++) {
        targetWheelPower[i] = wheelDemand[i] * wheelLimitCoe;
        targetSteerPower[i] = steerDemand[i] * steerLimitCoe;
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
void CModChassis::UpdateHandler_(){

    // 检查模块状态
    if (moduleStatus == APP_RESET) return;

    // 读取功率计反馈值
    if (powerMeterRxNode_.timestamp > powerMeterLastTimestamp_ &&
        powerMeterRxNode_.dataBuffer.size() >= sizeof(float)) {
        std::memcpy(&powermeter, powerMeterRxNode_.dataBuffer.data(), sizeof(float));
        powerMeterLastTimestamp_ = powerMeterRxNode_.timestamp;
    }

    auto calcFeedbackPower = [](CAlgoPowerControl &powerCtrl, CDevMtr *motor) -> float {
        if (motor == nullptr) {
            return 0.0f;
        }

        return std::max(0.0f, powerCtrl.CalcMotorPower(
            static_cast<float>(motor->motorData[CDevMtr::DATA_SPEED]),
            static_cast<float>(motor->motorData[CDevMtr::DATA_CURRENT])));
    };

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
    constexpr float kFeedbackPowerLpfAlpha = 0.18f;
    feedbackMeasuredPower_ += kFeedbackPowerLpfAlpha * (feedbackPowerRaw - feedbackMeasuredPower_);
    power_feedback_est = feedbackMeasuredPower_;

    UpdatePowerBudget_();

    // 更新底盘轮组
    comWheelset_.UpdateComponent();

    // 功率分配
    float dynamicTargetWheelPower[4] = {0.0f};
    float dynamicTargetSteerPower[4] = {0.0f};
    AllocDynamicPower(comWheelset_, dynamicTargetWheelPower, dynamicTargetSteerPower); // 内部基于统一数据源计算

    auto toPowerUInt = [](float power) -> uint16_t {
        return static_cast<uint16_t>(std::lround(std::max(0.0f, power)));
    };

    // 设置电机功率上限
    powerCtrlLF_.SetDefaultMaxPower(toPowerUInt(dynamicTargetWheelPower[CComWheelset::LF]));
    powerCtrlRF_.SetDefaultMaxPower(toPowerUInt(dynamicTargetWheelPower[CComWheelset::RF]));
    powerCtrlLB_.SetDefaultMaxPower(toPowerUInt(dynamicTargetWheelPower[CComWheelset::LB]));
    powerCtrlRB_.SetDefaultMaxPower(toPowerUInt(dynamicTargetWheelPower[CComWheelset::RB]));

    powerCtrlSteerLF_.SetDefaultMaxPower(toPowerUInt(dynamicTargetSteerPower[CComWheelset::LF]));
    powerCtrlSteerRF_.SetDefaultMaxPower(toPowerUInt(dynamicTargetSteerPower[CComWheelset::RF]));
    powerCtrlSteerLB_.SetDefaultMaxPower(toPowerUInt(dynamicTargetSteerPower[CComWheelset::LB]));
    powerCtrlSteerRB_.SetDefaultMaxPower(toPowerUInt(dynamicTargetSteerPower[CComWheelset::RB]));

    int16_t limitedTorque[4];
    int16_t limitedSteerTorque[4];
    // LF电机
    {
        float speedLF = static_cast<float>(comWheelset_.motor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]);
        limitedTorque[0] = powerCtrlLF_.UpdatePowerLimit(speedLF, comWheelset_.mtrOutputBuffer[CComWheelset::LF]); 
    }
    // RF电机
    {
        float speedRF = static_cast<float>(comWheelset_.motor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]);
        limitedTorque[1] = powerCtrlRF_.UpdatePowerLimit(speedRF, comWheelset_.mtrOutputBuffer[CComWheelset::RF]); 
    }
    // LB电机
    {
        float speedLB = static_cast<float>(comWheelset_.motor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]);
        limitedTorque[2] = powerCtrlLB_.UpdatePowerLimit(speedLB, comWheelset_.mtrOutputBuffer[CComWheelset::LB]); 
    }
    // RB电机
    {
        float speedRB = static_cast<float>(comWheelset_.motor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]);
        limitedTorque[3] = powerCtrlRB_.UpdatePowerLimit(speedRB, comWheelset_.mtrOutputBuffer[CComWheelset::RB]); 
    }

    // LF舵向电机
    {
        float speedLF = static_cast<float>(comWheelset_.steerMotor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]);
        limitedSteerTorque[0] = powerCtrlSteerLF_.UpdatePowerLimit(speedLF, comWheelset_.mtrSteerOutputBuffer[CComWheelset::LF]);
    }
    // RF舵向电机
    {
        float speedRF = static_cast<float>(comWheelset_.steerMotor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]);
        limitedSteerTorque[1] = powerCtrlSteerRF_.UpdatePowerLimit(speedRF, comWheelset_.mtrSteerOutputBuffer[CComWheelset::RF]);
    }
    // LB舵向电机
    {
        float speedLB = static_cast<float>(comWheelset_.steerMotor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]);
        limitedSteerTorque[2] = powerCtrlSteerLB_.UpdatePowerLimit(speedLB, comWheelset_.mtrSteerOutputBuffer[CComWheelset::LB]);
    }
    // RB舵向电机
    {
        float speedRB = static_cast<float>(comWheelset_.steerMotor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]);
        limitedSteerTorque[3] = powerCtrlSteerRB_.UpdatePowerLimit(speedRB, comWheelset_.mtrSteerOutputBuffer[CComWheelset::RB]);
    }

    /**********************用于debug start*****************************/
    // 更新全局变量以供调试
    const float wheel_power_lf_raw = powerCtrlLF_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]),
                                                                 static_cast<float>(comWheelset_.mtrOutputBuffer[CComWheelset::LF]));
    const float wheel_power_rf_raw = powerCtrlRF_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]),
                                                                 static_cast<float>(comWheelset_.mtrOutputBuffer[CComWheelset::RF]));
    const float wheel_power_lb_raw = powerCtrlLB_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]),
                                                                 static_cast<float>(comWheelset_.mtrOutputBuffer[CComWheelset::LB]));
    const float wheel_power_rb_raw = powerCtrlRB_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]),
                                                                 static_cast<float>(comWheelset_.mtrOutputBuffer[CComWheelset::RB]));

    constexpr float kWheelPowerLpfAlpha = 0.08f;
    wheel_power_lf += kWheelPowerLpfAlpha * (wheel_power_lf_raw - wheel_power_lf);
    wheel_power_rf += kWheelPowerLpfAlpha * (wheel_power_rf_raw - wheel_power_rf);
    wheel_power_lb += kWheelPowerLpfAlpha * (wheel_power_lb_raw - wheel_power_lb);
    wheel_power_rb += kWheelPowerLpfAlpha * (wheel_power_rb_raw - wheel_power_rb);

    steer_power_lf = powerCtrlSteerLF_.CalcMotorPower(static_cast<float>(comWheelset_.steerMotor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]),
                                                      static_cast<float>(comWheelset_.mtrSteerOutputBuffer[CComWheelset::LF]));
    steer_power_rf = powerCtrlSteerRF_.CalcMotorPower(static_cast<float>(comWheelset_.steerMotor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]),
                                                      static_cast<float>(comWheelset_.mtrSteerOutputBuffer[CComWheelset::RF]));
    steer_power_lb = powerCtrlSteerLB_.CalcMotorPower(static_cast<float>(comWheelset_.steerMotor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]),
                                                      static_cast<float>(comWheelset_.mtrSteerOutputBuffer[CComWheelset::LB]));
    steer_power_rb = powerCtrlSteerRB_.CalcMotorPower(static_cast<float>(comWheelset_.steerMotor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]),
                                                      static_cast<float>(comWheelset_.mtrSteerOutputBuffer[CComWheelset::RB]));
    power_demand_total = std::max(0.0f, wheel_power_lf_raw) + std::max(0.0f, wheel_power_rf_raw) + std::max(0.0f, wheel_power_lb_raw) + std::max(0.0f, wheel_power_rb_raw) +
                         std::max(0.0f, steer_power_lf) + std::max(0.0f, steer_power_rf) + std::max(0.0f, steer_power_lb) + std::max(0.0f, steer_power_rb);

    // 应用全局缩放，得到最终发送转矩
    int16_t finalTorque[4] = {
        static_cast<int16_t>(limitedTorque[0]),
        static_cast<int16_t>(limitedTorque[1]),
        static_cast<int16_t>(limitedTorque[2]), 
        static_cast<int16_t>(limitedTorque[3])
    };
    int16_t finalSteerTorque[4] = {
        static_cast<int16_t>(limitedSteerTorque[0]),
        static_cast<int16_t>(limitedSteerTorque[1]),
        static_cast<int16_t>(limitedSteerTorque[2]), 
        static_cast<int16_t>(limitedSteerTorque[3])
    };

    const float wheel_power_lf_cmd = powerCtrlLF_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]),
                                                                 static_cast<float>(comWheelset_.motor[CComWheelset::LF]->motorData[CDevMtr::DATA_CURRENT]));
    const float wheel_power_rf_cmd = powerCtrlRF_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]),
                                                                 static_cast<float>(comWheelset_.motor[CComWheelset::RF]->motorData[CDevMtr::DATA_CURRENT]));
    const float wheel_power_lb_cmd = powerCtrlLB_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]),
                                                                 static_cast<float>(comWheelset_.motor[CComWheelset::LB]->motorData[CDevMtr::DATA_CURRENT]));
    const float wheel_power_rb_cmd = powerCtrlRB_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]),
                                                                 static_cast<float>(comWheelset_.motor[CComWheelset::RB]->motorData[CDevMtr::DATA_CURRENT]));
    const float steer_power_lf_cmd = powerCtrlSteerLF_.CalcMotorPower(static_cast<float>(comWheelset_.steerMotor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]),
                                                                      static_cast<float>(finalSteerTorque[0]));
    const float steer_power_rf_cmd = powerCtrlSteerRF_.CalcMotorPower(static_cast<float>(comWheelset_.steerMotor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]),
                                                                      static_cast<float>(finalSteerTorque[1]));
    const float steer_power_lb_cmd = powerCtrlSteerLB_.CalcMotorPower(static_cast<float>(comWheelset_.steerMotor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]),
                                                                      static_cast<float>(finalSteerTorque[2]));
    const float steer_power_rb_cmd = powerCtrlSteerRB_.CalcMotorPower(static_cast<float>(comWheelset_.steerMotor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]),
                                                                      static_cast<float>(finalSteerTorque[3]));
    power_cmd_total = std::max(0.0f, wheel_power_lf_cmd) + std::max(0.0f, wheel_power_rf_cmd) + std::max(0.0f, wheel_power_lb_cmd) + std::max(0.0f, wheel_power_rb_cmd) +
                      std::max(0.0f, steer_power_lf_cmd) + std::max(0.0f, steer_power_rf_cmd) + std::max(0.0f, steer_power_lb_cmd) + std::max(0.0f, steer_power_rb_cmd);
    power_total = power_cmd_total;

    // 更新最终转矩到输出缓冲区
    comWheelset_.mtrOutputBuffer[CComWheelset::LF] = finalTorque[0];
    comWheelset_.mtrOutputBuffer[CComWheelset::RF] = finalTorque[1];
    comWheelset_.mtrOutputBuffer[CComWheelset::LB] = finalTorque[2];
    comWheelset_.mtrOutputBuffer[CComWheelset::RB] = finalTorque[3];
    comWheelset_.mtrSteerOutputBuffer[CComWheelset::LF] = finalSteerTorque[0];
    comWheelset_.mtrSteerOutputBuffer[CComWheelset::RF] = finalSteerTorque[1];
    comWheelset_.mtrSteerOutputBuffer[CComWheelset::LB] = finalSteerTorque[2];
    comWheelset_.mtrSteerOutputBuffer[CComWheelset::RB] = finalSteerTorque[3];


    // 填充电机发送缓冲区
    CDevMtrDJI::FillCanTxBuffer(comWheelset_.motor[CComWheelset::LF],
                                comWheelset_.mtrCanTxNode[CComWheelset::LF]->dataBuffer,
                                comWheelset_.mtrOutputBuffer[CComWheelset::LF]);
    CDevMtrDJI::FillCanTxBuffer(comWheelset_.motor[CComWheelset::RF],
                                comWheelset_.mtrCanTxNode[CComWheelset::RF]->dataBuffer,
                                comWheelset_.mtrOutputBuffer[CComWheelset::RF]);
    CDevMtrDJI::FillCanTxBuffer(comWheelset_.motor[CComWheelset::LB],
                                comWheelset_.mtrCanTxNode[CComWheelset::LB]->dataBuffer,
                                comWheelset_.mtrOutputBuffer[CComWheelset::LB]);
    CDevMtrDJI::FillCanTxBuffer(comWheelset_.motor[CComWheelset::RB],
                                comWheelset_.mtrCanTxNode[CComWheelset::RB]->dataBuffer,
                                comWheelset_.mtrOutputBuffer[CComWheelset::RB]);            

    CDevMtrDJI::FillCanTxBuffer(comWheelset_.steerMotor[CComWheelset::LF],
                                comWheelset_.mtrSteerCanTxNode[CComWheelset::LF]->dataBuffer,
                                comWheelset_.mtrSteerOutputBuffer[CComWheelset::LF]);
    CDevMtrDJI::FillCanTxBuffer(comWheelset_.steerMotor[CComWheelset::RF],
                                comWheelset_.mtrSteerCanTxNode[CComWheelset::RF]->dataBuffer,
                                comWheelset_.mtrSteerOutputBuffer[CComWheelset::RF]);
    CDevMtrDJI::FillCanTxBuffer(comWheelset_.steerMotor[CComWheelset::LB],
                                comWheelset_.mtrSteerCanTxNode[CComWheelset::LB]->dataBuffer,
                                comWheelset_.mtrSteerOutputBuffer[CComWheelset::LB]);
    CDevMtrDJI::FillCanTxBuffer(comWheelset_.steerMotor[CComWheelset::RB],
                                comWheelset_.mtrSteerCanTxNode[CComWheelset::RB]->dataBuffer,
                                comWheelset_.mtrSteerOutputBuffer[CComWheelset::RB]);

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

    // 限制底盘模块的控制命令大小
    // chassisCmd.speed_X = std::clamp(chassisCmd.speed_X, -100.0f, 100.0f);
    // chassisCmd.speed_Y = std::clamp(chassisCmd.speed_Y, -100.0f, 100.0f);
    // // 平面速度圆限幅：避免斜向输入时合速度超过100%
    // {
    //     const float planarMag = std::sqrt(chassisCmd.speed_X * chassisCmd.speed_X +
    //                                       chassisCmd.speed_Y * chassisCmd.speed_Y);
    //     if (planarMag > 100.0f) {
    //         const float scale = 100.0f / planarMag;
    //         chassisCmd.speed_X *= scale;
    //         chassisCmd.speed_Y *= scale;
    //     }
    // }
    // chassisCmd.speed_W = std::clamp(chassisCmd.speed_W, -100.0f, 100.0f);

    // 自动控制启用，则不继续做限制
    if (chassisCmd.isAutoCtrl) return APP_OK;

    return APP_OK;
}

} // namespace my_engineer
