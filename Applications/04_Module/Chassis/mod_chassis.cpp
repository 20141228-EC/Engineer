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
        static_cast<float>(wheelset.mtrSteerOutputBuffer[CComWheelset::LF])) * 3.f);
    steerDemand[CComWheelset::RF] = std::max(0.0f, powerCtrlSteerRF_.CalcMotorPower(
        static_cast<float>(wheelset.steerMotor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.mtrSteerOutputBuffer[CComWheelset::RF])) * 3.f);
    steerDemand[CComWheelset::LB] = std::max(0.0f, powerCtrlSteerLB_.CalcMotorPower(
        static_cast<float>(wheelset.steerMotor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.mtrSteerOutputBuffer[CComWheelset::LB])) * 3.f);
    steerDemand[CComWheelset::RB] = std::max(0.0f, powerCtrlSteerRB_.CalcMotorPower(
        static_cast<float>(wheelset.steerMotor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.mtrSteerOutputBuffer[CComWheelset::RB])) * 3.f);
        // 舵电机预测乘原始3倍

    for (int i = 0; i < 4; i++) {
        totalDemand += wheelDemand[i] + steerDemand[i];
    }

    return totalDemand;
}


// 动态分配功率
void CModChassis::AllocDynamicPower(const CComWheelset& wheelset,
                                    float targetWheelPower[4],
                                    float targetSteerPower[4])
{
    // 1. 计算轮向和舵向需求功率
    float wheelDemand[4] = {0};
    float steerDemand[4] = {0};
    CalcTotalDemandPower(wheelset, wheelDemand, steerDemand);

    float totalWheelDemand = wheelDemand[0]+wheelDemand[1]+wheelDemand[2]+wheelDemand[3];
    float totalSteerDemand = steerDemand[0]+steerDemand[1]+steerDemand[2]+steerDemand[3];
    float totalDemand = totalWheelDemand + totalSteerDemand;

    // 2. 总功率限制
    constexpr float kTotalPower = 115.f;    // 总功率上限
    constexpr float kMinBufferLimit = 20.f; // 缓冲低于该值主动限制

    // 3. 默认输出等于需求
    for(int i=0;i<4;i++){
        targetWheelPower[i] = wheelDemand[i];
        targetSteerPower[i] = steerDemand[i];
    }

    // 4. 获取实时缓冲能量
    float buffer = SysReferee.refereeInfo.energy.buffer_energy;

    // 5. 判断是否需要主动削减功率
    bool activeLimit = (totalDemand > kTotalPower) || (buffer < kMinBufferLimit);

    if(activeLimit){
        // 轮向优先
        float wheelScale = (totalWheelDemand > 1e-6f) ? std::max(0.0f, (kTotalPower - totalSteerDemand) / totalWheelDemand) : 0.0f;
        for(int i=0;i<4;i++) targetWheelPower[i] *= wheelScale;

        // 舵向按剩余功率比例缩放
        float steerTotal = steerDemand[0]+steerDemand[1]+steerDemand[2]+steerDemand[3];
        float availableSteerPower = kTotalPower - (targetWheelPower[0]+targetWheelPower[1]+targetWheelPower[2]+targetWheelPower[3]);
        float steerScale = (steerTotal > 1e-6f) ? std::max(0.0f, availableSteerPower / steerTotal) : 0.0f;
        for(int i=0;i<4;i++) targetSteerPower[i] *= steerScale;
    }

    // 不再在本地修改缓冲能量，裁判系统会更新
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

    if(moduleStatus == APP_RESET) return;

    // ------------------ 读取功率计 ------------------
    if(powerMeterRxNode_.timestamp > powerMeterLastTimestamp_ &&
       powerMeterRxNode_.dataBuffer.size() >= sizeof(float)){
        std::memcpy(&powermeter, powerMeterRxNode_.dataBuffer.data(), sizeof(float));
        powerMeterLastTimestamp_ = powerMeterRxNode_.timestamp;
    }

    // ------------------ 计算预测功率 ------------------
    auto calcFeedbackPower = [](CAlgoPowerControl &ctrl, CDevMtr *motor) -> float{
        if(!motor) return 0.0f;
        return std::max(0.0f, ctrl.CalcMotorPower(
            static_cast<float>(motor->motorData[CDevMtr::DATA_SPEED]),
            static_cast<float>(motor->motorData[CDevMtr::DATA_CURRENT])
        ));
    };

    float feedbackPowerRaw =
        calcFeedbackPower(powerCtrlLF_, comWheelset_.motor[CComWheelset::LF]) +
        calcFeedbackPower(powerCtrlRF_, comWheelset_.motor[CComWheelset::RF]) +
        calcFeedbackPower(powerCtrlLB_, comWheelset_.motor[CComWheelset::LB]) +
        calcFeedbackPower(powerCtrlRB_, comWheelset_.motor[CComWheelset::RB]) +
        calcFeedbackPower(powerCtrlSteerLF_, comWheelset_.steerMotor[CComWheelset::LF]) +
        calcFeedbackPower(powerCtrlSteerRF_, comWheelset_.steerMotor[CComWheelset::RF]) +
        calcFeedbackPower(powerCtrlSteerLB_, comWheelset_.steerMotor[CComWheelset::LB]) +
        calcFeedbackPower(powerCtrlSteerRB_, comWheelset_.steerMotor[CComWheelset::RB]);

    feedbackMeasuredPowerRaw_ = feedbackPowerRaw;
    feedbackMeasuredPower_ = feedbackPowerRaw;
    power_feedback_est = feedbackPowerRaw;

    // ------------------ 更新功率预算 ------------------
    UpdatePowerBudget_();

    // ------------------ 更新舵轮 ------------------
    comWheelset_.UpdateComponent();

    // ------------------ 动态功率分配 ------------------
    float dynamicWheelPower[4]={0.0f}, dynamicSteerPower[4]={0.0f};
    AllocDynamicPower(comWheelset_, dynamicWheelPower, dynamicSteerPower);

    auto toPowerUInt = [](float p){ return static_cast<uint16_t>(std::lround(std::max(0.0f,p))); };

    // 设置轮向电机功率上限
    powerCtrlLF_.SetDefaultMaxPower(toPowerUInt(dynamicWheelPower[CComWheelset::LF]));
    powerCtrlRF_.SetDefaultMaxPower(toPowerUInt(dynamicWheelPower[CComWheelset::RF]));
    powerCtrlLB_.SetDefaultMaxPower(toPowerUInt(dynamicWheelPower[CComWheelset::LB]));
    powerCtrlRB_.SetDefaultMaxPower(toPowerUInt(dynamicWheelPower[CComWheelset::RB]));

    // 设置舵向电机功率上限
    powerCtrlSteerLF_.SetDefaultMaxPower(toPowerUInt(dynamicSteerPower[CComWheelset::LF]));
    powerCtrlSteerRF_.SetDefaultMaxPower(toPowerUInt(dynamicSteerPower[CComWheelset::RF]));
    powerCtrlSteerLB_.SetDefaultMaxPower(toPowerUInt(dynamicSteerPower[CComWheelset::LB]));
    powerCtrlSteerRB_.SetDefaultMaxPower(toPowerUInt(dynamicSteerPower[CComWheelset::RB]));

    // ------------------ 限幅轮向输出 ------------------
   for(int i = 0; i < 4; i++){
        float wheelSpeed = static_cast<float>(comWheelset_.motor[i]->motorData[CDevMtr::DATA_SPEED]);
        int16_t limitedTorque = (&powerCtrlLF_ + i)->UpdatePowerLimit(wheelSpeed, comWheelset_.mtrOutputBuffer[i]);
        comWheelset_.mtrOutputBuffer[i] = limitedTorque;
    }

    // ------------------ 填充CAN发送 ------------------
    for(int i = 0; i < 4; i++){
        CDevMtrDJI::FillCanTxBuffer(comWheelset_.motor[i],
                                    comWheelset_.mtrCanTxNode[i]->dataBuffer,
                                    comWheelset_.mtrOutputBuffer[i]);
        CDevMtrDJI::FillCanTxBuffer(comWheelset_.steerMotor[i],
                                    comWheelset_.mtrSteerCanTxNode[i]->dataBuffer,
                                    comWheelset_.mtrSteerOutputBuffer[i]);
    }
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
    const float planarMag = std::sqrt(chassisCmd.speed_X * chassisCmd.speed_X +
                                          chassisCmd.speed_Y * chassisCmd.speed_Y);
    if (planarMag > 440.0f) {
        const float scale = 440.0f / planarMag;
        chassisCmd.speed_X *= scale;
        chassisCmd.speed_Y *= scale;
    }

    // 急停的时候晚一点跟云台
    static float lastPlanarMag = 0.0f;
    static int brakeHoldCnt = 0;

    constexpr float BRAKE_SPEED_TH = 150.0f;  // 上一瞬间目标速度大于这个，认为之前速度很大
    constexpr float STOP_SPEED_TH  = 40.0f;   // 当前目标速度小于这个，认为急停
    constexpr int   BRAKE_HOLD_TICK = 200;     // 200ms缓冲

    bool hardBrake = (lastPlanarMag > BRAKE_SPEED_TH &&
                      planarMag < STOP_SPEED_TH);

    if (hardBrake) {
        brakeHoldCnt = BRAKE_HOLD_TICK;
    }

    if (brakeHoldCnt > 0) {
        chassisCmd.speed_W = 0.0f;
        brakeHoldCnt--;
    }

    lastPlanarMag = planarMag;

    // 自动控制启用，则不继续做限制
    if (chassisCmd.isAutoCtrl) return APP_OK;

    return APP_OK;
}

} // namespace my_engineer
