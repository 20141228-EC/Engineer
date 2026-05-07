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
    const float totalDemand = CalcTotalDemandPower(wheelset, wheelDemand, steerDemand);
    const float pMaxHard = std::min(static_cast<float>(chassisMaxPower_), dynamicPowerBudget_);// 硬功率上限：底盘最大功率和当前动态预算的较小值
    const float budgetFloor = static_cast<float>(chassisMaxPower_) * kFloorBudgetRatio_;       // 预算下限：底盘最大功率的固定占比，确保在极端情况下仍有基本的功率输出能力
    const float measuredPowerNow = std::max(power_measure_used, feedbackMeasuredPowerRaw_);    // 当前功率测量值：取实际测量的功率和基于反馈估算的瞬时功率中的较大值，作为当前的功率测量值
    constexpr float kTerrainPowerGuardGain = 1.25f; // 地形过载保护增益：当测量功率超过软预算或命令需求超过硬预算时，增加的保护力度，用于更积极地限制输出以应对可能的陡坡或突增负载情况
    constexpr float kTerrainDemandGuardGain = 0.55f;// 需求过载保护增益：当命令需求超过软预算时，增加的保护力度，用于限制输出以防止过度追求性能导致的过载情况
    const float terrainGuardBudget = std::clamp(
        pMaxHard - kTerrainPowerGuardGain * std::max(0.0f, measuredPowerNow - softPowerBudget_) - kTerrainDemandGuardGain * std::max(0.0f, totalDemand - pMaxHard),
        budgetFloor,
        pMaxHard);// 地形过载保护预算：在硬功率上限的基础上，根据当前测量功率超过软预算的程度和命令需求超过硬预算的程度，动态调整的保护预算，用于在陡坡或突增负载情况下更积极地限制输出，防止过载，同时保证不低于预算下限
    const float guardedPMaxHard = std::min(pMaxHard, terrainGuardBudget);
    const float pMaxSoft = std::min(guardedPMaxHard, softPowerBudget_);
    constexpr float eps = 1e-6f;
    power_guard_budget = guardedPMaxHard;

    float wheelDemandTotal = 0.0f;
    float steerDemandTotal = 0.0f;
    for (int i = 0; i < 4; i++) {
        wheelDemandTotal += wheelDemand[i];
        steerDemandTotal += steerDemand[i];
    }

    float wheelLimitCoe = 1.0f;
    float steerLimitCoe = 1.0f;

    if (totalDemand > pMaxSoft + eps) {
        float steerErrMean = 0.0f;
        float steerErrMax = 0.0f;
        for (int i = 0; i < 4; i++) {
            steerErrMean += wheelset.steerErrRad[i];
            steerErrMax = std::max(steerErrMax, wheelset.steerErrRad[i]);
        }
        steerErrMean *= 0.25f; // 平均舵轮误差：4个舵轮误差的平均值，反映整体的舵轮对齐情况，用于动态调整功率分配策略，确保在舵轮误差较大时优先保证舵向控制能力

        const float steerAlignNeed = std::clamp((0.65f * steerErrMean + 0.35f * steerErrMax) / static_cast<float>(deg2rad(50.0f)), 0.0f, 1.0f); // 舵轮对齐需求：综合考虑平均舵轮误差和最大舵轮误差，计算得到的一个0到1之间的值，反映当前的舵轮对齐需求程度，用于动态调整功率分配策略，确保在舵轮误差较大时优先保证舵向控制能力
        constexpr float kSteerRatioMin = 0.08f;
        constexpr float kSteerRatioMax = 0.20f;
        const float steerBudgetRatio = kSteerRatioMin + (kSteerRatioMax - kSteerRatioMin) * steerAlignNeed;// 舵向预算占比：根据舵轮对齐需求动态调整的舵向功率占比，确保在舵轮误差较大时分配更多的功率给舵向控制，以提高底盘的稳定性和响应性

        float steerBudgetHard = 0.0f;
        float wheelBudgetHard = 0.0f;
        if (steerDemandTotal <= eps) {
            wheelBudgetHard = guardedPMaxHard;
        } else if (wheelDemandTotal <= eps) {
            steerBudgetHard = guardedPMaxHard;
        } else {
            steerBudgetHard = std::min(steerDemandTotal, steerBudgetRatio * guardedPMaxHard);
            wheelBudgetHard = guardedPMaxHard - steerBudgetHard;

            if (wheelDemandTotal < wheelBudgetHard) {
                const float spareBudget = wheelBudgetHard - wheelDemandTotal;
                wheelBudgetHard = wheelDemandTotal;
                steerBudgetHard = std::min(steerDemandTotal, steerBudgetHard + spareBudget);// 当车轮需求未满额定预算时，回收多余预算给舵向
            } else if (steerDemandTotal < steerBudgetHard) {
                const float spareBudget = steerBudgetHard - steerDemandTotal;
                steerBudgetHard = steerDemandTotal;
                wheelBudgetHard = std::min(wheelDemandTotal, wheelBudgetHard + spareBudget);// 当舵向需求未满额定预算时，回收多余预算给车轮
            }
        }

        const float hardWheelLimitCoe = std::clamp(wheelBudgetHard / std::max(wheelDemandTotal, eps), 0.0f, 1.0f);
        const float hardSteerLimitCoe = std::clamp(steerBudgetHard / std::max(steerDemandTotal, eps), 0.0f, 1.0f);
        const float softBlend = std::clamp((totalDemand - pMaxSoft) / std::max(guardedPMaxHard - pMaxSoft, eps), 0.0f, 1.0f);

        wheelLimitCoe = 1.0f - softBlend * (1.0f - hardWheelLimitCoe);
        steerLimitCoe = 1.0f - softBlend * (1.0f - hardSteerLimitCoe);
    }

    targetWheelPower[CComWheelset::LF] = wheelDemand[CComWheelset::LF] * wheelLimitCoe;
    targetWheelPower[CComWheelset::RF] = wheelDemand[CComWheelset::RF] * wheelLimitCoe;
    targetWheelPower[CComWheelset::LB] = wheelDemand[CComWheelset::LB] * wheelLimitCoe;
    targetWheelPower[CComWheelset::RB] = wheelDemand[CComWheelset::RB] * wheelLimitCoe;
    
    const bool highPowerMode = std::max(measuredPowerNow, totalDemand) > 80.0f;
    const float wheelSpeedAbs[4] = {
        std::fabs(static_cast<float>(wheelset.motor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED])),
        std::fabs(static_cast<float>(wheelset.motor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED])),
        std::fabs(static_cast<float>(wheelset.motor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED])),
        std::fabs(static_cast<float>(wheelset.motor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]))
    };
    bool flag = 0;
    for(int i = 0; i < 4; i++)if (highPowerMode && wheelSpeedAbs[i] < 60.0f) {flag = 1; break;}
            
    if (flag && wheelDemandTotal > eps) {
        constexpr float kSlowSpeedRef = 60.0f; // 慢速参考速度
        constexpr float kSlowPriorityBlend = 0.70f;
        constexpr float kSlowPriorityBase = 0.30f; // 基础占比，防止在高速时完全不考虑慢速优先级，导致功率分配过于极端



        float slowPriority[4] = {0.0f};     // 慢速优先级
        float slowPrioritySum = 0.0f;
        for (int i = 0; i < 4; i++) {
            const float slowScore = std::clamp(1.0f - wheelSpeedAbs[i] / kSlowSpeedRef, 0.0f, 1.0f);
            slowPriority[i] = kSlowPriorityBase + slowScore;
            slowPrioritySum += slowPriority[i];
        }

        if (slowPrioritySum > eps) {
            float priorityWheelPower[4] = {0.0f};
            for (int i = 0; i < 4; i++) {
                priorityWheelPower[i] = wheelDemandTotal * slowPriority[i] / slowPrioritySum;
                targetWheelPower[i] = (1.0f - kSlowPriorityBlend) * targetWheelPower[i] + kSlowPriorityBlend * priorityWheelPower[i];
            }
        }
    }

    targetSteerPower[CComWheelset::LF] = steerDemand[CComWheelset::LF] * steerLimitCoe;
    targetSteerPower[CComWheelset::RF] = steerDemand[CComWheelset::RF] * steerLimitCoe;
    targetSteerPower[CComWheelset::LB] = steerDemand[CComWheelset::LB] * steerLimitCoe;
    targetSteerPower[CComWheelset::RB] = steerDemand[CComWheelset::RB] * steerLimitCoe;

    // 舵向功率保底：避免总功率受限时舵轮因预算过低而无法回正
    constexpr float kSteerMinPowerPerWheel = 4.0f;
    constexpr float kSteerDemandEnableTh = 0.5f;
    float steerBoostPower = 0.0f;
    for (int i = 0; i < 4; i++) {
        if (steerDemand[i] > kSteerDemandEnableTh) {
            const float boosted = std::max(targetSteerPower[i], kSteerMinPowerPerWheel);
            steerBoostPower += (boosted - targetSteerPower[i]);
            targetSteerPower[i] = boosted;
        }
    }

    // 从驱动预算中等额回收，保持总功率预算不被破坏
    if (steerBoostPower > eps) {
        float wheelTargetTotal = 0.0f;
        for (int i = 0; i < 4; i++) {
            wheelTargetTotal += targetWheelPower[i];
        }

        if (wheelTargetTotal > eps) {
            const float reducePower = std::min(steerBoostPower, wheelTargetTotal);
            const float wheelScale = (wheelTargetTotal - reducePower) / wheelTargetTotal;
            for (int i = 0; i < 4; i++) {
                targetWheelPower[i] *= wheelScale;
            }
        }
    }

    float targetTotal = 0.0f;
    for (int i = 0; i < 4; i++) {
        targetTotal += targetWheelPower[i] + targetSteerPower[i];
    }
    if (targetTotal > guardedPMaxHard + eps) {
        const float totalScale = guardedPMaxHard / targetTotal;
        for (int i = 0; i < 4; i++) {
            targetWheelPower[i] *= totalScale;
            targetSteerPower[i] *= totalScale;
        }
    }
}

void CModChassis::UpdatePowerBudget_() {
    if (SysReferee.refereeInfo.robot.robotMaxPower > 0) {
        chassisMaxPower_ = static_cast<uint16_t>(SysReferee.refereeInfo.robot.robotMaxPower);
        max_power = chassisMaxPower_;
    }

    constexpr float kPowerLpfAlpha = 0.12f;                 // 功率测量低通滤波系数
    constexpr float kFeedbackGain = 2.8f;                   // 基于功率反馈的限制增益
    constexpr float kEmergencyOverGain = 4.5f;              // 紧急过载限制增益
    constexpr float kEmergencyRiseGain = 1.5f;              // 紧急上升限制增益
    constexpr float kPredictiveCmdGain = 0.65f;             // 预测命令增益
    constexpr float kTerrainPenaltyPowerGain = 1.10f;       // 地形惩罚功率增益
    constexpr float kTerrainPenaltyDemandGain = 0.45f;      // 地形惩罚需求增益
    constexpr float kTerrainPenaltyAttackAlpha = 0.35f;     // 地形惩罚攻击Alpha
    constexpr float kTerrainPenaltyReleaseAlpha = 0.06f;    // 地形惩罚释放Alpha
    const float hardPowerLimit = static_cast<float>(chassisMaxPower_);
    const float budgetFloor = hardPowerLimit * kFloorBudgetRatio_;
    const float budgetWarn = hardPowerLimit * kWarnBudgetRatio_;
    const uint32_t now = HAL_GetTick();
    const bool powerMeterOnline = (powerMeterLastTimestamp_ != 0U)
        && ((now - powerMeterLastTimestamp_) <= kPowerMeterOfflineTimeoutMs_);
    const float measuredPower = powerMeterOnline ? std::max(0.0f, powermeter) : std::max(0.0f, feedbackMeasuredPower_);
    const float measuredPowerPeak = std::max(measuredPower, feedbackMeasuredPowerRaw_); // 当前功率峰值：取测量功率和基于反馈估算的瞬时功率中的较大值，作为当前的功率峰值，用于更敏感地捕捉可能的过载情况
    power_measure_used = measuredPower;

    if (!powerBudgetInitialized_) { // 功率预算未初始化，使用测量值进行初始设定，避免初始阶段过度限制
        measuredPowerLpf_ = measuredPower;
        lastMeasuredPower_ = measuredPower;
        terrainOverloadPenalty_ = 0.0f;
        dynamicPowerBudget_ = hardPowerLimit;
        softPowerBudget_ = hardPowerLimit * kSoftLimitRatio_;
        power_budget = dynamicPowerBudget_;
        power_buffer_est = 0.0f;
        power_guard_budget = dynamicPowerBudget_;
        powerBudgetInitialized_ = true;
        return;
    }

    measuredPowerLpf_ += kPowerLpfAlpha * (measuredPower - measuredPowerLpf_); // 功率测量低通滤波
    const float powerRise = std::max(0.0f, measuredPower - lastMeasuredPower_);// 功率上升速率：当前测量功率与上一拍测量功率的差值，反映功率的上升趋势，用于检测突发过载情况

    const float predictiveOverPower = std::max(
        std::max(0.0f, power_cmd_total - softPowerBudget_),
        0.8f * std::max(0.0f, power_demand_total - budgetWarn)); // 预测过载功率：基于当前命令和需求相对于预算的超出程度，给予一定权重，提前预判可能的过载情况
    const float terrainTargetPenalty = std::clamp(
        kTerrainPenaltyPowerGain * std::max(0.0f, measuredPowerPeak - budgetWarn)
        + kTerrainPenaltyDemandGain * std::max(0.0f, power_demand_total - hardPowerLimit),
        0.0f,
        hardPowerLimit - budgetFloor); // 地形过载目标惩罚：根据当前功率峰值和需求相对于预算的超出程度，计算一个目标惩罚值，用于动态调整功率预算以应对复杂地形带来的过载风险
    const float terrainPenaltyAlpha = (terrainTargetPenalty > terrainOverloadPenalty_)
        ? kTerrainPenaltyAttackAlpha
        : kTerrainPenaltyReleaseAlpha;
    terrainOverloadPenalty_ += terrainPenaltyAlpha * (terrainTargetPenalty - terrainOverloadPenalty_); // 地形过载惩罚：当前的地形过载惩罚值逐渐逼近目标惩罚值，攻击和释放速率不同，以实现既能快速响应突发过载又能平滑恢复的效果

    const float feedbackLimitedPower = std::clamp(
        hardPowerLimit - kFeedbackGain * std::max(0.0f, measuredPowerLpf_ - hardPowerLimit),
        budgetFloor,
        hardPowerLimit); // 反馈限制功率：基于测量功率的低通滤波值相对于硬限制的超出程度，给予一定增益的限制，防止持续过载，同时保持一定的缓冲空间避免频繁触发过载保护

    const float emergencyLimitedPower = std::clamp(
        hardPowerLimit - kEmergencyOverGain * std::max(0.0f, measuredPowerPeak - hardPowerLimit) - kEmergencyRiseGain * std::max(0.0f, powerRise - 4.0f),
        budgetFloor,
        hardPowerLimit); // 紧急限制功率：针对突发过载情况的快速响应机制，基于功率峰值和功率上升速率的超出程度，给予较高增益的限制，能够在检测到可能的过载风险时迅速降低功率预算

    const float predictiveLimitedPower = std::clamp(
        hardPowerLimit - kPredictiveCmdGain * predictiveOverPower,
        budgetFloor,
        hardPowerLimit); // 预测限制功率：基于对当前命令和需求的预测过载情况，给予一定增益的限制，能够提前调整功率预算以应对即将到来的过载风险，提升系统的预见性和稳定性

    const float terrainLimitedPower = std::clamp(
        hardPowerLimit - terrainOverloadPenalty_,
        budgetFloor,
        hardPowerLimit); // 地形限制功率：根据当前的地形过载惩罚值，直接从硬限制中扣除，动态调整功率预算以适应复杂地形带来的过载风险，确保在恶劣地形条件下系统的安全和稳定

    dynamicPowerBudget_ = std::clamp(
        std::min(std::min(feedbackLimitedPower, emergencyLimitedPower), std::min(predictiveLimitedPower, terrainLimitedPower)),
        budgetFloor,
        hardPowerLimit);
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
    // comWheelset_.mtrOutputBuffer[CComWheelset::LF] = finalTorque[0];
    // comWheelset_.mtrOutputBuffer[CComWheelset::RF] = finalTorque[1];
    // comWheelset_.mtrOutputBuffer[CComWheelset::LB] = finalTorque[2];
    // comWheelset_.mtrOutputBuffer[CComWheelset::RB] = finalTorque[3];
    // comWheelset_.mtrSteerOutputBuffer[CComWheelset::LF] = finalSteerTorque[0];
    // comWheelset_.mtrSteerOutputBuffer[CComWheelset::RF] = finalSteerTorque[1];
    // comWheelset_.mtrSteerOutputBuffer[CComWheelset::LB] = finalSteerTorque[2];
    // comWheelset_.mtrSteerOutputBuffer[CComWheelset::RB] = finalSteerTorque[3];


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
