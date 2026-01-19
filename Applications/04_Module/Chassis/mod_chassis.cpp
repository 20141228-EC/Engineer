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
    comHip_.InitComponent(param);

    chassisMaxPower_ = chassisParam.chassisMaxPower;            // 保存底盘总功率限制

    // 初始化4个电机的功率限制实例
    powerCtrlLF_.InitPowerControl(&chassisParam.powerParamLF);  // 左前电机功率初始化
    powerCtrlRF_.InitPowerControl(&chassisParam.powerParamRF);  // 右前电机功率初始化
    powerCtrlLB_.InitPowerControl(&chassisParam.powerParamLB);  // 左后电机功率初始化
    powerCtrlRB_.InitPowerControl(&chassisParam.powerParamRB);  // 右后电机功率初始化

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
float CModChassis::CalcTotalDemandPower(const CComWheelset& wheelset){

    float totalDemand = 0.0f;   ///< 需求总功率
    float torque[4] = {
        static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::LF]),
        static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::RF]),
        static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::LB]),
        static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::RB])
    };
    float speed[4] = {
        static_cast<float>(wheelset.motor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.motor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.motor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float>(wheelset.motor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED])
    };
    totalDemand += (powerCtrlLF_.CalcMotorPower(speed[0], torque[0]) > 0.0f) ? powerCtrlLF_.CalcMotorPower(speed[0], torque[0]) : 0.0f;
    totalDemand += (powerCtrlRF_.CalcMotorPower(speed[1], torque[1]) > 0.0f) ? powerCtrlRF_.CalcMotorPower(speed[1], torque[1]) : 0.0f;
    totalDemand += (powerCtrlLB_.CalcMotorPower(speed[2], torque[2]) > 0.0f) ? powerCtrlLB_.CalcMotorPower(speed[2], torque[2]) : 0.0f;
    totalDemand += (powerCtrlRB_.CalcMotorPower(speed[3], torque[3]) > 0.0f) ? powerCtrlRB_.CalcMotorPower(speed[3], torque[3]) : 0.0f;
    return totalDemand;
}

/**
 * @brief 动态功率分配
 * 
 * 
 */
void CModChassis::AllocDynamicPower(const CComWheelset& wheelset, float targetPower[4]) {

    float totalDemand = CalcTotalDemandPower(wheelset);
    const float maxTotal = static_cast<float>(chassisMaxPower_);

    // 初始化targetPower为固定值（单电机默认功率上限）
    targetPower[0] = maxTotal;
    targetPower[1] = maxTotal;
    targetPower[2] = maxTotal;
    targetPower[3] = maxTotal;

    // 功率超限才执行动态分配 否则不作限制
    if (totalDemand > maxTotal + 1e-6f) {
        float demand[4] = {
            powerCtrlLF_.CalcMotorPower(
                static_cast<float>(wheelset.motor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]),
                static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::LF])
            ),
            powerCtrlRF_.CalcMotorPower(
                static_cast<float>(wheelset.motor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]),
                static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::RF])
            ),
            powerCtrlLB_.CalcMotorPower(
                static_cast<float>(wheelset.motor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]),
                static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::LB])
            ),
            powerCtrlRB_.CalcMotorPower(
                static_cast<float>(wheelset.motor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]),
                static_cast<float>(wheelset.mtrOutputBuffer[CComWheelset::RB])
            )
        };

        // 计算总需求
        float totalAbsDemand = std::max(demand[0], 0.0f) + std::max(demand[1], 0.0f) + std::max(demand[2], 0.0f) + std::max(demand[3], 0.0f);
        if (totalAbsDemand < 1e-3f) {
            // 无有效需求时，平均分配总功率
            float avgPower = maxTotal / 4.0f;
            targetPower[0] = avgPower;
            targetPower[1] = avgPower;
            targetPower[2] = avgPower;
            targetPower[3] = avgPower;
        } else {
            // 按负载比例动态分配总功率
            targetPower[0] = (std::max(demand[0], 0.0f) / totalAbsDemand) * maxTotal;
            targetPower[1] = (std::max(demand[1], 0.0f) / totalAbsDemand) * maxTotal;
            targetPower[2] = (std::max(demand[2], 0.0f) / totalAbsDemand) * maxTotal;
            targetPower[3] = (std::max(demand[3], 0.0f) / totalAbsDemand) * maxTotal;
        }

        // 二次校准：消除浮点误差，确保总功率不超限
        float allocTotal = targetPower[0] + targetPower[1] + targetPower[2] + targetPower[3];
        if (allocTotal > maxTotal) {
            float ratio = maxTotal / allocTotal;
            for (int i = 0; i < 4; i++) {
                targetPower[i] *= ratio;
            }
        }
    }
}

/**
 * @brief 更新处理
 * 
 * @return EAppStatus 
 */
void CModChassis::UpdateHandler_(){

    // 检查模块状态
    if (moduleStatus == APP_RESET) return;

    static uint8_t HalfTickRate = 0;
	HalfTickRate = 1 - HalfTickRate;

    comHip_.MovMode_ = MovMode; ///< 更新面向底层髋关节组件的运动模式

    DataBuffer<float_t> roll_Target = {0.0f}; ///< 目标roll角度，目前暂时写这个，后续出车之后根据实际可能有些误差待改

    // 计算Roll角
    roll_Measure = {filter->Imu_Ave_Info.imu_ave_pitch * ROLL_LIFT_DIR};

    // 底盘roll轴是一个三环pid控制，最外环为控roll轴角度，输出目标腿长，内环是控腿长
    DataBuffer<float_t> roll_target_climbing;
    if(comHip_.MovMode_ == EmovMode::CLIMBING)
    {
        roll_target_climbing = comHip_.pidRollCtrl.UpdatePidController(roll_Target, roll_Measure);
        chassisCmd.L_length += roll_target_climbing[0] * ROLL_DEG_ECD_RATIO * ROLL_LIFT_DIR * 3.f / 1000.f / 10.f; ///< 在当前腿长目标基础上进行累加
    } 

    if(reset_hip){  // 要求复位腿
            chassisCmd.L_length = 0; ///< 直接回到初始化腿长
            comHip_.pidRollCtrl.ResetPidController(); ///< 同时重置PID控制器
            reset_hip = 0;      ///< 清空标志位
    }

    // 更新底盘轮组
    comWheelset_.UpdateComponent();
    // 更新髋关节
    if(HalfTickRate){comHip_.UpdateComponent();} ///< 降为500Hz

    // 功率分配
    float dynamicTargetPower[4] = {0.0f};
    AllocDynamicPower(comWheelset_, dynamicTargetPower); // 内部基于统一数据源计算

    // 设置电机功率上限
    powerCtrlLF_.SetDefaultMaxPower(static_cast<uint16_t>(dynamicTargetPower[0]));
    powerCtrlRF_.SetDefaultMaxPower(static_cast<uint16_t>(dynamicTargetPower[1]));
    powerCtrlLB_.SetDefaultMaxPower(static_cast<uint16_t>(dynamicTargetPower[2]));
    powerCtrlRB_.SetDefaultMaxPower(static_cast<uint16_t>(dynamicTargetPower[3]));

    int16_t limitedTorque[4];
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
    // 应用全局缩放，得到最终发送转矩
    int16_t finalTorque[4] = {
        static_cast<int16_t>(limitedTorque[0]),
        static_cast<int16_t>(limitedTorque[1]),
        static_cast<int16_t>(limitedTorque[2]),
        static_cast<int16_t>(limitedTorque[3])
    };

    // 更新最终转矩到输出缓冲区
    comWheelset_.mtrOutputBuffer[CComWheelset::LF] = finalTorque[0];
    comWheelset_.mtrOutputBuffer[CComWheelset::RF] = finalTorque[1];
    comWheelset_.mtrOutputBuffer[CComWheelset::LB] = finalTorque[2];
    comWheelset_.mtrOutputBuffer[CComWheelset::RB] = finalTorque[3];


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
    chassisCmd.speed_X = std::clamp(chassisCmd.speed_X, -100.0f, 100.0f);
    chassisCmd.speed_Y = std::clamp(chassisCmd.speed_Y, -100.0f, 100.0f);
    chassisCmd.speed_W = std::clamp(chassisCmd.speed_W, -100.0f, 100.0f);
    chassisCmd.L_length = std::clamp(chassisCmd.L_length, 0.f, 100.0f);

    // 自动控制启用，则不继续做限制
    if (chassisCmd.isAutoCtrl) return APP_OK;

    return APP_OK;
}

} // namespace my_engineer
