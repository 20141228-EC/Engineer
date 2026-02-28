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

float wheel_power_lf = 0.0f;
float wheel_power_rf = 0.0f;
float wheel_power_lb = 0.0f;
float wheel_power_rb = 0.0f;
float wheel_torque_lf = 0.0f;
float wheel_torque_rf = 0.0f;
float wheel_torque_lb = 0.0f;
float wheel_torque_rb = 0.0f;
float_t crawler_torque_l = 0.0f;
float_t crawler_torque_r = 0.0f;
float raw_torque_LL = 0.0f;
float raw_torque_LR = 0.0f;
float actual_torque_LL = 0.0f;
float actual_torque_LR = 0.0f;
float_t raw_speed_LL = 0.0f;

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
    comHip_.parent = this;  ///< 初始化父类指针
    comCrawler_.InitComponent(param);
    comCrawler_.parent = this;

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

        // 前轮多分功率版
        const float front_weight = 1.f; // 1000.f 定义前轮权重
        const float back_weight = 1.5f;

        // 计算加权后的各轮需求
        float weighted_demand_lf = std::max(demand[0], 0.0f) * front_weight;
        float weighted_demand_rf = std::max(demand[1], 0.0f) * front_weight;
        float weighted_demand_lb = std::max(demand[2], 0.0f) * back_weight;
        float weighted_demand_rb = std::max(demand[3], 0.0f) * back_weight;

        // 2. 计算加权后的总需求
        float total_weighted_demand = weighted_demand_lf + weighted_demand_rf + weighted_demand_lb + weighted_demand_rb;

        if (total_weighted_demand < 1e-3f) {
            // 无有效需求时，平均分配总功率
            float avgPower = maxTotal / 4.0f;
            targetPower[0] = avgPower;
            targetPower[1] = avgPower;
            targetPower[2] = avgPower;
            targetPower[3] = avgPower;
        } else {
            // 3. 按加权后的比例分配总功率
            targetPower[0] = (weighted_demand_lf / total_weighted_demand) * maxTotal;
            targetPower[1] = (weighted_demand_rf / total_weighted_demand) * maxTotal;
            targetPower[2] = (weighted_demand_lb / total_weighted_demand) * maxTotal;
            targetPower[3] = (weighted_demand_rb / total_weighted_demand) * maxTotal;
        }

        // // 计算总需求
        // float totalAbsDemand = std::max(demand[0], 0.0f) + std::max(demand[1], 0.0f) + std::max(demand[2], 0.0f) + std::max(demand[3], 0.0f);
        // if (totalAbsDemand < 1e-3f) {
        //     // 无有效需求时，平均分配总功率
        //     float avgPower = maxTotal / 4.0f;
        //     targetPower[0] = avgPower;
        //     targetPower[1] = avgPower;
        //     targetPower[2] = avgPower;
        //     targetPower[3] = avgPower;
        // } else {
        //     // 按负载比例动态分配总功率
        //     targetPower[0] = (std::max(demand[0], 0.0f) / totalAbsDemand) * maxTotal;
        //     targetPower[1] = (std::max(demand[1], 0.0f) / totalAbsDemand) * maxTotal;
        //     targetPower[2] = (std::max(demand[2], 0.0f) / totalAbsDemand) * maxTotal;
        //     targetPower[3] = (std::max(demand[3], 0.0f) / totalAbsDemand) * maxTotal;
        // }

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

    // 用于无符号类型的转化
    static auto uint_to_float = [](uint16_t x_uint, float xmin, float xmax, uint8_t bits) -> float {
        float span = xmax - xmin;
        float data_norm = static_cast<float>(x_uint) / ((1 << bits) - 1);
        return data_norm * span + xmin;
    };

    // 用于有符号类型的转化
    static auto int_to_float = [](int16_t x_int, float xmin, float xmax, uint8_t bits) -> float {
        float span = xmax - xmin;
        // 计算有符号数的最大值：2^(bits-1) - 1 （12位则为2047）
        int32_t int_max = (1 << (bits - 1)) - 1;
        // 有符号数归一化：映射到[-1, 1]区间，再缩放至[xmin, xmax]
        float data_norm = static_cast<float>(x_int) / static_cast<float>(int_max);
        return (data_norm + 1.0f) * 0.5f * span + xmin;
    };

    static uint8_t HalfTickRate = 0;
	HalfTickRate = 1 - HalfTickRate;

    comHip_.MovMode_ = MovMode; ///< 更新面向底层髋关节组件的运动模式

    DataBuffer<float_t> roll_Target = {-4.0f}; ///< 目标roll角度，目前暂时写这个，后续出车之后根据实际可能有些误差待改

    // 计算Roll角
    roll_Measure = {filter->Imu_Ave_Info.imu_ave_pitch};

    // 底盘roll轴是一个三环pid控制，最外环为控roll轴角度，输出目标腿长，内环是控腿长
    DataBuffer<float_t> roll_target_climbing;
    if(comHip_.MovMode_ == EmovMode::CLIMBING)
    {   
        if(reset_hip){  // 要求复位腿
            chassisCmd.L_length = 0; ///< 直接回到初始化腿长
            roll_target_climbing = {0};
            comHip_.pidRollCtrl.ResetPidController(); ///< 同时重置PID控制器
        }
        else{
            roll_target_climbing = comHip_.pidRollCtrl.UpdatePidController(roll_Target, roll_Measure);
            chassisCmd.L_length += roll_target_climbing[0] * ROLL_DEG_ECD_RATIO * ROLL_LIFT_DIR * 3.f / 1000.f / 10.f; ///< 在当前腿长目标基础上进行累加
        }
    } 
    else if(MovMode == EmovMode::NORMAL && reset_hip){   ///< 普通行进模式下复位腿标志位用一次清一次
            chassisCmd.L_length = 0; ///< 直接回到初始化腿长
            comHip_.pidRollCtrl.ResetPidController(); ///< 同时重置PID控制器
            reset_hip = 0;      ///< 清空标志位
    }

    if(crawler_on){
        comCrawler_.CrawlerCmd.speed_crawler = 60.f * 30.f;
    }
    else{
        comCrawler_.CrawlerCmd.speed_crawler = 0;
        comCrawler_.PidCrawlerSpdCtrl.ResetPidController();
        comCrawler_.mtrOutputBuffer.fill(0);    ///< 卸力
    }

    // 更新底盘轮组
    comWheelset_.UpdateComponent();
   // 更新髋关节
   if(HalfTickRate){comHip_.UpdateComponent();} ///< 降为500Hz
    // 更新履带组件
    comCrawler_.UpdateComponent();

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

    /**********************用于debug start*****************************/
    // 更新全局变量以供调试
    wheel_power_lf = powerCtrlLF_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::LF]->motorData[CDevMtr::DATA_SPEED]), limitedTorque[0]);
    wheel_power_rf = powerCtrlRF_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::RF]->motorData[CDevMtr::DATA_SPEED]), limitedTorque[1]);
    wheel_power_lb = powerCtrlLB_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::LB]->motorData[CDevMtr::DATA_SPEED]), limitedTorque[2]);
    wheel_power_rb = powerCtrlRB_.CalcMotorPower(static_cast<float>(comWheelset_.motor[CComWheelset::RB]->motorData[CDevMtr::DATA_SPEED]), limitedTorque[3]);

    // 定义转换系数0000000000
    const float FEEDBACK_TO_AMP_RATIO = 20.0f / 16384.0f; // 反馈电流值到安培的转换系数
    const float TORQUE_CONSTANT_NM_PER_A = 0.3f;          // 电机扭矩常数 (N·m/A)

    // 计算每个电机轴上的实际物理扭矩 (N·m)
    wheel_torque_lf = static_cast<float>(comWheelset_.motor[CComWheelset::LF]->motorData[CDevMtr::DATA_CURRENT]) * FEEDBACK_TO_AMP_RATIO * TORQUE_CONSTANT_NM_PER_A;
    wheel_torque_rf = static_cast<float>(comWheelset_.motor[CComWheelset::RF]->motorData[CDevMtr::DATA_CURRENT]) * FEEDBACK_TO_AMP_RATIO * TORQUE_CONSTANT_NM_PER_A;
    wheel_torque_lb = static_cast<float>(comWheelset_.motor[CComWheelset::LB]->motorData[CDevMtr::DATA_CURRENT]) * FEEDBACK_TO_AMP_RATIO * TORQUE_CONSTANT_NM_PER_A;
    wheel_torque_rb = static_cast<float>(comWheelset_.motor[CComWheelset::RB]->motorData[CDevMtr::DATA_CURRENT]) * FEEDBACK_TO_AMP_RATIO * TORQUE_CONSTANT_NM_PER_A;
    
    crawler_torque_l = static_cast<float>(comCrawler_.motor[CComCrawler::L]->motorData[CDevMtr::DATA_CURRENT]) * FEEDBACK_TO_AMP_RATIO * TORQUE_CONSTANT_NM_PER_A;
    crawler_torque_r = static_cast<float>(comCrawler_.motor[CComCrawler::R]->motorData[CDevMtr::DATA_CURRENT]) * FEEDBACK_TO_AMP_RATIO * TORQUE_CONSTANT_NM_PER_A;

    // 应用全局缩放，得到最终发送转矩
    int16_t finalTorque[4] = {
        static_cast<int16_t>(limitedTorque[0]),
        static_cast<int16_t>(limitedTorque[1]),
        static_cast<int16_t>(limitedTorque[2]), 
        static_cast<int16_t>(limitedTorque[3])
    };

    auto pMtr_Hip_LL = comHip_.motor[0];
    auto pMtr_Hip_LR = comHip_.motor[1];
    // 读取左髋关节电机的实际输出力矩 (N·m)
    raw_torque_LL = int_to_float(
        pMtr_Hip_LL->motorData[CDevMtr::DATA_TORQUE],
        -54, // 扭矩下限
        54,  // 扭矩上限
        12   // 扭矩数据是12位
    );
    actual_torque_LL = abs(54 - raw_torque_LL) * ((raw_torque_LL - 54) > 0 ? 1 : -1);
    // 读取右髋关节电机的实际输出力矩 (N·m)
    raw_torque_LR = int_to_float(
        pMtr_Hip_LR->motorData[CDevMtr::DATA_TORQUE],
        -54,
        54,
        12
    );
    actual_torque_LR = abs(raw_torque_LR - 54) * ((raw_torque_LR - 54) > 0 ? 1 : -1);

    /**********************用于debug end*****************************/

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
    CDevMtrDJI::FillCanTxBuffer(comCrawler_.motor[CComCrawler::L],
                                comCrawler_.mtrCanTxNode[CComCrawler::L]->dataBuffer,
                                comCrawler_.mtrOutputBuffer[CComCrawler::L]);
    CDevMtrDJI::FillCanTxBuffer(comCrawler_.motor[CComCrawler::R],
                                comCrawler_.mtrCanTxNode[CComCrawler::R]->dataBuffer,
                                comCrawler_.mtrOutputBuffer[CComCrawler::R]);

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
    chassisCmd.L_length = std::clamp(chassisCmd.L_length, 0.f, 1680.f);
    chassisCmd.speed_crawler = std::clamp(chassisCmd.speed_crawler, -100.f, 100.f);

    // 自动控制启用，则不继续做限制
    if (chassisCmd.isAutoCtrl) return APP_OK;

    return APP_OK;
}

} // namespace my_engineer
