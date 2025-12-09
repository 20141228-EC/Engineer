/**
 * @file mod_chassis.cpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 底盘模块
 * @version 1.0
 * @date 2024-11-05
 * 
 * @copyright Copyright (c) 2024
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

    // 初始化底盘轮组
    comWheelset_.InitComponent(param);
    comHip_.InitComponent(param);

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
 * @brief 更新处理
 * 
 * @return EAppStatus 
 */
void CModChassis::UpdateHandler_(){

    // 检查模块状态
    if (moduleStatus == APP_RESET) return;

    static uint8_t HalfTickRate = 0;
	HalfTickRate = 1 - HalfTickRate;

    comWheelset_.MovMode_ = MovMode; ///< 更新面向底层轮组的运动模式

    DataBuffer<float_t> pitch_Target = {0.0f}; ///< 目标pitch角度，目前暂时写这个，后续出车之后根据实际可能有些误差待改

    // 计算Pitch角
    float_t acc_x = comHip_.mems->memsData[CMemsBase::DATA_ACC_X];
    float_t acc_y = comHip_.mems->memsData[CMemsBase::DATA_ACC_Y];
    float_t acc_z = comHip_.mems->memsData[CMemsBase::DATA_ACC_Z];
    pitch_Measure = {atan2f(acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z))};

    // 底盘pitch轴是一个三环pid控制，最外环为控pitch轴角度，输出目标腿长，内环是控腿长
    DataBuffer<float_t> pitch_target_climbing;
    if(MovMode == EmovMode::CLIMBING)
    {
        pitch_target_climbing = comHip_.pidPitchCtrl.UpdatePidController(pitch_Target, pitch_Measure);
        chassisCmd.L_length += pitch_target_climbing[0] * PITCH_DEG_ECD_RATIO; ///< 在当前腿长目标基础上进行累加
    } 

    // 更新底盘轮组
    comWheelset_.UpdateComponent();
    if(HalfTickRate){comHip_.UpdateComponent();} ///< 降为500Hz


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
    chassisCmd.L_length = std::clamp(chassisCmd.L_length, -100.0f, 100.0f);

    //看后续是否要加对髋关节的控制命令大小的限制

    // 自动控制启用，则不继续做限制
    if (chassisCmd.isAutoCtrl) return APP_OK;

    return APP_OK;
}

} // namespace my_engineer
