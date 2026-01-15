/**
 * @file conf_module.cpp
 * @author Sassinak, Ciallo
 * @brief 模块初始化配置
 * @version 1.1
 * @date 2025-05-14
 * @LastEditors Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-15
 *
 * @details
 */

#include "conf_module.hpp"
#include "Module.hpp"

extern TIM_HandleTypeDef htim2;

namespace my_engineer {

EAppStatus InitAllModule() {

    /*----------- 左臂控制器模块 -----------*/ 
    // CAN分配：CAN1=双臂DJI(Yaw), CAN2=左臂MIT(Pitch1/2+Roll+PitchEnd)
    CModController::SModInitParam_Controller leftInitParam;
    leftInitParam.moduleID = EModuleID::MOD_CONTROLLER_LEFT;
    leftInitParam.rocker_id = EDeviceID::DEV_ROCKER;
    leftInitParam.buzzer_id = EDeviceID::DEV_BUZZER;
    // 左臂电机设备ID
    leftInitParam.yaw_id = EDeviceID::DEV_MTR_YAW_L;
    leftInitParam.pitch1_id = EDeviceID::DEV_MTR_PITCH1_L;
    leftInitParam.pitch2_id = EDeviceID::DEV_MTR_PITCH2_L;
    leftInitParam.roll_id = EDeviceID::DEV_MTR_ROLL_L;
    leftInitParam.pitch_end_id = EDeviceID::DEV_MTR_PITCH_END_L;
    // 左臂CAN发送节点
    leftInitParam.yawTxNode = &TxNode_Can1_1FF;           // CAN1 DJI
    leftInitParam.pitch1TxNode = &MitTxNode_Can2_30;      // CAN2 MIT 0x30
    leftInitParam.pitch2TxNode = &MitTxNode_Can2_32;      // CAN2 MIT 0x32
    leftInitParam.rollTxNode = &MitTxNode_Can2_34;        // CAN2 MIT 0x34 (DM3510)
    leftInitParam.pitchEndTxNode = &MitTxNode_Can2_36;    // CAN2 MIT 0x36 (DM3510)
    // 左臂PID参数 - Yaw (M6020)
    leftInitParam.yawPosPidParam.kp = 3.0f;
    leftInitParam.yawPosPidParam.ki = 0.0f;
    leftInitParam.yawPosPidParam.kd = 0.0f;
    leftInitParam.yawPosPidParam.maxIntegral = 20;
    leftInitParam.yawPosPidParam.maxOutput = 5000;
    leftInitParam.yawSpdPidParam.kp = 1.0f;
    leftInitParam.yawSpdPidParam.ki = 0.1f;
    leftInitParam.yawSpdPidParam.kd = 0.0f;
    leftInitParam.yawSpdPidParam.maxIntegral = 200;
    leftInitParam.yawSpdPidParam.maxOutput = 20000;
    // 创建左臂控制器模块实例
    static auto controllerModuleLeft = CModController(leftInitParam);

    // 右臂控制器模块
    // CAN分配：CAN1=双臂DJI(Yaw), CAN3=右臂MIT(Pitch1/2+Roll+PitchEnd)
    CModController::SModInitParam_Controller rightInitParam;
    rightInitParam.moduleID = EModuleID::MOD_CONTROLLER_RIGHT;
    rightInitParam.rocker_id = EDeviceID::DEV_ROCKER;     // 共享摇杆
    rightInitParam.buzzer_id = EDeviceID::DEV_BUZZER;     // 共享蜂鸣器
    // 右臂电机设备ID
    rightInitParam.yaw_id = EDeviceID::DEV_MTR_YAW_R;
    rightInitParam.pitch1_id = EDeviceID::DEV_MTR_PITCH1_R;
    rightInitParam.pitch2_id = EDeviceID::DEV_MTR_PITCH2_R;
    rightInitParam.roll_id = EDeviceID::DEV_MTR_ROLL_R;
    rightInitParam.pitch_end_id = EDeviceID::DEV_MTR_PITCH_END_R;
    // 右臂CAN发送节点
    rightInitParam.yawTxNode = &TxNode_Can1_1FF;          // CAN1 DJI (共用)
    rightInitParam.pitch1TxNode = &MitTxNode_Can3_30;     // CAN3 MIT 0x30
    rightInitParam.pitch2TxNode = &MitTxNode_Can3_32;     // CAN3 MIT 0x32
    rightInitParam.rollTxNode = &MitTxNode_Can3_34;       // CAN3 MIT 0x34 (DM3510)
    rightInitParam.pitchEndTxNode = &MitTxNode_Can3_36;   // CAN3 MIT 0x36 (DM3510)
    // 右臂PID参数 - Yaw (M6020)
    rightInitParam.yawPosPidParam.kp = 3.0f;
    rightInitParam.yawPosPidParam.ki = 0.0f;
    rightInitParam.yawPosPidParam.kd = 0.0f;
    rightInitParam.yawPosPidParam.maxIntegral = 20;
    rightInitParam.yawPosPidParam.maxOutput = 5000;
    rightInitParam.yawSpdPidParam.kp = 1.0f;
    rightInitParam.yawSpdPidParam.ki = 0.1f;
    rightInitParam.yawSpdPidParam.kd = 0.0f;
    rightInitParam.yawSpdPidParam.maxIntegral = 200;
    rightInitParam.yawSpdPidParam.maxOutput = 20000;
    // 创建右臂控制器模块实例
    static auto controllerModuleRight = CModController(rightInitParam);

    return APP_OK;
}

} // namespace my_engineer
