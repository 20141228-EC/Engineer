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

// 控制器模块实例
CModController controllerModule;

EAppStatus InitAllModule() {

    /*----------- 单臂控制器模块 -----------*/
    // CAN分配：CAN1=DJI(Yaw), CAN2=MIT(Roll+PitchEnd), CAN3=MIT(Pitch1/2)
    CModController::SModInitParam_Controller initParam;
    initParam.moduleID = EModuleID::MOD_CONTROLLER;
    initParam.rocker_id = EDeviceID::DEV_ROCKER;  // 摇杆（双轴）
    initParam.buzzer_id = EDeviceID::DEV_BUZZER;
    // 摇杆X轴校准 (PA2/CH14, 3.3V供电, 实测: 9255 ~ 61000, 中心 33450)
    initParam.rocker_x_center    = 33450;
    initParam.rocker_x_range_pos = 27550;  // 61000 - 33450
    initParam.rocker_x_range_neg = 24195;  // 33450 - 9255
    initParam.rocker_x_dir       = 1;
    // 摇杆Y轴校准 (PA5/CH19, 3.3V供电, 实测: 7400 ~ 58800, 中心 33450)
    initParam.rocker_y_center    = 33450;
    initParam.rocker_y_range_pos = 25350;  // 58800 - 33450
    initParam.rocker_y_range_neg = 26050;  // 33450 - 7400
    initParam.rocker_y_dir       = 1;
    // 电机设备ID
    initParam.yaw_id = EDeviceID::DEV_MTR_YAW;
    initParam.pitch1_id = EDeviceID::DEV_MTR_PITCH1;
    initParam.pitch2_id = EDeviceID::DEV_MTR_PITCH2;
    initParam.roll_id = EDeviceID::DEV_MTR_ROLL;
    initParam.pitch_end_id = EDeviceID::DEV_MTR_PITCH_END;
    // CAN发送节点（Pitch1/2->CAN3, Roll/PitchEnd->CAN2, Yaw->CAN1）
    initParam.yawTxNode = &TxNode_Can1_1FF;           // CAN1 DJI (ID1-4用0x1FF)
    initParam.pitch1TxNode = &MitTxNode_Can3_30;      // CAN3 MIT 0x30
    initParam.pitch2TxNode = &MitTxNode_Can3_32;      // CAN3 MIT 0x32
    initParam.rollTxNode = &MitTxNode_Can2_34;         // CAN2 MIT 0x34
    initParam.pitchEndTxNode = &MitTxNode_Can2_36;     // CAN2 MIT 0x36 
    // PID参数 - Yaw (M6020)
    initParam.yawPosPidParam.kp = 3.0f;
    initParam.yawPosPidParam.ki = 0.0f;
    initParam.yawPosPidParam.kd = 0.0f;
    initParam.yawPosPidParam.maxIntegral = 20;
    initParam.yawPosPidParam.maxOutput = 5000;
    initParam.yawSpdPidParam.kp = 1.0f;
    initParam.yawSpdPidParam.ki = 0.1f;
    initParam.yawSpdPidParam.kd = 0.0f;
    initParam.yawSpdPidParam.maxIntegral = 200;
    initParam.yawSpdPidParam.maxOutput = 20000;

    // initParam.pitch3PosPidParam.kp = 3.0f;
    // initParam.pitch3PosPidParam.ki = 0.0f;
    // initParam.pitch3PosPidParam.kd = 0.0f;
    // initParam.pitch3PosPidParam.maxIntegral = 20;
    // initParam.pitch3PosPidParam.maxOutput = 5000;
    // initParam.pitch3SpdPidParam.kp = 1.0f;
    // initParam.pitch3SpdPidParam.ki = 0.1f;
    // initParam.pitch3SpdPidParam.kd = 0.0f;
    // initParam.pitch3SpdPidParam.maxIntegral = 200;
    // initParam.pitch3SpdPidParam.maxOutput = 20000;
    // 初始化控制器模块
    controllerModule.InitModule(initParam);

    return APP_OK;
}

} // namespace my_engineer
