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
    // CAN发送节点
    initParam.yawTxNode = &TxNode_Can1_1FF;           // CAN1 DJI (ID1-4用0x1FF)
    initParam.pitch1TxNode = &MitTxNode_Can2_30;      // CAN3 MIT 0x30
    initParam.pitch2TxNode = &MitTxNode_Can2_32;      // CAN3 MIT 0x32
    initParam.rollTxNode = &MitTxNode_Can3_34;         // CAN2 MIT 0x34
    initParam.pitchEndTxNode = &MitTxNode_Can3_36;     // CAN2 MIT 0x36 
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

    // 重力补偿参数
    auto &gp = initParam.gravParam;
    gp.ramp_alpha = 0.05f;

    // 零点偏移
    gp.pitch1_zero_deg = 0.0f;
    gp.pitch2_zero_deg = 0.0f;
    gp.roll_zero_deg = 0.0f;
    gp.pitch_end_zero_deg = 0.0f;

    // 电机转换
    gp.pitch1_motor.type = EMotorType::DM_MIT;
    gp.pitch1_motor.reduction = CONTROLLER_GEAR_RATIO_DM4310;  // 10.0
    gp.pitch1_motor.direction = static_cast<float>(CONTROLLER_PITCH1_MOTOR_DIR);  // -1
    gp.pitch1_ff_limit = 2.5f;        // DM4310 电机侧力矩限幅 (N·m)
    gp.pitch2_motor.type = EMotorType::DM_MIT;
    gp.pitch2_motor.reduction = CONTROLLER_GEAR_RATIO_DM4310;
    gp.pitch2_motor.direction = static_cast<float>(CONTROLLER_PITCH2_MOTOR_DIR);  // -1
    gp.pitch2_ff_limit = 2.5f;
    gp.roll_motor.type = EMotorType::DM_MIT;
    gp.roll_motor.reduction = 1.0f;   // DM3510 直驱
    gp.roll_motor.direction = static_cast<float>(CONTROLLER_ROLL_MOTOR_DIR);  // -1
    gp.roll_ff_limit = 0.5f;          // DM3510 力矩限幅
    gp.pitch_end_motor.type = EMotorType::DM_MIT;
    gp.pitch_end_motor.reduction = 1.0f;  // DM3510 直驱
    gp.pitch_end_motor.direction = static_cast<float>(CONTROLLER_PITCH_END_MOTOR_DIR);  // -1
    gp.pitch_end_ff_limit = 0.5f;     // DM3510 力矩限幅

    // 工作空间 (deg), 超出强制 ramp 下降
    gp.ws_pitch1_min = -180.0f;  gp.ws_pitch1_max = 180.0f;
    gp.ws_pitch2_min = -180.0f;  gp.ws_pitch2_max = 180.0f;
    gp.ws_roll_min = -180.0f;    gp.ws_roll_max = 180.0f;

    // 初始化控制器模块
    controllerModule.InitModule(initParam);

    return APP_OK;
}

} // namespace my_engineer
