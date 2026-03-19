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

// 控制器模块实例（文件作用域，调试器可直接查看，无需解引用指针）
CModController controllerModuleLeft;
CModController controllerModuleRight;

EAppStatus InitAllModule() {

    /*----------- 左臂控制器模块 -----------*/ 
    // CAN分配：CAN1=双臂DJI(Yaw), CAN2=左臂MIT(Pitch1/2+Roll+PitchEnd)
    CModController::SModInitParam_Controller leftInitParam;
    leftInitParam.moduleID = EModuleID::MOD_CONTROLLER_LEFT;
    leftInitParam.rocker_id = EDeviceID::DEV_ROCKER_LEFT;  // 左臂摇杆（单轴）
    leftInitParam.buzzer_id = EDeviceID::DEV_BUZZER;
    // 左臂摇杆X轴校准 (PA0/CH16, 实测: 8600 ~ 61200, 中心 34900)
    leftInitParam.rocker_x_center    = 34900;
    leftInitParam.rocker_x_range_pos = 26300;  // 61200 - 34900
    leftInitParam.rocker_x_range_neg = 26300;  // 34900 - 8600
    leftInitParam.rocker_x_dir       = -1;
    // 左臂电机设备ID
    leftInitParam.yaw_id = EDeviceID::DEV_MTR_YAW_L;
    leftInitParam.pitch1_id = EDeviceID::DEV_MTR_PITCH1_L;
    leftInitParam.pitch2_id = EDeviceID::DEV_MTR_PITCH2_L;
    leftInitParam.roll_id = EDeviceID::DEV_MTR_ROLL_L;
    leftInitParam.pitch_end_id = EDeviceID::DEV_MTR_PITCH_END_L;
    // 左臂CAN发送节点
    leftInitParam.yawTxNode = &TxNode_Can1_1FF;           // CAN1 DJI (ID5-8用0x1FF)
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
    // 初始化左臂控制器模块（对象已在文件作用域定义）
    controllerModuleLeft.InitModule(leftInitParam);

    // 右臂控制器模块
    // CAN分配：CAN1=双臂DJI(Yaw), CAN3=右臂MIT(Pitch1/2+Roll+PitchEnd)
    CModController::SModInitParam_Controller rightInitParam;
    rightInitParam.moduleID = EModuleID::MOD_CONTROLLER_RIGHT;
    rightInitParam.rocker_id = EDeviceID::DEV_ROCKER_RIGHT;  // 右臂摇杆（双轴）
    rightInitParam.buzzer_id = EDeviceID::DEV_BUZZER;
    // 右臂摇杆X轴校准 (PA2/CH14, 3.3V供电, 实测: 9255 ~ 61000, 中心 33450)
    rightInitParam.rocker_x_center    = 33450;
    rightInitParam.rocker_x_range_pos = 27550;  // 61000 - 33450
    rightInitParam.rocker_x_range_neg = 24195;  // 33450 - 9255
    rightInitParam.rocker_x_dir       = 1;      // PA2方向与PA0相反
    // 右臂摇杆Y轴校准 (PA5/CH19, 3.3V供电, 实测: 7400 ~ 58800, 中心 33450)
    rightInitParam.rocker_y_center    = 33450;
    rightInitParam.rocker_y_range_pos = 25350;  // 58800 - 33450
    rightInitParam.rocker_y_range_neg = 26050;  // 33450 - 7400
    rightInitParam.rocker_y_dir       = 1;
    // 右臂电机设备ID
    rightInitParam.yaw_id = EDeviceID::DEV_MTR_YAW_R;
    rightInitParam.pitch1_id = EDeviceID::DEV_MTR_PITCH1_R;
    rightInitParam.pitch2_id = EDeviceID::DEV_MTR_PITCH2_R;
    rightInitParam.roll_id = EDeviceID::DEV_MTR_ROLL_R;
    rightInitParam.pitch_end_id = EDeviceID::DEV_MTR_PITCH_END_R;
    // 右臂CAN发送节点
    rightInitParam.yawTxNode = &TxNode_Can1_1FF;          // CAN1 DJI (ID5-8用0x1FF，这是因为底层的库的编写没有考虑实际的编号和物理编号之间的区别）
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
    // 初始化右臂控制器模块（对象已在文件作用域定义）
    controllerModuleRight.InitModule(rightInitParam);

    return APP_OK;
}

} // namespace my_engineer
