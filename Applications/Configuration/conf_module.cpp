/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-05-14 01:05:00
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-15 20:19:03
 */

#include "conf_module.hpp"
#include "Module.hpp"

extern TIM_HandleTypeDef htim2;

namespace my_engineer {

EAppStatus InitAllModule() {

    // /******初始化自定义控制器模块******/
    CModController::SModInitParam_Controller controllerInitParam;
    controllerInitParam.moduleID = EModuleID::MOD_CONTROLLER;
    controllerInitParam.rocker_id = EDeviceID::DEV_ROCKER;
    controllerInitParam.buzzer_id = EDeviceID::DEV_BUZZER;
    controllerInitParam.yaw_id = EDeviceID::DEV_CONTROLLER_MTR_YAW;
    controllerInitParam.pitch1_id = EDeviceID::DEV_CONTROLLER_MTR_PITCH1;
    controllerInitParam.pitch2_id = EDeviceID::DEV_CONTROLLER_MTR_PITCH2;
    controllerInitParam.roll_id = EDeviceID::DEV_CONTROLLER_MTR_ROLL;
    // controllerInitParam.roll_end_id = EDeviceID::DEV_CONTROLLER_MTR_ROLL_END;
    controllerInitParam.pitch_end_id = EDeviceID::DEV_CONTROLLER_MTR_PITCH_END;
    // 设置can发送节点
    controllerInitParam.pitch1TxNode = &MitTxNode_Can2_30;
    controllerInitParam.pitch2TxNode = &MitTxNode_Can2_32;
    controllerInitParam.yawTxNode = &TxNode_Can1_1FF;
    controllerInitParam.rollTxNode = &TxNode_Can1_200;
    controllerInitParam.pitchEndTxNode = &TxNode_Can1_200;
    // 设置PID参数
    /*--------------------yaw---------------------------*/
    controllerInitParam.yawPosPidParam.kp = 3.0f;
    controllerInitParam.yawPosPidParam.ki = 0.0f;
    controllerInitParam.yawPosPidParam.kd = 0.0f;
    controllerInitParam.yawPosPidParam.maxIntegral = 20;
    controllerInitParam.yawPosPidParam.maxOutput = 5000;
    controllerInitParam.yawSpdPidParam.kp = 1.0f;
    controllerInitParam.yawSpdPidParam.ki = 0.1f;
    controllerInitParam.yawSpdPidParam.kd = 0.0f;
    controllerInitParam.yawSpdPidParam.maxIntegral = 200;
    controllerInitParam.yawSpdPidParam.maxOutput = 20000;
    /*--------------------roll----------------------------------*/
    controllerInitParam.rollPosPidParam.kp = 1.0f;
    controllerInitParam.rollPosPidParam.ki = 0.0f;
    controllerInitParam.rollPosPidParam.kd = 0.0f;
    controllerInitParam.rollPosPidParam.maxIntegral = 20;
    controllerInitParam.rollPosPidParam.maxOutput = 5000;
    controllerInitParam.rollSpdPidParam.kp = 0.5f;
    controllerInitParam.rollSpdPidParam.ki = 0.0f;
    controllerInitParam.rollSpdPidParam.kd = 0.0f;
    controllerInitParam.rollSpdPidParam.maxIntegral = 200;
    controllerInitParam.rollSpdPidParam.maxOutput = 8000;
    /*--------------------pitch end--------------------------------*/
    controllerInitParam.pitchEndPosPidParam.kp = 1.0f;
    controllerInitParam.pitchEndPosPidParam.ki = 0.0f;
    controllerInitParam.pitchEndPosPidParam.kd = 0.0f;
    controllerInitParam.pitchEndPosPidParam.maxIntegral = 20;
    controllerInitParam.pitchEndPosPidParam.maxOutput = 5000;
    controllerInitParam.pitchEndSpdPidParam.kp = 0.5f;
    controllerInitParam.pitchEndSpdPidParam.ki = 0.0f;
    controllerInitParam.pitchEndSpdPidParam.kd = 0.0f;
    controllerInitParam.pitchEndSpdPidParam.maxIntegral = 50;
    controllerInitParam.pitchEndSpdPidParam.maxOutput = 8000;
    // 使用初始化后的参数创建 controllerModule 实例
    static auto controllerModule = CModController(controllerInitParam);

    return APP_OK;
}

} // namespace my_engineer
