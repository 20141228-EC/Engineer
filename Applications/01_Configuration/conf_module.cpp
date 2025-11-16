/**
 * @file conf_module.cpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 完成所有模块的配置
 * @version 1.0
 * @date 2024-11-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "conf_module.hpp"
#include "Module.hpp"

namespace my_engineer {

EAppStatus InitAllModule() {

    /******初始化机械臂模块******/
    CModArm::SModInitParam_Arm armInitParam;
    armInitParam.moduleID = EModuleID::MOD_ARM;
    armInitParam.MotorID_Yaw = EDeviceID::DEV_ARM_MTR_YAW;
    armInitParam.MotorID_Pitch1 = EDeviceID::DEV_ARM_MTR_PITCH1;
    armInitParam.MotorID_Pitch2 = EDeviceID::DEV_ARM_MTR_PITCH2;
    armInitParam.MotorID_Roll = EDeviceID::DEV_ARM_MTR_ROLL;
    armInitParam.MotorID_End_L = EDeviceID::DEV_ARM_MTR_END_L;
    armInitParam.MotorID_End_R = EDeviceID::DEV_ARM_MTR_END_R;
    // 设置can发送节点
    armInitParam.MotorTxNode_Yaw = &TxNode_Can3_280;
    armInitParam.MotorTxNode_Pitch1 = &TxNode_Can3_280;
    armInitParam.MotorTxNode_Pitch2 = &TxNode_Can3_280;
    armInitParam.MotorTxNode_End_L = &TxNode_Can2_1FF;
    armInitParam.MotorTxNode_End_R = &TxNode_Can2_1FF;
    // 初始化 YawPosPidParam 的成员
   armInitParam.YawPosPidParam.kp = 0.4;
   armInitParam.YawPosPidParam.ki = 0.0f;
   armInitParam.YawPosPidParam.kd = 0.0f;
   armInitParam.YawPosPidParam.maxIntegral = 3000.0f;
   armInitParam.YawPosPidParam.maxOutput = 3000.0f;
   // 初始化 YawSpdPidParam 的成员
   armInitParam.YawSpdPidParam.kp = 0.1f;
   armInitParam.YawSpdPidParam.ki = 0.0f;
   armInitParam.YawSpdPidParam.kd = 0.0f;
   armInitParam.YawSpdPidParam.maxIntegral = 1000.0f;
   armInitParam.YawSpdPidParam.maxOutput = 2000.0f;
    // 初始化 Pitch1PosPidParam 的成员
   armInitParam.Pitch1PosPidParam.kp = 2.8f;
   armInitParam.Pitch1PosPidParam.ki = 0.0f;
   armInitParam.Pitch1PosPidParam.kd = 0.0f;
   armInitParam.Pitch1PosPidParam.maxIntegral = 3000.0f;
   armInitParam.Pitch1PosPidParam.maxOutput = 3000.0f;
   // 初始化 Pitch1SpdPidParam 的成员
   armInitParam.Pitch1SpdPidParam.kp = 0.1f;
   armInitParam.Pitch1SpdPidParam.ki = 0.05f;
   armInitParam.Pitch1SpdPidParam.kd = 0.0f;
   armInitParam.Pitch1SpdPidParam.maxIntegral = 2000.0f;
   armInitParam.Pitch1SpdPidParam.maxOutput = 2000.0f;
   // 初始化 Pitch2PosPidParam 的成员
   armInitParam.Pitch2PosPidParam.kp = 2.3f;
   armInitParam.Pitch2PosPidParam.ki = 0.0f;
   armInitParam.Pitch2PosPidParam.kd = 0.0f;
   armInitParam.Pitch2PosPidParam.maxIntegral = 3000.0f;
   armInitParam.Pitch2PosPidParam.maxOutput = 3000.0f;
   // 初始化 Pitch2SpdPidParam 的成员
   armInitParam.Pitch2SpdPidParam.kp = 0.1f;
   armInitParam.Pitch2SpdPidParam.ki = 0.05f;
   armInitParam.Pitch2SpdPidParam.kd = 0.0f;
   armInitParam.Pitch2SpdPidParam.maxIntegral = 2000.0f;
   armInitParam.Pitch2SpdPidParam.maxOutput = 2000.0f;
   // 初始化 mitCtrl_Roll 的成员
   armInitParam.MIT_Roll_kp = 20.0f;
   armInitParam.MIT_Roll_kd = 1.0f;
   // 初始化 endPosPidParam 的成员
   armInitParam.endPosPidParam.kp = 0.18f;
   armInitParam.endPosPidParam.ki = 0.0f;
   armInitParam.endPosPidParam.kd = 0.0f;
   armInitParam.endPosPidParam.maxOutput = 4000.0f;
   // 初始化 endSpdPidParam 的成员
   armInitParam.endSpdPidParam.kp = 4.0f;
   armInitParam.endSpdPidParam.ki = 0.8f;
   armInitParam.endSpdPidParam.kd = 0.0f;
   armInitParam.endSpdPidParam.maxIntegral = 4000.0f;
   armInitParam.endSpdPidParam.maxOutput = 4500.0f;
    // 使用初始化后的参数创建 armModule 实例
    static auto armModule = CModArm(armInitParam);


    /******初始化子龙门模块******/
    CModSubGantry::SModInitParam_SubGantry subGantryInitParam;
    subGantryInitParam.moduleID = EModuleID::MOD_SUBGANTRY;
    subGantryInitParam.liftMotorID_L = EDeviceID::DEV_SUBGANTRY_MTR_LIFT_L;
    subGantryInitParam.liftMotorID_R = EDeviceID::DEV_SUBGANTRY_MTR_LIFT_R;
    subGantryInitParam.stretchMotorID_L = EDeviceID::DEV_SUBGANTRY_MTR_STRETCH_L;
    subGantryInitParam.stretchMotorID_R = EDeviceID::DEV_SUBGANTRY_MTR_STRETCH_R;
    subGantryInitParam.liftMotorTxNode_L = &TxNode_Can2_200;
    subGantryInitParam.liftMotorTxNode_R = &TxNode_Can2_200;
    subGantryInitParam.stretchMotorTxNode_L = &TxNode_Can2_200;
    subGantryInitParam.stretchMotorTxNode_R = &TxNode_Can2_200;
    subGantryInitParam.LeftPumpPort = LeftPump_GPIO_Port;
    subGantryInitParam.LeftPumpPin = LeftPump_Pin;
    subGantryInitParam.RightPumpPort = RightPump_GPIO_Port;
    subGantryInitParam.RightPumpPin = RightPump_Pin;
    subGantryInitParam.ArmPumpPort = ArmPump_GPIO_Port;
    subGantryInitParam.ArmPumpPin = ArmPump_Pin;
    // 设置PID参数
    subGantryInitParam.liftPosPidParam_L.kp = 0.3f;
    subGantryInitParam.liftPosPidParam_L.ki = 0.0f;
    subGantryInitParam.liftPosPidParam_L.kd = 0.1f;
    subGantryInitParam.liftPosPidParam_L.maxIntegral = 0.0f;
    subGantryInitParam.liftPosPidParam_L.maxOutput = 5000.0f;
    subGantryInitParam.liftSpdPidParam_L.kp = 1.5f;
    subGantryInitParam.liftSpdPidParam_L.ki = 0.5f;
    subGantryInitParam.liftSpdPidParam_L.kd = 0.0f;
    subGantryInitParam.liftSpdPidParam_L.maxIntegral = 4500.0f;
    subGantryInitParam.liftSpdPidParam_L.maxOutput = 8000.f;//8000.0f;
    subGantryInitParam.liftPosPidParam_R.kp = 0.3f;
    subGantryInitParam.liftPosPidParam_R.ki = 0.0f;
    subGantryInitParam.liftPosPidParam_R.kd = 0.1f;
    subGantryInitParam.liftPosPidParam_R.maxIntegral = 0.0f;
    subGantryInitParam.liftPosPidParam_R.maxOutput = 5000.0f;
    subGantryInitParam.liftSpdPidParam_R.kp = 1.0f;
    subGantryInitParam.liftSpdPidParam_R.ki = 0.5f;
    subGantryInitParam.liftSpdPidParam_R.kd = 0.0f;
    subGantryInitParam.liftSpdPidParam_R.maxIntegral = 4500.0f;
    subGantryInitParam.liftSpdPidParam_R.maxOutput = 7000.f;//8000.0f;
    subGantryInitParam.stretchPosPidParam.kp = 0.18f;
    subGantryInitParam.stretchPosPidParam.ki = 0.0f;
    subGantryInitParam.stretchPosPidParam.kd = 0.1f;
    subGantryInitParam.stretchPosPidParam.maxIntegral = 0.0f;
    subGantryInitParam.stretchPosPidParam.maxOutput = 5000.0f;
    subGantryInitParam.stretchSpdPidParam.kp = 3.0f;
    subGantryInitParam.stretchSpdPidParam.ki = 0.3f;
    subGantryInitParam.stretchSpdPidParam.kd = 0.0f;
    subGantryInitParam.stretchSpdPidParam.maxIntegral = 4500.0f;
    subGantryInitParam.stretchSpdPidParam.maxOutput = 8000.0f;
    // 使用初始化后的参数创建 subGantryModule 实例
    static auto subGantryModule = CModSubGantry(subGantryInitParam);

    /******初始化云台模块******/
    CModGimbal::SModInitParam_Gimbal gimbalInitParam;
    gimbalInitParam.moduleID = EModuleID::MOD_GIMBAL;
    gimbalInitParam.liftMotorID = EDeviceID::DEV_GIMBAL_MTR;
    // 设置can发送节点
    gimbalInitParam.liftMotorTxNode = &TxNode_Can2_1FF;
    // 初始化 liftPosPidParam 的成员
   gimbalInitParam.liftPosPidParam.kp = 0.3f;
   gimbalInitParam.liftPosPidParam.ki = 0.0f;
   gimbalInitParam.liftPosPidParam.kd = 0.3f;
   gimbalInitParam.liftPosPidParam.maxOutput = 4500.0f;
   // 初始化 liftSpdPidParam 的成员
   gimbalInitParam.liftSpdPidParam.kp = 2.0f;
   gimbalInitParam.liftSpdPidParam.ki = 0.1f;
   gimbalInitParam.liftSpdPidParam.kd = 0.0f;
   gimbalInitParam.liftSpdPidParam.maxIntegral = 2000.0f;
   gimbalInitParam.liftSpdPidParam.maxOutput = 7000.0f;
   //  使用初始化后的参数创建 gimbalModule 实例
    static auto gimbalModule = CModGimbal(gimbalInitParam);

    /******初始化底盘模块******/
    CModChassis::SModInitParam_Chassis chassisInitParam;
    chassisInitParam.moduleID = EModuleID::MOD_CHASSIS;
    chassisInitParam.memsDevID = EDeviceID::DEV_MEMS_BMI088;
    chassisInitParam.wheelsetMotorID_LF = EDeviceID::DEV_CHAS_MTR_LF;
    chassisInitParam.wheelsetMotorID_RF = EDeviceID::DEV_CHAS_MTR_RF;
    chassisInitParam.wheelsetMotorID_LB = EDeviceID::DEV_CHAS_MTR_LB;
    chassisInitParam.wheelsetMotorID_RB = EDeviceID::DEV_CHAS_MTR_RB;
    // 设置can发送节点
    chassisInitParam.wheelsetMotorTxNode_LF = &TxNode_Can1_200;
    chassisInitParam.wheelsetMotorTxNode_RF = &TxNode_Can1_200;
    chassisInitParam.wheelsetMotorTxNode_LB = &TxNode_Can1_200;
    chassisInitParam.wheelsetMotorTxNode_RB = &TxNode_Can1_200;
    // 设置PID参数
    chassisInitParam.yawCorrectionPidParam.kp = 10.0f;
    chassisInitParam.yawCorrectionPidParam.ki = 20.0f;
    chassisInitParam.yawCorrectionPidParam.kd = 0.0f;
    chassisInitParam.yawCorrectionPidParam.Input_deadband = 1.0f;
    chassisInitParam.yawCorrectionPidParam.maxIntegral = 50.0f;
    chassisInitParam.yawCorrectionPidParam.maxOutput = 5000.0f;
    chassisInitParam.lineCorrectionPidParam.kp = 0.0f;
    chassisInitParam.lineCorrectionPidParam.ki = 0.0f;
    chassisInitParam.lineCorrectionPidParam.kd = 0.0f;
    chassisInitParam.lineCorrectionPidParam.maxIntegral = 0.0f;
    chassisInitParam.lineCorrectionPidParam.maxOutput = 0.0f;
    chassisInitParam.wheelsetSpdPidParam.kp = 8.0f;
    chassisInitParam.wheelsetSpdPidParam.ki = 1.0f;
    chassisInitParam.wheelsetSpdPidParam.kd = 0.0f;
    chassisInitParam.wheelsetSpdPidParam.Input_deadband = 1.0f;
    chassisInitParam.wheelsetSpdPidParam.maxIntegral = 4000.0f;
    chassisInitParam.wheelsetSpdPidParam.maxOutput = 15000.0f;
    // 使用初始化后的参数创建 chassisModule 实例
    static auto chassisModule = CModChassis(chassisInitParam);




    return APP_OK;
}

} // namespace my_engineer
