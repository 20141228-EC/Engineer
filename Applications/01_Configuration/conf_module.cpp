/**
 * @file conf_module.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 完成所有模块的配置
 * @version 1.0
 * @date 2026-03-06
 * 
 * @copyright Copyright (c) 2026
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
    armInitParam.MotorID_Pitch3 = EDeviceID::DEV_ARM_MTR_PITCH3;
    armInitParam.MotorID_Roll = EDeviceID::DEV_ARM_MTR_ROLL;
    armInitParam.MotorID_End_L = EDeviceID::DEV_ARM_MTR_END_L;
    armInitParam.MotorID_End_R = EDeviceID::DEV_ARM_MTR_END_R;
    armInitParam.MotorID_Grip = EDeviceID::DEV_ARM_MTR_GRIP;
    // 设置can发送节点
    armInitParam.MotorTxNode_Yaw = &TxNode_Can1_280;
    armInitParam.MotorTxNode_Pitch1 = &TxNode_Can1_280;
    armInitParam.MotorTxNode_Pitch2 = &TxNode_Can1_280;
    armInitParam.MotorTxNode_Pitch3 = &TxNode_Can1_280;
    armInitParam.MotorTxNode_End_L = &TxNode_Can2_1FF;  // 板2: CAN2->CAN1
    armInitParam.MotorTxNode_End_R = &TxNode_Can2_1FF;  // 板2: CAN2->CAN1
    armInitParam.MotorTxNode_Grip = &TxNode_Can2_1FF;   // 板2: CAN2->CAN1
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
   // 初始化 Pitch3PosPidParam 的成员
   armInitParam.Pitch3PosPidParam.kp = 0.f;
   armInitParam.Pitch3PosPidParam.ki = 0.0f;
   armInitParam.Pitch3PosPidParam.kd = 0.0f;
   armInitParam.Pitch3PosPidParam.maxIntegral = 3000.0f;
   armInitParam.Pitch3PosPidParam.maxOutput = 3000.0f;
   // 初始化 Pitch3SpdPidParam 的成员
   armInitParam.Pitch3SpdPidParam.kp = 0.f;
   armInitParam.Pitch3SpdPidParam.ki = 0.0f;
   armInitParam.Pitch3SpdPidParam.kd = 0.0f;
   armInitParam.Pitch3SpdPidParam.maxIntegral = 2000.0f;
   armInitParam.Pitch3SpdPidParam.maxOutput = 2000.0f;
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
   // 初始化 GripPosPidParam 的成员
   armInitParam.GripPosPidParam.kp = 0.2f;
    armInitParam.GripPosPidParam.ki = 0.0f;
    armInitParam.GripPosPidParam.kd = 0.0f;
    armInitParam.GripPosPidParam.maxOutput = 3000.0f;
    // 初始化 GripSpdPidParam 的成员
    armInitParam.GripSpdPidParam.kp = 23.f;
    armInitParam.GripSpdPidParam.ki = 0.0f;
    armInitParam.GripSpdPidParam.kd = 0.0f;
    armInitParam.GripSpdPidParam.maxIntegral = 1500.0f;
    armInitParam.GripSpdPidParam.maxOutput = 2000.0f;
    // 使用初始化后的参数创建 armModule 实例
    static auto armModule = CModArm(armInitParam);


    /******初始化子龙门模块******/ //-删除
/*
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
*/
    /******初始化云台模块******/
    CModGimbal::SModInitParam_Gimbal gimbalInitParam;
    gimbalInitParam.moduleID = EModuleID::MOD_GIMBAL;
    gimbalInitParam.yawMotorID = EDeviceID::DEV_GIMBAL_MTR_YAW;
    gimbalInitParam.storageMotorID_B = EDeviceID::DEV_GIMBAL_MTR_STORAGE_B;
    gimbalInitParam.storageMotorID_F = EDeviceID::DEV_GIMBAL_MTR_STORAGE_F;
    gimbalInitParam.pitchMotorID = EDeviceID::DEV_GIMBAL_MTR_PITCH;
    gimbalInitParam.MotorTxNode_Yaw = &TxNode_Can3_280;
    gimbalInitParam.storageMotorTxNode_F = &TxNode_Can1_1FF;
    gimbalInitParam.storageMotorTxNode_B = &TxNode_Can1_1FF;
    gimbalInitParam.pitchMotorTxNode = &TxNode_Can1_1FF;
    gimbalInitParam.FilterID = EAlgoID::ALGO_IMU_AVE;
    gimbalInitParam.memsDevID = EDeviceID::DEV_MEMS_BMI088;
    // 初始化pid参数
    gimbalInitParam.YawPosPidParam_Gyro.kp = 0.f;
    gimbalInitParam.YawPosPidParam_Gyro.ki = 0.0f;
    gimbalInitParam.YawPosPidParam_Gyro.kd = 0.f;
    gimbalInitParam.YawPosPidParam_Gyro.maxOutput = 4500.0f;

    gimbalInitParam.YawSpdPidParam_Gyro.kp = 0.f;
    gimbalInitParam.YawSpdPidParam_Gyro.ki = 0.f;
    gimbalInitParam.YawSpdPidParam_Gyro.kd = 0.0f;
    gimbalInitParam.YawSpdPidParam_Gyro.maxIntegral = 2000.0f;
    gimbalInitParam.YawSpdPidParam_Gyro.maxOutput = 3000.0f;

    gimbalInitParam.YawPosPidParam_Mec.kp = 0.f;
    gimbalInitParam.YawPosPidParam_Mec.ki = 0.0f;
    gimbalInitParam.YawPosPidParam_Mec.kd = 0.f;
    gimbalInitParam.YawPosPidParam_Mec.maxOutput = 4500.0f;

    gimbalInitParam.YawSpdPidParam_Mec.kp = 0.f;
    gimbalInitParam.YawSpdPidParam_Mec.ki = 0.f;
    gimbalInitParam.YawSpdPidParam_Mec.kd = 0.0f;
    gimbalInitParam.YawSpdPidParam_Mec.maxIntegral = 2000.0f;
    gimbalInitParam.YawSpdPidParam_Mec.maxOutput = 3000.0f;   ///< pid参数待调

    // 初始化存矿电机参数，双电机用同一套参数
    gimbalInitParam.storagePosPidParam.kp = 0.f;
    gimbalInitParam.storagePosPidParam.ki = 0.0f;
    gimbalInitParam.storagePosPidParam.kd = 0.f;
    gimbalInitParam.storagePosPidParam.maxOutput = 4500.0f;

    gimbalInitParam.storageSpdPidParam.kp = 0.0f;
    gimbalInitParam.storageSpdPidParam.ki = 0.f;
    gimbalInitParam.storageSpdPidParam.kd = 0.0f;
    gimbalInitParam.storageSpdPidParam.maxIntegral = 2000.0f;
    gimbalInitParam.storageSpdPidParam.maxOutput = 3000.0f;

    // 俯仰参数
    gimbalInitParam.pitchPosPidParam.kp = 0.f;
    gimbalInitParam.pitchPosPidParam.ki = 0.0f;
    gimbalInitParam.pitchPosPidParam.kd = 0.f;
    gimbalInitParam.pitchPosPidParam.maxOutput = 4500.0f;
    gimbalInitParam.pitchSpdPidParam.kp = 0.f;
    gimbalInitParam.pitchSpdPidParam.ki = 0.f;
    gimbalInitParam.pitchSpdPidParam.kd = 0.0f;
    gimbalInitParam.pitchSpdPidParam.maxIntegral = 2000.0f;
    gimbalInitParam.pitchSpdPidParam.maxOutput = 3000.0f;

    // 使用初始化后的参数创建 gimbalModule 实例
    static auto gimbalModule = CModGimbal(gimbalInitParam);

    return APP_OK;
}

} // namespace my_engineer
