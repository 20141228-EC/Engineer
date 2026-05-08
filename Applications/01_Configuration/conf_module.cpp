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
    armInitParam.MotorTxNode_End_L = &TxNode_Can2_1FF;
    armInitParam.MotorTxNode_End_R = &TxNode_Can2_1FF;
    armInitParam.MotorTxNode_Grip = &TxNode_Can2_1FF;
    // 初始化 YawPosPidParam 的成员
   armInitParam.YawPosPidParam.kp = 2.5f;
   armInitParam.YawPosPidParam.ki = 0.05f;
   armInitParam.YawPosPidParam.kd = 0.0f;
   armInitParam.YawPosPidParam.maxIntegral = 3000.0f;
   armInitParam.YawPosPidParam.maxOutput = 5000.0f;
   // 初始化 YawSpdPidParam 的成员
   armInitParam.YawSpdPidParam.kp = 0.062f;
   armInitParam.YawSpdPidParam.ki = 0.05f;
   armInitParam.YawSpdPidParam.kd = 0.0f;
   armInitParam.YawSpdPidParam.maxIntegral = 3000.0f;
   armInitParam.YawSpdPidParam.maxOutput = 3000.0f;
    // 初始化 Pitch1PosPidParam 的成员
   armInitParam.Pitch1PosPidParam.kp = 3.f;
   armInitParam.Pitch1PosPidParam.ki = 0.0f;
   armInitParam.Pitch1PosPidParam.kd = 0.0f;
   armInitParam.Pitch1PosPidParam.maxIntegral = 3000.0f;
   armInitParam.Pitch1PosPidParam.maxOutput = 4000.0f;
   // 初始化 Pitch1SpdPidParam 的成员
   armInitParam.Pitch1SpdPidParam.kp = 0.06f;
   armInitParam.Pitch1SpdPidParam.ki = 0.05f;
   armInitParam.Pitch1SpdPidParam.kd = 0.0f;
   armInitParam.Pitch1SpdPidParam.maxIntegral = 2000.0f;
   armInitParam.Pitch1SpdPidParam.maxOutput = 2000.0f;
   // 初始化 Pitch2PosPidParam 的成员
   armInitParam.Pitch2PosPidParam.kp = 3.f;
   armInitParam.Pitch2PosPidParam.ki = 0.01f;
   armInitParam.Pitch2PosPidParam.kd = 0.0f;
   armInitParam.Pitch2PosPidParam.maxIntegral = 4000.0f;
   armInitParam.Pitch2PosPidParam.maxOutput = 5000.0f;
   // 初始化 Pitch2SpdPidParam 的成员
   armInitParam.Pitch2SpdPidParam.kp = 0.07f;
   armInitParam.Pitch2SpdPidParam.ki = 0.01f;
   armInitParam.Pitch2SpdPidParam.kd = 0.0f;
   armInitParam.Pitch2SpdPidParam.maxIntegral = 2000.0f;
   armInitParam.Pitch2SpdPidParam.maxOutput = 2000.0f;
   // 初始化 Pitch3PosPidParam 的成员
   armInitParam.Pitch3PosPidParam.kp = 4.f;
   armInitParam.Pitch3PosPidParam.ki = 0.05f;
   armInitParam.Pitch3PosPidParam.kd = 0.0f;
   armInitParam.Pitch3PosPidParam.maxIntegral = 4000.0f;
   armInitParam.Pitch3PosPidParam.maxOutput = 5500.0f;
   // 初始化 Pitch3SpdPidParam 的成员
   armInitParam.Pitch3SpdPidParam.kp = 0.07f;
   armInitParam.Pitch3SpdPidParam.ki = 0.05f;
   armInitParam.Pitch3SpdPidParam.kd = 0.0f;
   armInitParam.Pitch3SpdPidParam.maxIntegral = 2000.0f;
   armInitParam.Pitch3SpdPidParam.maxOutput = 2000.0f;
   // 初始化 mitCtrl_Roll 的成员
   armInitParam.MIT_Roll_kp = 15.0f;
   armInitParam.MIT_Roll_kd = 1.0f;
//    armInitParam.Pitch1PosPidParam.Grav_Load_Mode = CAlgoPid::EGravLoadMode::END_ROLL_G; ///< 末端roll的重补模式
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
   armInitParam.GripPosPidParam.kp = 0.12f;
    armInitParam.GripPosPidParam.ki = 0.15f;
    armInitParam.GripPosPidParam.kd = 0.0f;
    armInitParam.GripPosPidParam.maxOutput = 3000.0f;
    // 初始化 GripSpdPidParam 的成员
    armInitParam.GripSpdPidParam.kp = 3.15f;
    armInitParam.GripSpdPidParam.ki = 0.4f;
    armInitParam.GripSpdPidParam.kd = 0.0f;
    armInitParam.GripSpdPidParam.maxIntegral = 3000.0f;
    armInitParam.GripSpdPidParam.maxOutput = 4000.0f;
    // armInitParam.Need_Grav_Compensation = false;
    // 使用初始化后的参数创建 armModule 实例 
    static auto armModule = CModArm(armInitParam);

   /******初始化云台模块******/
    CModGimbal::SModInitParam_Gimbal gimbalInitParam;
    gimbalInitParam.yawMotorID = EDeviceID::DEV_GIMBAL_MTR_YAW;
    gimbalInitParam.moduleID = EModuleID::MOD_GIMBAL;
    gimbalInitParam.FilterID = EAlgoID::ALGO_IMU_AVE;
    gimbalInitParam.memsDevID = EDeviceID::DEV_MEMS_BMI088;
    gimbalInitParam.MotorTxNode_Yaw = &TxNode_Can3_280;

    gimbalInitParam.YawPosPidParam_Gyro.kp = 9.f;
    gimbalInitParam.YawPosPidParam_Gyro.ki = 0.f;
    gimbalInitParam.YawPosPidParam_Gyro.kd = 0.f;
    gimbalInitParam.YawPosPidParam_Gyro.maxIntegral = 4000.f;
    gimbalInitParam.YawPosPidParam_Gyro.maxOutput = 5000.f;

    gimbalInitParam.YawSpdPidParam_Gyro.kp = 3.6f;
    gimbalInitParam.YawSpdPidParam_Gyro.ki = 0.01f;
    gimbalInitParam.YawSpdPidParam_Gyro.kd = 0.0f;
    gimbalInitParam.YawSpdPidParam_Gyro.maxIntegral = 2000.f;
    gimbalInitParam.YawSpdPidParam_Gyro.maxOutput = 5000.f;

    gimbalInitParam.YawPosPidParam_Mec.kp = 2.5f;
    gimbalInitParam.YawPosPidParam_Mec.ki = 0.f;
    gimbalInitParam.YawPosPidParam_Mec.kd = 0.f;
    gimbalInitParam.YawPosPidParam_Mec.maxIntegral = 4000.f;
    gimbalInitParam.YawPosPidParam_Mec.maxOutput = 5000.f;

    gimbalInitParam.YawSpdPidParam_Mec.kp = 2.f;
    gimbalInitParam.YawSpdPidParam_Mec.ki = 0.0f;
    gimbalInitParam.YawSpdPidParam_Mec.kd = 0.f;
    gimbalInitParam.YawSpdPidParam_Mec.maxIntegral = 1000.f;
    gimbalInitParam.YawSpdPidParam_Mec.maxOutput = 5000.f;

    // 使用初始化后的参数创建 gimbalModule 实例
    static auto gimbalModule = CModGimbal(gimbalInitParam);
    return APP_OK;
}

} // namespace my_engineer
