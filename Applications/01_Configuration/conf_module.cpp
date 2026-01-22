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
    armInitParam.MotorID_Grip = EDeviceID::DEV_ARM_MTR_GRIP;
    // 设置can发送节点
    armInitParam.MotorTxNode_Yaw = &TxNode_Can3_280;
    armInitParam.MotorTxNode_Pitch1 = &TxNode_Can3_280;
    armInitParam.MotorTxNode_Pitch2 = &TxNode_Can3_280;
    armInitParam.MotorTxNode_End_L = &TxNode_Can2_1FF;
    armInitParam.MotorTxNode_End_R = &TxNode_Can2_1FF;
    armInitParam.MotorTxNode_Grip = &TxNode_Can2_1FF;
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
//    armInitParam.Pitch1PosPidParam.Need_Grav_compensation = true; ///< 使用重力补偿
//    armInitParam.Pitch1PosPidParam.Grav_Load_Mode = CAlgoPid::EGravLoadMode::PITCH1_G; ///< 大pitch的重补模式
   // 初始化 Pitch1SpdPidParam 的成员
   armInitParam.Pitch1SpdPidParam.kp = 0.1f;
   armInitParam.Pitch1SpdPidParam.ki = 0.05f;
   armInitParam.Pitch1SpdPidParam.kd = 0.0f;
   armInitParam.Pitch1SpdPidParam.maxIntegral = 2000.0f;
   armInitParam.Pitch1SpdPidParam.maxOutput = 2000.0f;
//    armInitParam.Pitch1SpdPidParam.Need_Grav_compensation = true; ///<使用重力补偿
//    armInitParam.Pitch1PosPidParam.Grav_Load_Mode = CAlgoPid::EGravLoadMode::PITCH1_G;///< 大pitch的重补模式
   // 初始化 Pitch2PosPidParam 的成员
   armInitParam.Pitch2PosPidParam.kp = 2.3f;
   armInitParam.Pitch2PosPidParam.ki = 0.0f;
   armInitParam.Pitch2PosPidParam.kd = 0.0f;
   armInitParam.Pitch2PosPidParam.maxIntegral = 3000.0f;
   armInitParam.Pitch2PosPidParam.maxOutput = 3000.0f;
//    armInitParam.Pitch2PosPidParam.Need_Grav_compensation = true; ///< 使用重力补偿
//    armInitParam.Pitch1PosPidParam.Grav_Load_Mode = CAlgoPid::EGravLoadMode::PITCH2_G;///< 小pitch的重补模式
   // 初始化 Pitch2SpdPidParam 的成员
   armInitParam.Pitch2SpdPidParam.kp = 0.1f;
   armInitParam.Pitch2SpdPidParam.ki = 0.05f;
   armInitParam.Pitch2SpdPidParam.kd = 0.0f;
   armInitParam.Pitch2SpdPidParam.maxIntegral = 2000.0f;
   armInitParam.Pitch2SpdPidParam.maxOutput = 2000.0f;
//    armInitParam.Pitch2SpdPidParam.Need_Grav_compensation = true; ///< 使用重力补偿
//    armInitParam.Pitch1PosPidParam.Grav_Load_Mode = CAlgoPid::EGravLoadMode::PITCH2_G;   ///< 小pitch的重补模式
   // 初始化 mitCtrl_Roll 的成员
   armInitParam.MIT_Roll_kp = 20.0f;
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
   armInitParam.GripPosPidParam.kp = 0.35f;
    armInitParam.GripPosPidParam.ki = 0.0f;
    armInitParam.GripPosPidParam.kd = 0.0f;
    armInitParam.GripPosPidParam.maxOutput = 3000.0f;
    // 初始化 GripSpdPidParam 的成员
    armInitParam.GripSpdPidParam.kp = 23.f;
    armInitParam.GripSpdPidParam.ki = 0.0f;
    armInitParam.GripSpdPidParam.kd = 0.0f;
    armInitParam.GripSpdPidParam.maxIntegral = 1500.0f;
    armInitParam.GripSpdPidParam.maxOutput = 4000.0f;
    // armInitParam.Need_Grav_Compensation = false;
    // 使用初始化后的参数创建 armModule 实例 
    static auto armModule = CModArm(armInitParam);

    /******初始化底盘模块******/
    CModChassis::SModInitParam_Chassis chassisInitParam;
    chassisInitParam.moduleID = EModuleID::MOD_CHASSIS;
    chassisInitParam.memsDevID = EDeviceID::DEV_MEMS_BMI088;
    chassisInitParam.FilterID = EAlgoID::ALGO_IMU_AVE;      //使用互补滤波
    chassisInitParam.wheelsetMotorID_LF = EDeviceID::DEV_CHAS_MTR_LF;
    chassisInitParam.wheelsetMotorID_RF = EDeviceID::DEV_CHAS_MTR_RF;
    chassisInitParam.wheelsetMotorID_LB = EDeviceID::DEV_CHAS_MTR_LB;
    chassisInitParam.wheelsetMotorID_RB = EDeviceID::DEV_CHAS_MTR_RB;
    chassisInitParam.hipMotorID_L_L = EDeviceID::DEV_CHAS_L_HIP;
    chassisInitParam.hipMotorID_L_R = EDeviceID::DEV_CHAS_R_HIP;
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
    chassisInitParam.rollCorrectionPidParam.kp = 0.1f;
    chassisInitParam.rollCorrectionPidParam.ki = 0.0f;
    chassisInitParam.rollCorrectionPidParam.kd = 0.0f;
    chassisInitParam.rollCorrectionPidParam.Input_deadband = 1.0f;
    chassisInitParam.rollCorrectionPidParam.maxIntegral = 50.0f;
    chassisInitParam.rollCorrectionPidParam.maxOutput = 5000.0f; ///< roll轴pid待调
    chassisInitParam.MIT_L_kd = 0.0f;
    chassisInitParam.MIT_L_kp = 0.0f; // mit参数待调

    chassisInitParam.powerParamLF.kDefaultMaxPower = 30;
    chassisInitParam.powerParamLF.kTorqueCoeff  = 2.09688994e-6f;
    chassisInitParam.powerParamLF.k1 = 1.23e-07f;
    chassisInitParam.powerParamLF.k2 = 1.453e-08f;
    chassisInitParam.powerParamLF.kConstant = 5.581f;
    chassisInitParam.powerParamLF.kMotorOutputMax = 16000;
    
    chassisInitParam.powerParamRF.kDefaultMaxPower = 30;
    chassisInitParam.powerParamRF.kTorqueCoeff  = 2.09688994e-6f;
    chassisInitParam.powerParamRF.k1 = 1.23e-07f;
    chassisInitParam.powerParamRF.k2 = 1.453e-08f;
    chassisInitParam.powerParamRF.kConstant = 5.581f;
    chassisInitParam.powerParamRF.kMotorOutputMax = 16000;

    chassisInitParam.powerParamLB.kDefaultMaxPower = 30;
    chassisInitParam.powerParamLB.kTorqueCoeff  = 2.09688994e-6f;
    chassisInitParam.powerParamLB.k1 = 1.23e-07f;
    chassisInitParam.powerParamLB.k2 = 1.453e-08f;
    chassisInitParam.powerParamLB.kConstant = 5.581f;
    chassisInitParam.powerParamLB.kMotorOutputMax = 16000;

    chassisInitParam.powerParamRB.kDefaultMaxPower = 30;
    chassisInitParam.powerParamRB.kTorqueCoeff  = 2.09688994e-6f;
    chassisInitParam.powerParamRB.k1 = 1.23e-07f;
    chassisInitParam.powerParamRB.k2 = 1.453e-08f;
    chassisInitParam.powerParamRB.kConstant = 5.581f;
    chassisInitParam.powerParamRB.kMotorOutputMax = 16000;

    // 使用初始化后的参数创建 chassisModule 实例
    static auto chassisModule = CModChassis(chassisInitParam);




    return APP_OK;
}

} // namespace my_engineer
