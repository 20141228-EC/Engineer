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
    armInitParam.MotorTxNode_Yaw = &TxNode_Can3_280;
    armInitParam.MotorTxNode_Pitch1 = &TxNode_Can3_280;
    armInitParam.MotorTxNode_Pitch2 = &TxNode_Can3_280;
    armInitParam.MotorTxNode_Pitch3 = &TxNode_Can3_280;
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
    chassisInitParam.crawlerMotorID_L = EDeviceID::DEV_CHAS_CRAWLER_L;
    chassisInitParam.crawlerMotorID_R = EDeviceID::DEV_CHAS_CRAWLER_R;
    // 设置can发送节点
    chassisInitParam.wheelsetMotorTxNode_LF = &TxNode_Can1_200;
    chassisInitParam.wheelsetMotorTxNode_RF = &TxNode_Can1_200;
    chassisInitParam.wheelsetMotorTxNode_LB = &TxNode_Can1_200;
    chassisInitParam.wheelsetMotorTxNode_RB = &TxNode_Can1_200;
    chassisInitParam.crawlerMotorTxNodeID_L = &TxNode_Can3_200;
    chassisInitParam.crawlerMotorTxNodeID_R = &TxNode_Can3_200;
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
    // 为每个轮毂设置独立的PID参数
    chassisInitParam.wheelsetSpdPidParam[3].kp = 8.f;
    chassisInitParam.wheelsetSpdPidParam[3].ki = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[3].kd = 0.0f;
    chassisInitParam.wheelsetSpdPidParam[3].Input_deadband = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[3].maxIntegral = 4000.0f;
    chassisInitParam.wheelsetSpdPidParam[3].maxOutput = 20000.0f;
    chassisInitParam.wheelsetSpdPidParam[0].kp = 8.f;
    chassisInitParam.wheelsetSpdPidParam[0].ki = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[0].kd = 0.0f;    
    chassisInitParam.wheelsetSpdPidParam[0].Input_deadband = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[0].maxIntegral = 4000.0f;
    chassisInitParam.wheelsetSpdPidParam[0].maxOutput = 20000.0f;
    chassisInitParam.wheelsetSpdPidParam[1].kp = 8.f;
    chassisInitParam.wheelsetSpdPidParam[1].ki = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[1].kd = 0.0f;
    chassisInitParam.wheelsetSpdPidParam[1].Input_deadband = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[1].maxIntegral = 4000.0f;
    chassisInitParam.wheelsetSpdPidParam[1].maxOutput = 20000.0f;
    chassisInitParam.wheelsetSpdPidParam[2].kp = 8.0f;
    chassisInitParam.wheelsetSpdPidParam[2].ki = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[2].kd = 0.0f;
    chassisInitParam.wheelsetSpdPidParam[2].Input_deadband = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[2].maxIntegral = 4000.0f;
    chassisInitParam.wheelsetSpdPidParam[2].maxOutput = 20000.0f;        

    // 髋关节组件pid
    chassisInitParam.rollCorrectionPidParam.kp = 0.03f;
    chassisInitParam.rollCorrectionPidParam.ki = 0.00f;
    chassisInitParam.rollCorrectionPidParam.kd = 0.0f;
    chassisInitParam.rollCorrectionPidParam.Input_deadband = 1.0f;
    chassisInitParam.rollCorrectionPidParam.maxIntegral = 50.0f;
    chassisInitParam.rollCorrectionPidParam.maxOutput = 1000.0f; ///< roll轴pid待调
    chassisInitParam.MIT_L_kp = 260.f;//15.0f;
    chassisInitParam.MIT_L_kd = 2.f;//1.0f; // mit参数待调
    chassisInitParam.MIT_L_tau = 1.f;
    chassisInitParam.MIT_R_kp = 310.f;
    chassisInitParam.MIT_R_kd = 2.f;
    chassisInitParam.MIT_R_tau = -1.f;

    chassisInitParam.HipPosPidParam_L.kp = 0.f;
    chassisInitParam.HipPosPidParam_L.ki = 0.f;
    chassisInitParam.HipPosPidParam_L.kd = 0.f;
    chassisInitParam.HipPosPidParam_L.maxIntegral = 1000.f;
    chassisInitParam.HipPosPidParam_L.maxOutput = 10000.f;
    chassisInitParam.HipPosPidParam_L.MachineModeErrorRange = 65535;
    chassisInitParam.HipPosPidParam_L.kp = 0.f;
    chassisInitParam.HipPosPidParam_L.ki = 0.f;
    chassisInitParam.HipPosPidParam_L.kd = 0.f;
    chassisInitParam.HipPosPidParam_L.maxIntegral = 1000.f;
    chassisInitParam.HipPosPidParam_L.maxOutput = 10000.f;
    chassisInitParam.HipSpdPidParam_L.kp = 0.f;
    chassisInitParam.HipSpdPidParam_L.ki = 0.f;
    chassisInitParam.HipSpdPidParam_L.kd = 0.f;
    chassisInitParam.HipSpdPidParam_L.maxIntegral = 1000.f;
    chassisInitParam.HipSpdPidParam_L.maxOutput = 10000.f;
    chassisInitParam.HipPosPidParam_R.MachineModeErrorRange = 65535;
    chassisInitParam.HipSpdPidParam_R.kp = 0.f;
    chassisInitParam.HipSpdPidParam_R.ki = 0.f;
    chassisInitParam.HipSpdPidParam_R.kd = 0.f;
    chassisInitParam.HipSpdPidParam_R.maxIntegral = 1000.f;
    chassisInitParam.HipSpdPidParam_R.maxOutput = 10000.f;

    chassisInitParam.CrawlerSpdPidParam.kp = 5.5f;
    chassisInitParam.CrawlerSpdPidParam.ki = 1.0f;
    chassisInitParam.CrawlerSpdPidParam.kd = 0.f;
    chassisInitParam.CrawlerSpdPidParam.maxIntegral = 4000.f;
    chassisInitParam.CrawlerSpdPidParam.maxOutput = 15000.f;

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

   /******初始化云台模块******/
    CModGimbal::SModInitParam_Gimbal gimbalInitParam;
    gimbalInitParam.yawVisualMotorID = EDeviceID::DEV_GIMBAL_MTR_VISUAL_YAW;
    gimbalInitParam.moduleID = EModuleID::MOD_GIMBAL;
    gimbalInitParam.MIT_YAW_kd = 0.06f;
    gimbalInitParam.MIT_YAW_kp = 1;
    // 使用初始化后的参数创建 gimbalModule 实例
    static auto gimbalModule = CModGimbal(gimbalInitParam);
    return APP_OK;
}

} // namespace my_engineer
