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
    armInitParam.MotorID_End_Pitch = EDeviceID::DEV_ARM_MTR_END_PITCH;
    armInitParam.MotorID_End_Roll = EDeviceID::DEV_ARM_MTR_END_ROLL;
    armInitParam.MotorID_Grip = EDeviceID::DEV_ARM_MTR_GRIP;
    // 设置can发送节点
    armInitParam.MotorTxNode_Yaw = &TxNode_Can3_280;
    armInitParam.MotorTxNode_Pitch1 = &TxNode_Can3_280;
    armInitParam.MotorTxNode_Pitch2 = &TxNode_Can3_280;
    armInitParam.MotorTxNode_Grip = &TxNode_Can3_280;
    // 初始化 YawPosPidParam 的成员
   armInitParam.YawPosPidParam.kp = 2.5f;
   armInitParam.YawPosPidParam.ki = 0.1f;
   armInitParam.YawPosPidParam.kd = 20.f;
   armInitParam.YawPosPidParam.maxIntegral = 3000.0f;
   armInitParam.YawPosPidParam.maxOutput = 5000.0f;
   // 初始化 YawSpdPidParam 的成员
   armInitParam.YawSpdPidParam.kp = 0.1f;
   armInitParam.YawSpdPidParam.ki = 0.05f;
   armInitParam.YawSpdPidParam.kd = 0.0f;
   armInitParam.YawSpdPidParam.maxIntegral = 3000.0f;
   armInitParam.YawSpdPidParam.maxOutput = 3000.0f;
    // 初始化 Pitch1PosPidParam 的成员
   armInitParam.Pitch1PosPidParam.kp = 4.2f;
   armInitParam.Pitch1PosPidParam.ki = 0.15f;
   armInitParam.Pitch1PosPidParam.kd = 0.0f;
   armInitParam.Pitch1PosPidParam.maxIntegral = 3000.0f;
   armInitParam.Pitch1PosPidParam.maxOutput = 4000.0f;
   // 初始化 Pitch1SpdPidParam 的成员
   armInitParam.Pitch1SpdPidParam.kp = 0.1f;
   armInitParam.Pitch1SpdPidParam.ki = 0.1f;
   armInitParam.Pitch1SpdPidParam.kd = 0.0f;
   armInitParam.Pitch1SpdPidParam.maxIntegral = 2000.0f;
   armInitParam.Pitch1SpdPidParam.maxOutput = 2000.0f;
   // 初始化 Pitch2PosPidParam 的成员
   armInitParam.Pitch2PosPidParam.kp = 2.f;
   armInitParam.Pitch2PosPidParam.ki = 0.05f;
   armInitParam.Pitch2PosPidParam.kd = 0.0f;
   armInitParam.Pitch2PosPidParam.maxIntegral = 2000.0f;
   armInitParam.Pitch2PosPidParam.maxOutput = 5000.0f;
   // 初始化 Pitch2SpdPidParam 的成员
   armInitParam.Pitch2SpdPidParam.kp = 0.1f;
   armInitParam.Pitch2SpdPidParam.ki = 0.05f;
   armInitParam.Pitch2SpdPidParam.kd = 0.0f;
   armInitParam.Pitch2SpdPidParam.maxIntegral = 2000.0f;
   armInitParam.Pitch2SpdPidParam.maxOutput = 2000.0f;

       // 重力补偿参数
    armInitParam.gravParam.ramp_alpha = 0.05f;

    // Pitch1
    armInitParam.gravParam.pitch1_zero_deg = 45.0f;           ///< 零点偏移
    armInitParam.gravParam.pitch1_motor.kt = 1.09f;         ///< 电机力矩系数
    armInitParam.gravParam.pitch1_motor.reduction = 36.0f;  ///< 减速比
    armInitParam.gravParam.pitch1_motor.amp_to_raw = 0.0f;  ///< 物理电流转换系数
    armInitParam.gravParam.pitch1_ff_limit = 1000.0f;       ///< 前馈限幅
    armInitParam.gravParam.ws_pitch1_min = -180.0f;        ///< 工作空间下限
    armInitParam.gravParam.ws_pitch1_max = 180.0f;         ///< 工作空间上限
    armInitParam.gravParam.pitch1_motor.type = EMotorType::MG8010_I36V2; ///< 电机型号

    // Pitch2
    armInitParam.gravParam.pitch2_zero_deg = 0.0f;
    armInitParam.gravParam.pitch2_motor.kt = 0.175f;
    armInitParam.gravParam.pitch2_motor.reduction = 36.0f;
    armInitParam.gravParam.pitch2_motor.direction = 1.0f;
    armInitParam.gravParam.pitch2_motor.amp_to_raw = 0.0f;
    armInitParam.gravParam.pitch2_ff_limit = 1000.0f;
    armInitParam.gravParam.ws_pitch2_min = -180.0f;
    armInitParam.gravParam.ws_pitch2_max = 180.0f;
    armInitParam.gravParam.pitch2_motor.type = EMotorType::MG6012_I36V3;

    // Roll
    armInitParam.gravParam.roll_tau_limit = 10.f;
    armInitParam.gravParam.ws_roll_min = -180.0f;
    armInitParam.gravParam.ws_roll_max = 180.0f;
    armInitParam.gravParam.roll_motor.direction = 1.0f;
    armInitParam.gravParam.roll_motor.reduction = 1.0f;
    armInitParam.gravParam.roll_motor.type = EMotorType::DM_MIT;

    // 末端Roll
    armInitParam.gravParam.end_roll_zero_deg = 0.0f;
    armInitParam.gravParam.end_roll_motor.direction = -1.0f;
    armInitParam.gravParam.end_roll_motor.reduction = 1.0f;
    armInitParam.gravParam.end_roll_motor.type = EMotorType::DM_MIT;

    // 末端Pitch
    armInitParam.gravParam.end_pitch_zero_deg = 0.0f;
    armInitParam.gravParam.end_pitch_motor.direction = 1.0f;
    armInitParam.gravParam.end_pitch_motor.reduction = 1.0f;
    armInitParam.gravParam.end_pitch_motor.type = EMotorType::DM_MIT;


   // 初始化 mitCtrl_Roll 的成员
   armInitParam.MIT_Roll_kp = 45.0f;
   armInitParam.MIT_Roll_kd = 1.5f;

   armInitParam.MIT_End_Pitch_kp = 50.0f;
   armInitParam.MIT_End_Pitch_kd = 1.5f;
   
   armInitParam.MIT_End_Roll_kp = 25.0f;
   armInitParam.MIT_End_Roll_kd = 2.0f;
   // 初始化 GripPosPidParam 的成员
   armInitParam.GripPosPidParam.kp = 5.3f;
    armInitParam.GripPosPidParam.ki = 0.15f;
    armInitParam.GripPosPidParam.kd = 0.0f;
    armInitParam.GripPosPidParam.maxOutput = 3000.0f;
    // 初始化 GripSpdPidParam 的成员
    armInitParam.GripSpdPidParam.kp = 0.08f;
    armInitParam.GripSpdPidParam.ki = 0.5f;
    armInitParam.GripSpdPidParam.kd = 0.0f;
    armInitParam.GripSpdPidParam.maxIntegral = 2000.0f;
    armInitParam.GripSpdPidParam.maxOutput = 1000.0f;
    //armInitParam.GripSpdPidParam.input_integralSeparation = 5000.0f;  ///< 误差过大时清零积分，防止撞限位时积分过冲误判自锁
    // 夹取检测参数
    armInitParam.GripDetectParam.closeTorqueThresh = 60.0f;  ///< 滤波电流接触阈值
    armInitParam.GripDetectParam.closeTorqueRange  = 100.0f;  ///< 接触到夹紧的电流区间
    armInitParam.GripDetectParam.detectTorque      = 80.0f;  ///< 夹取成功阈值
    armInitParam.GripDetectParam.filterAlpha       = 0.90f;  ///< LowPassFilter滤波系数
    // armInitParam.Need_Grav_Compensation = false;
    // 使用初始化后的参数创建 armModule 实例 
    static auto armModule = CModArm(armInitParam);

    /******初始化底盘模块******/
    CModChassis::SModInitParam_Chassis chassisInitParam;
    chassisInitParam.moduleID = EModuleID::MOD_CHASSIS;
    chassisInitParam.memsDevID = EDeviceID::DEV_MEMS_BMI088;
    // chassisInitParam.FilterID = EAlgoID::ALGO_IMU_AVE;      //使用互补滤波
    chassisInitParam.FilterID = EAlgoID::ALGO_IMU_EKF;
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
    gimbalInitParam.moduleID = EModuleID::MOD_GIMBAL;

    // Yaw轴 (DM_MIT)
    gimbalInitParam.yawVisualMotorID = EDeviceID::DEV_GIMBAL_MTR_VISUAL_YAW;
    gimbalInitParam.MIT_YAW_kd = 0.06f;
    gimbalInitParam.MIT_YAW_kp = 1;

    // Pitch轴 (DJI M2006, CAN2)
    gimbalInitParam.pitchMotorID = EDeviceID::DEV_GIMBAL_MTR_PITCH;
    gimbalInitParam.pitchMotorTxNode = &TxNode_Can2_200;
    // Pitch位置PID参数
    gimbalInitParam.PitchPosPidParam.kp = 0.23f;
    gimbalInitParam.PitchPosPidParam.ki = 0.0f;
    gimbalInitParam.PitchPosPidParam.kd = 0.0f;
    gimbalInitParam.PitchPosPidParam.maxIntegral = 3000.0f;
    gimbalInitParam.PitchPosPidParam.maxOutput = 4000.0f;
    // Pitch速度PID参数
    gimbalInitParam.PitchSpdPidParam.kp = 0.5f;
    gimbalInitParam.PitchSpdPidParam.ki = 0.05f;
    gimbalInitParam.PitchSpdPidParam.kd = 0.1f;
    gimbalInitParam.PitchSpdPidParam.maxIntegral = 3000.0f;
    gimbalInitParam.PitchSpdPidParam.maxOutput = 4000.0f;

    // 使用初始化后的参数创建 gimbalModule 实例
    static auto gimbalModule = CModGimbal(gimbalInitParam);
    return APP_OK;
}

} // namespace my_engineer
