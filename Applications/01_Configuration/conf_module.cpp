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

    /******初始化底盘模块******/
    CModChassis::SModInitParam_Chassis chassisInitParam;
    chassisInitParam.moduleID = EModuleID::MOD_CHASSIS;
    chassisInitParam.memsDevID = EDeviceID::DEV_MEMS_BMI088;
    chassisInitParam.FilterID = EAlgoID::ALGO_IMU_AVE;      //使用互补滤波
    chassisInitParam.wheelsetMotorID_LF = EDeviceID::DEV_CHAS_MTR_LF;
    chassisInitParam.wheelsetMotorID_RF = EDeviceID::DEV_CHAS_MTR_RF;
    chassisInitParam.wheelsetMotorID_LB = EDeviceID::DEV_CHAS_MTR_LB;
    chassisInitParam.wheelsetMotorID_RB = EDeviceID::DEV_CHAS_MTR_RB;
    chassisInitParam.steerMotorID_LF = EDeviceID::DEV_CHAS_STEER_LF;
    chassisInitParam.steerMotorID_RF = EDeviceID::DEV_CHAS_STEER_RF;
    chassisInitParam.steerMotorID_LB = EDeviceID::DEV_CHAS_STEER_LB;
    chassisInitParam.steerMotorID_RB = EDeviceID::DEV_CHAS_STEER_RB;

    // 设置can发送节点
    chassisInitParam.wheelsetMotorTxNode_LF = &TxNode_Can2_200;
    chassisInitParam.wheelsetMotorTxNode_RF = &TxNode_Can2_200;
    chassisInitParam.wheelsetMotorTxNode_LB = &TxNode_Can2_200;
    chassisInitParam.wheelsetMotorTxNode_RB = &TxNode_Can2_200;
    chassisInitParam.steerMotorTxNode_LF = &TxNode_Can1_1FF;
    chassisInitParam.steerMotorTxNode_RF = &TxNode_Can1_1FF;
    chassisInitParam.steerMotorTxNode_LB = &TxNode_Can1_1FF;
    chassisInitParam.steerMotorTxNode_RB = &TxNode_Can1_1FF;

    // 设置PID参数
    chassisInitParam.yawCorrectionPidParam.kp = 10.0f;
    chassisInitParam.yawCorrectionPidParam.ki = 0.0f;
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
    chassisInitParam.wheelsetSpdPidParam[0].kp = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[0].ki = 0.0f;
    chassisInitParam.wheelsetSpdPidParam[0].kd = 0.0f;
    chassisInitParam.wheelsetSpdPidParam[0].Input_deadband = 5.0f;
    chassisInitParam.wheelsetSpdPidParam[0].maxIntegral = 10000.0f;
    chassisInitParam.wheelsetSpdPidParam[0].maxOutput = 10000.0f;
    chassisInitParam.wheelsetSpdPidParam[1].kp = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[1].ki = 0.0f;
    chassisInitParam.wheelsetSpdPidParam[1].kd = 0.0f;
    chassisInitParam.wheelsetSpdPidParam[1].Input_deadband = 5.0f;
    chassisInitParam.wheelsetSpdPidParam[1].maxIntegral = 10000.0f;
    chassisInitParam.wheelsetSpdPidParam[1].maxOutput = 10000.0f;
    chassisInitParam.wheelsetSpdPidParam[2].kp = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[2].ki = 0.0f;
    chassisInitParam.wheelsetSpdPidParam[2].kd = 0.0f;
    chassisInitParam.wheelsetSpdPidParam[2].Input_deadband = 5.0f;
    chassisInitParam.wheelsetSpdPidParam[2].maxIntegral = 10000.0f;
    chassisInitParam.wheelsetSpdPidParam[2].maxOutput = 10000.0f;
    chassisInitParam.wheelsetSpdPidParam[3].kp = 1.0f;
    chassisInitParam.wheelsetSpdPidParam[3].ki = 0.0f;
    chassisInitParam.wheelsetSpdPidParam[3].kd = 0.0f;
    chassisInitParam.wheelsetSpdPidParam[3].Input_deadband = 5.0f;
    chassisInitParam.wheelsetSpdPidParam[3].maxIntegral = 10000.0f;
    chassisInitParam.wheelsetSpdPidParam[3].maxOutput = 10000.0f;        

    chassisInitParam.steerPosPidParam[0].kp = 0.3f;
    chassisInitParam.steerPosPidParam[0].ki = 0.0f;
    chassisInitParam.steerPosPidParam[0].kd = 0.0f;
    chassisInitParam.steerPosPidParam[0].Input_deadband = 0.0f;
    chassisInitParam.steerPosPidParam[0].maxIntegral = 0.0f;
    chassisInitParam.steerPosPidParam[0].maxOutput = 12000.0f;
    chassisInitParam.steerPosPidParam[0].errorMode = CAlgoPid::EPidErrorMode::MACHINE;
    chassisInitParam.steerPosPidParam[0].MachineModeErrorRange = 8192;

    chassisInitParam.steerPosPidParam[1].kp = 0.3f;
    chassisInitParam.steerPosPidParam[1].ki = 0.0f;
    chassisInitParam.steerPosPidParam[1].kd = 0.0f;
    chassisInitParam.steerPosPidParam[1].Input_deadband = 0.0f;
    chassisInitParam.steerPosPidParam[1].maxIntegral = 0.0f;
    chassisInitParam.steerPosPidParam[1].maxOutput = 12000.0f;
    chassisInitParam.steerPosPidParam[1].errorMode = CAlgoPid::EPidErrorMode::MACHINE;
    chassisInitParam.steerPosPidParam[1].MachineModeErrorRange = 8192;

    chassisInitParam.steerPosPidParam[2].kp = 0.3f;
    chassisInitParam.steerPosPidParam[2].ki = 0.0f;
    chassisInitParam.steerPosPidParam[2].kd = 0.0f;
    chassisInitParam.steerPosPidParam[2].Input_deadband = 0.0f;
    chassisInitParam.steerPosPidParam[2].maxIntegral = 0.0f;
    chassisInitParam.steerPosPidParam[2].maxOutput = 12000.0f;
    chassisInitParam.steerPosPidParam[2].errorMode = CAlgoPid::EPidErrorMode::MACHINE;
    chassisInitParam.steerPosPidParam[2].MachineModeErrorRange = 8192;

    chassisInitParam.steerPosPidParam[3].kp = 0.3f;
    chassisInitParam.steerPosPidParam[3].ki = 0.0f;
    chassisInitParam.steerPosPidParam[3].kd = 0.0f;
    chassisInitParam.steerPosPidParam[3].Input_deadband = 0.0f;
    chassisInitParam.steerPosPidParam[3].maxIntegral = 0.0f;
    chassisInitParam.steerPosPidParam[3].maxOutput = 12000.0f;
    chassisInitParam.steerPosPidParam[3].errorMode = CAlgoPid::EPidErrorMode::MACHINE;
    chassisInitParam.steerPosPidParam[3].MachineModeErrorRange = 8192;

    chassisInitParam.steerSpdPidParam[0].kp = 85.f;
    chassisInitParam.steerSpdPidParam[0].ki = 0.0f;
    chassisInitParam.steerSpdPidParam[0].kd = 0.f;
    chassisInitParam.steerSpdPidParam[0].Input_deadband = 0.0f;
    chassisInitParam.steerSpdPidParam[0].maxIntegral = 10000.0f;
    chassisInitParam.steerSpdPidParam[0].maxOutput = 16000.0f;

    chassisInitParam.steerSpdPidParam[1].kp = 85.f;
    chassisInitParam.steerSpdPidParam[1].ki = 0.0f;
    chassisInitParam.steerSpdPidParam[1].kd = 0.f;
    chassisInitParam.steerSpdPidParam[1].Input_deadband = 0.0f;
    chassisInitParam.steerSpdPidParam[1].maxIntegral = 10000.0f;
    chassisInitParam.steerSpdPidParam[1].maxOutput = 16000.0f;

    chassisInitParam.steerSpdPidParam[2].kp = 85.f;
    chassisInitParam.steerSpdPidParam[2].ki = 0.0f;
    chassisInitParam.steerSpdPidParam[2].kd = 0.f;
    chassisInitParam.steerSpdPidParam[2].Input_deadband = 0.0f;
    chassisInitParam.steerSpdPidParam[2].maxIntegral = 10000.0f;
    chassisInitParam.steerSpdPidParam[2].maxOutput = 16000.0f;

    chassisInitParam.steerSpdPidParam[3].kp = 85.0f;
    chassisInitParam.steerSpdPidParam[3].ki = 0.0f;
    chassisInitParam.steerSpdPidParam[3].kd = 0.f;
    chassisInitParam.steerSpdPidParam[3].Input_deadband = 0.0f;
    chassisInitParam.steerSpdPidParam[3].maxIntegral = 10000.0f;
    chassisInitParam.steerSpdPidParam[3].maxOutput = 16000.0f;

    chassisInitParam.powerParamLF.kDefaultMaxPower = 30;
    chassisInitParam.powerParamLF.kTorqueCoeff  = 1.1172101083651914e-06f;
    chassisInitParam.powerParamLF.k1 = 2.217009924016037e-07f;
    chassisInitParam.powerParamLF.k2 = 1.9290809050850713e-07f;
    chassisInitParam.powerParamLF.kConstant = 2.7857962849067786f;
    chassisInitParam.powerParamLF.kMotorOutputMax = 16000;

    chassisInitParam.powerParamRF.kDefaultMaxPower = 30;
    chassisInitParam.powerParamRF.kTorqueCoeff  = 1.1172101083651914e-06f;
    chassisInitParam.powerParamRF.k1 = 2.217009924016037e-07f;
    chassisInitParam.powerParamRF.k2 = 1.9290809050850713e-07f;
    chassisInitParam.powerParamRF.kConstant = 2.7857962849067786f;
    chassisInitParam.powerParamRF.kMotorOutputMax = 16000;

    chassisInitParam.powerParamLB.kDefaultMaxPower = 30;
    chassisInitParam.powerParamLB.kTorqueCoeff  = 1.1172101083651914e-06f;
    chassisInitParam.powerParamLB.k1 = 2.217009924016037e-07f;
    chassisInitParam.powerParamLB.k2 = 1.9290809050850713e-07f;
    chassisInitParam.powerParamLB.kConstant = 2.7857962849067786f;
    chassisInitParam.powerParamLB.kMotorOutputMax = 16000;

    chassisInitParam.powerParamRB.kDefaultMaxPower = 30;
    chassisInitParam.powerParamRB.kTorqueCoeff  = 1.1172101083651914e-06f;
    chassisInitParam.powerParamRB.k1 = 2.217009924016037e-07f;
    chassisInitParam.powerParamRB.k2 = 1.9290809050850713e-07f;
    chassisInitParam.powerParamRB.kConstant = 2.7857962849067786f;
    chassisInitParam.powerParamRB.kMotorOutputMax = 16000;

    chassisInitParam.steerPowerParamLF.kDefaultMaxPower = 20;
    chassisInitParam.steerPowerParamLF.kTorqueCoeff  = -5.07732860399221e-07f;
    chassisInitParam.steerPowerParamLF.k1 = 1.3080155950307635e-08f;
    chassisInitParam.steerPowerParamLF.k2 = 1.8747301277544523e-05f;
    chassisInitParam.steerPowerParamLF.kConstant = 3.1027061096343103f;
    chassisInitParam.steerPowerParamLF.kMotorOutputMax = 10000;

    chassisInitParam.steerPowerParamRF.kDefaultMaxPower = 20;
    chassisInitParam.steerPowerParamRF.kTorqueCoeff  = -5.07732860399221e-07f;
    chassisInitParam.steerPowerParamRF.k1 = 1.3080155950307635e-08f;
    chassisInitParam.steerPowerParamRF.k2 = 1.8747301277544523e-05f;
    chassisInitParam.steerPowerParamRF.kConstant = 3.1027061096343103f;
    chassisInitParam.steerPowerParamRF.kMotorOutputMax = 10000;

    chassisInitParam.steerPowerParamLB.kDefaultMaxPower = 20;
    chassisInitParam.steerPowerParamLB.kTorqueCoeff  = -5.07732860399221e-07f;
    chassisInitParam.steerPowerParamLB.k1 = 1.3080155950307635e-08f;
    chassisInitParam.steerPowerParamLB.k2 = 1.8747301277544523e-05f;
    chassisInitParam.steerPowerParamLB.kConstant = 3.1027061096343103f;
    chassisInitParam.steerPowerParamLB.kMotorOutputMax = 10000;

    chassisInitParam.steerPowerParamRB.kDefaultMaxPower = 20;
    chassisInitParam.steerPowerParamRB.kTorqueCoeff  = -5.07732860399221e-07f;
    chassisInitParam.steerPowerParamRB.k1 = 1.3080155950307635e-08f;
    chassisInitParam.steerPowerParamRB.k2 = 1.8747301277544523e-05f;
    chassisInitParam.steerPowerParamRB.kConstant = 3.1027061096343103f;
    chassisInitParam.steerPowerParamRB.kMotorOutputMax = 10000;

    // // 设置功率计CAN参数
    chassisInitParam.powerMeterCanID = EInterfaceID::INF_CAN2;  // 使用CAN1接口
    chassisInitParam.powerMeterStdID = 0x516;                   // 功率计CAN ID
    chassisInitParam.powerMeterFrameDlc = CInfCAN::ECanFrameDlc::DLC_8;
    // 使用初始化后的参数创建 chassisModule 实例
    static auto chassisModule = CModChassis(chassisInitParam);




    return APP_OK;
}

} // namespace my_engineer
