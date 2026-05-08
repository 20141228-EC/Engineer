/**
 * @file conf_interface.cpp
 * @author Zoe
 * @brief 完成所有设备的配置
 * @email 2328339747@qq.com
 * @date 2024-11-01
 * 
 * @details
 */

#include "conf_device.hpp"
#include "conf_CanTxNode.hpp"
#include "Device.hpp"

// extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

namespace my_engineer {

/**
 * @brief 配置并初始化所有设备
 * @return APP_OK - 初始化成功
 * @return APP_ERROR - 初始化失败
 */
EAppStatus InitAllDevice(){
    // 遥控器
    static CRcDR16 rc_dr16;
    CRcDR16::SRcDR16InitParam rc_dr16_initparam;
    rc_dr16_initparam.deviceID = EDeviceID::DEV_RC_DR16;
    rc_dr16_initparam.interfaceID = EInterfaceID::INF_DBUS;
    rc_dr16.InitDevice(&rc_dr16_initparam);

    // Bmi088
    static CMemsBmi088 bmi088;
    CMemsBmi088::SMemsInitParam_Bmi088 bmi088_initparam;
    bmi088_initparam.deviceID = EDeviceID::DEV_MEMS_BMI088;
    bmi088_initparam.interfaceID = EInterfaceID::INF_SPI2;
    bmi088_initparam.AccelUnitCsPort = BMI_ACC_CS_GPIO_Port;
    bmi088_initparam.AccelUnitCsPin = BMI_ACC_CS_Pin;
    bmi088_initparam.GyroUnitCsPort = BMI_GYRO_CS_GPIO_Port;
    bmi088_initparam.GyroUnitCsPin = BMI_GYRO_CS_Pin;
    bmi088_initparam.useTempControl = false;
    bmi088_initparam.tempTarget = 40.0f;
    bmi088_initparam.halTimHandle = &htim3;
    bmi088_initparam.halTimChannel = TIM_CHANNEL_4;
    bmi088_initparam.tempPidParam.kp = 10.0f;
    bmi088_initparam.tempPidParam.ki = 3.0f;
    bmi088_initparam.tempPidParam.kd = 0.5f;
    bmi088_initparam.tempPidParam.maxIntegral = 20;
    bmi088_initparam.tempPidParam.maxOutput = 100;
    bmi088.InitDevice(&bmi088_initparam);

    // 裁判系统
    static CDevReferee referee;
    CDevReferee::SDevInitParam_Referee referee_initparam;
    referee_initparam.deviceID = EDeviceID::DEV_RM_REFEREE;
    referee_initparam.interfaceID = EInterfaceID::INF_UART1;
    referee.InitDevice(&referee_initparam);

    // 视觉系统
    static CDevVision vision;
    CDevVision::SDevInitParam_Vision vision_initparam;
    vision_initparam.deviceID = EDeviceID::DEV_VISION;
    vision_initparam.interfaceID = EInterfaceID::INF_USB_CDC;
    vision.InitDevice(&vision_initparam);

    // 图传链路
    static CDevControllerLink controllerLink;
    CDevControllerLink::SDevInitParam_ControllerLink controllerLink_initparam;
    controllerLink_initparam.deviceID = EDeviceID::DEV_CONTROLLER_LINK;
    controllerLink_initparam.interfaceID = EInterfaceID::INF_UART10;
    controllerLink.InitDevice(&controllerLink_initparam);

    // ESP32
    static CDevESP32 esp32;
    CDevESP32::SDevInitParam_ESP32 esp32_initparam;
    esp32_initparam.deviceID = EDeviceID::DEV_ESP32;
    esp32_initparam.interfaceID = EInterfaceID::INF_UART7;
    esp32.InitDevice(&esp32_initparam);

    // 板间通信
    static CDevBoardLink boardLink;
    CDevBoardLink::SDevInitParam_BoardLink boardLink_initparam;
    boardLink_initparam.deviceID = EDeviceID::DEV_BOARD_LINK;
    boardLink_initparam.interfaceID = EInterfaceID::INF_CAN3;
    boardLink_initparam.offlineTimeout = 100;  // 100ms离线超时
    boardLink.InitDevice(&boardLink_initparam);

    /******************************************
    * 子龙门电机 - 已删除 
    ******************************************/
/*
    static CDevMtrM2006 subGantryMotor_Lift_L;
    CDevMtrM2006::SMtrInitParam_M2006 subGantryMotor_Lift_L_initparam;
    subGantryMotor_Lift_L_initparam.deviceID = EDeviceID::DEV_SUBGANTRY_MTR_LIFT_L;
    subGantryMotor_Lift_L_initparam.interfaceID = EInterfaceID::INF_CAN2;
    subGantryMotor_Lift_L_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_1;
    subGantryMotor_Lift_L_initparam.useAngleToPosit = true;
    subGantryMotor_Lift_L_initparam.useStallMonit = true;
    subGantryMotor_Lift_L_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    subGantryMotor_Lift_L.InitDevice(&subGantryMotor_Lift_L_initparam);

    static CDevMtrM2006 subGantryMotor_Lift_R;
    CDevMtrM2006::SMtrInitParam_M2006 subGantryMotor_Lift_R_initparam;
    subGantryMotor_Lift_R_initparam.deviceID = EDeviceID::DEV_SUBGANTRY_MTR_LIFT_R;
    subGantryMotor_Lift_R_initparam.interfaceID = EInterfaceID::INF_CAN2;
    subGantryMotor_Lift_R_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_2;
    subGantryMotor_Lift_R_initparam.useAngleToPosit = true;
    subGantryMotor_Lift_R_initparam.useStallMonit = true;
    subGantryMotor_Lift_R_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    subGantryMotor_Lift_R.InitDevice(&subGantryMotor_Lift_R_initparam);

    static CDevMtrM2006 subGantryMotor_Stretch_L;
    CDevMtrM2006::SMtrInitParam_M2006 subGantryMotor_Stretch_L_initparam;
    subGantryMotor_Stretch_L_initparam.deviceID = EDeviceID::DEV_SUBGANTRY_MTR_STRETCH_L;
    subGantryMotor_Stretch_L_initparam.interfaceID = EInterfaceID::INF_CAN2;
    subGantryMotor_Stretch_L_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_3;
    subGantryMotor_Stretch_L_initparam.useAngleToPosit = true;
    subGantryMotor_Stretch_L_initparam.useStallMonit = true;
    subGantryMotor_Stretch_L_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    subGantryMotor_Stretch_L.InitDevice(&subGantryMotor_Stretch_L_initparam);

    static CDevMtrM2006 subGantryMotor_Stretch_R;
    CDevMtrM2006::SMtrInitParam_M2006 subGantryMotor_Stretch_R_initparam;
    subGantryMotor_Stretch_R_initparam.deviceID = EDeviceID::DEV_SUBGANTRY_MTR_STRETCH_R;
    subGantryMotor_Stretch_R_initparam.interfaceID = EInterfaceID::INF_CAN2;
    subGantryMotor_Stretch_R_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_4;
    subGantryMotor_Stretch_R_initparam.useAngleToPosit = true;
    subGantryMotor_Stretch_R_initparam.useStallMonit = true;
    subGantryMotor_Stretch_R_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    subGantryMotor_Stretch_R.InitDevice(&subGantryMotor_Stretch_R_initparam);
*/

    /******************************************
     * 云台电机
     *****************************************/
    // 大yaw
    static CDevMtrKT gimbalMotor_Yaw;
    CDevMtrKT::SMtrInitParam_KT gimbalMotor_Yaw_initparam;
    gimbalMotor_Yaw_initparam.deviceID = EDeviceID::DEV_GIMBAL_MTR_YAW;
    gimbalMotor_Yaw_initparam.interfaceID = EInterfaceID::INF_CAN3;
    gimbalMotor_Yaw_initparam.ktMtrID = CDevMtrKT::EKtMtrID::ID_1;
    gimbalMotor_Yaw_initparam.encoderResolution = 65535;
    gimbalMotor_Yaw_initparam.useAngleToPosit = true;
    gimbalMotor_Yaw_initparam.useStallMonit = true;
    gimbalMotor_Yaw_initparam.stallMonitDataSrc = CDevMtr::DATA_CURRENT;
    gimbalMotor_Yaw_initparam.stallThreshold = 200;
    gimbalMotor_Yaw_initparam.stallTime = 500;
    gimbalMotor_Yaw.InitDevice(&gimbalMotor_Yaw_initparam);

    // // 云台存矿电机 - 前
    // static CDevMtrM2006 gimbalMotor_Storage_F;
    // CDevMtrM2006::SMtrInitParam_M2006 gimbalMotor_Storage_F_initparam;
    // gimbalMotor_Storage_F_initparam.deviceID = EDeviceID::DEV_GIMBAL_MTR_STORAGE_F;
    // gimbalMotor_Storage_F_initparam.interfaceID = EInterfaceID::INF_CAN1;
    // gimbalMotor_Storage_F_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_5;
    // gimbalMotor_Storage_F_initparam.useAngleToPosit = true;
    // gimbalMotor_Storage_F_initparam.useStallMonit = true;
    // gimbalMotor_Storage_F_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    // gimbalMotor_Storage_F.InitDevice(&gimbalMotor_Storage_F_initparam);

    // // 云台存矿电机 - 后
    // static CDevMtrM2006 gimbalMotor_Storage_B;
    // CDevMtrM2006::SMtrInitParam_M2006 gimbalMotor_Storage_B_initparam;
    // gimbalMotor_Storage_B_initparam.deviceID = EDeviceID::DEV_GIMBAL_MTR_STORAGE_B;
    // gimbalMotor_Storage_B_initparam.interfaceID = EInterfaceID::INF_CAN1;
    // gimbalMotor_Storage_B_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_6;
    // gimbalMotor_Storage_B_initparam.useAngleToPosit = true;
    // gimbalMotor_Storage_B_initparam.useStallMonit = true;
    // gimbalMotor_Storage_B_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    // gimbalMotor_Storage_B.InitDevice(&gimbalMotor_Storage_B_initparam);

    // static CDevMtrM2006 gimbalMotor_Pitch;
    // CDevMtrM2006::SMtrInitParam_M2006 gimbalMotor_Pitch_initparam;
    // gimbalMotor_Pitch_initparam.deviceID = EDeviceID::DEV_GIMBAL_MTR_PITCH;
    // gimbalMotor_Pitch_initparam.interfaceID = EInterfaceID::INF_CAN1;
    // gimbalMotor_Pitch_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_7;
    // gimbalMotor_Pitch_initparam.useAngleToPosit = true;
    // gimbalMotor_Pitch_initparam.useStallMonit = true;
    // gimbalMotor_Pitch_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    // gimbalMotor_Pitch.InitDevice(&gimbalMotor_Pitch_initparam);

    /******************************************
     * 机械臂电机
     *****************************************/
    static CDevMtrM2006 armMotor_End_L;
    CDevMtrM2006::SMtrInitParam_M2006 armMotor_End_L_initparam;
    armMotor_End_L_initparam.deviceID = EDeviceID::DEV_ARM_MTR_END_L;
    armMotor_End_L_initparam.interfaceID = EInterfaceID::INF_CAN2;
    armMotor_End_L_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_5;
    armMotor_End_L_initparam.useAngleToPosit = true;
    armMotor_End_L_initparam.useStallMonit = true;
    armMotor_End_L_initparam.stallThreshold = 900.0f; // 设置堵转阈值
    armMotor_End_L_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    armMotor_End_L.InitDevice(&armMotor_End_L_initparam);

    static CDevMtrM2006 armMotor_End_R;
    CDevMtrM2006::SMtrInitParam_M2006 armMotor_End_R_initparam;
    armMotor_End_R_initparam.deviceID = EDeviceID::DEV_ARM_MTR_END_R;
    armMotor_End_R_initparam.interfaceID = EInterfaceID::INF_CAN2;
    armMotor_End_R_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_6;
    armMotor_End_R_initparam.useAngleToPosit = true;
    armMotor_End_R_initparam.useStallMonit = true;
    armMotor_End_R_initparam.stallThreshold = 900.0f; // 设置堵转阈值
    armMotor_End_R_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    armMotor_End_R.InitDevice(&armMotor_End_R_initparam);

    static CDevMtrKT armMotor_Yaw;
    CDevMtrKT::SMtrInitParam_KT armMotor_Yaw_initparam;
    armMotor_Yaw_initparam.deviceID = EDeviceID::DEV_ARM_MTR_YAW;
    armMotor_Yaw_initparam.interfaceID = EInterfaceID::INF_CAN1;
    armMotor_Yaw_initparam.ktMtrID = CDevMtrKT::EKtMtrID::ID_1;
    armMotor_Yaw_initparam.encoderResolution = 65535;
    armMotor_Yaw_initparam.useAngleToPosit = true;
    armMotor_Yaw_initparam.useStallMonit = true;
    armMotor_Yaw_initparam.stallMonitDataSrc = CDevMtr::DATA_CURRENT;
    armMotor_Yaw_initparam.stallThreshold = 200;
    armMotor_Yaw_initparam.stallTime = 200;
    armMotor_Yaw.InitDevice(&armMotor_Yaw_initparam);

    static CDevMtrKT armMotor_Pitch1;
    CDevMtrKT::SMtrInitParam_KT armMotor_Pitch1_initparam;
    armMotor_Pitch1_initparam.deviceID = EDeviceID::DEV_ARM_MTR_PITCH1;
    armMotor_Pitch1_initparam.interfaceID = EInterfaceID::INF_CAN1;
    armMotor_Pitch1_initparam.ktMtrID = CDevMtrKT::EKtMtrID::ID_2;
    armMotor_Pitch1_initparam.encoderResolution = 65535;
    armMotor_Pitch1_initparam.useAngleToPosit = true;
    armMotor_Pitch1_initparam.useStallMonit = true;
    armMotor_Pitch1_initparam.stallMonitDataSrc = CDevMtr::DATA_CURRENT;
    armMotor_Pitch1_initparam.stallThreshold = 200;
    armMotor_Pitch1_initparam.stallTime = 200;
    armMotor_Pitch1.InitDevice(&armMotor_Pitch1_initparam);

    static CDevMtrKT armMotor_Pitch2;
    CDevMtrKT::SMtrInitParam_KT armMotor_Pitch2_initparam;
    armMotor_Pitch2_initparam.deviceID = EDeviceID::DEV_ARM_MTR_PITCH2;
    armMotor_Pitch2_initparam.interfaceID = EInterfaceID::INF_CAN1;
    armMotor_Pitch2_initparam.ktMtrID = CDevMtrKT::EKtMtrID::ID_3;
    armMotor_Pitch2_initparam.encoderResolution = 65535;
    armMotor_Pitch2_initparam.useAngleToPosit = true;
    armMotor_Pitch2_initparam.useStallMonit = true;
    armMotor_Pitch2_initparam.stallMonitDataSrc = CDevMtr::DATA_CURRENT;
    armMotor_Pitch2_initparam.stallThreshold = 200;
    armMotor_Pitch2_initparam.stallTime = 200;
    armMotor_Pitch2.InitDevice(&armMotor_Pitch2_initparam);

    static CDevMtrKT armMotor_Pitch3;
    CDevMtrKT::SMtrInitParam_KT armMotor_Pitch3_initparam;
    armMotor_Pitch3_initparam.deviceID = EDeviceID::DEV_ARM_MTR_PITCH3;
    armMotor_Pitch3_initparam.interfaceID = EInterfaceID::INF_CAN1;
    armMotor_Pitch3_initparam.ktMtrID = CDevMtrKT::EKtMtrID::ID_4;
    armMotor_Pitch3_initparam.encoderResolution = 65535;
    armMotor_Pitch3_initparam.useAngleToPosit = true;
    armMotor_Pitch3_initparam.useStallMonit = true;
    armMotor_Pitch3_initparam.stallMonitDataSrc = CDevMtr::DATA_CURRENT;
    armMotor_Pitch3_initparam.stallThreshold = 200;
    armMotor_Pitch3_initparam.stallTime = 200;
    armMotor_Pitch3.InitDevice(&armMotor_Pitch3_initparam);

    static CDevMtrDM_MIT armMotor_Roll;
    CDevMtrDM_MIT::SMtrInitParam_DM_MIT armMotor_Roll_initparam;
    armMotor_Roll_initparam.deviceID = EDeviceID::DEV_ARM_MTR_ROLL;
    armMotor_Roll_initparam.interfaceID = EInterfaceID::INF_CAN2;
    armMotor_Roll_initparam.MasterID = 0x30;
    armMotor_Roll_initparam.SlaveID = 0x31;
    armMotor_Roll_initparam.Q_MAX = 3.1416f;
    armMotor_Roll_initparam.DQ_MAX = 30.0f;
    armMotor_Roll_initparam.TAU_MAX = 10.0f;
    armMotor_Roll_initparam.useAngleToPosit = true;
    armMotor_Roll.InitDevice(&armMotor_Roll_initparam);

    /******************************************
    * 机械臂夹爪电机
    ******************************************/
    static CDevMtrM2006 armMotor_Grip;
    CDevMtrM2006::SMtrInitParam_M2006 armMotor_Grip_initparam;
    armMotor_Grip_initparam.deviceID = EDeviceID::DEV_ARM_MTR_GRIP;
    armMotor_Grip_initparam.interfaceID = EInterfaceID::INF_CAN2;
    armMotor_Grip_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_7;//暂时使用can2的ID8
    armMotor_Grip_initparam.useAngleToPosit = true;
    armMotor_Grip_initparam.useStallMonit = true;
    armMotor_Grip_initparam.stallThreshold = 1000.0f; // 设置堵转阈值
    armMotor_Grip_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE; //堵转的选项设置为电流
    armMotor_Grip.InitDevice(&armMotor_Grip_initparam);

    return APP_OK;
}

} // namespace my_engineer
