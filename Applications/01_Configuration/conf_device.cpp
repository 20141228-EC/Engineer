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
    CDevBoardLink::SDevInitParam_BoardLink boardLink_Initparam;
    boardLink_Initparam.deviceID = EDeviceID::DEV_BOARD_LINK;
    boardLink_Initparam.interfaceID = EInterfaceID::INF_CAN3;
    boardLink.InitDevice(&boardLink_Initparam);

    /******************************************
    * 机械臂末端电机
    ******************************************/

    // static CDevMtrM2006 subGantryMotor_Lift_L;
    // CDevMtrM2006::SMtrInitParam_M2006 subGantryMotor_Lift_L_initparam;
    // subGantryMotor_Lift_L_initparam.deviceID = EDeviceID::DEV_SUBGANTRY_MTR_LIFT_L;
    // subGantryMotor_Lift_L_initparam.interfaceID = EInterfaceID::INF_CAN2;
    // subGantryMotor_Lift_L_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_1;
    // subGantryMotor_Lift_L_initparam.useAngleToPosit = true;
    // subGantryMotor_Lift_L_initparam.useStallMonit = true;
    // subGantryMotor_Lift_L_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    // subGantryMotor_Lift_L.InitDevice(&subGantryMotor_Lift_L_initparam);

    // static CDevMtrM2006 subGantryMotor_Lift_R;
    // CDevMtrM2006::SMtrInitParam_M2006 subGantryMotor_Lift_R_initparam;
    // subGantryMotor_Lift_R_initparam.deviceID = EDeviceID::DEV_SUBGANTRY_MTR_LIFT_R;
    // subGantryMotor_Lift_R_initparam.interfaceID = EInterfaceID::INF_CAN2;
    // subGantryMotor_Lift_R_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_2;
    // subGantryMotor_Lift_R_initparam.useAngleToPosit = true;
    // subGantryMotor_Lift_R_initparam.useStallMonit = true;
    // subGantryMotor_Lift_R_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    // subGantryMotor_Lift_R.InitDevice(&subGantryMotor_Lift_R_initparam);

    // static CDevMtrM2006 subGantryMotor_Stretch_L;
    // CDevMtrM2006::SMtrInitParam_M2006 subGantryMotor_Stretch_L_initparam;
    // subGantryMotor_Stretch_L_initparam.deviceID = EDeviceID::DEV_SUBGANTRY_MTR_STRETCH_L;
    // subGantryMotor_Stretch_L_initparam.interfaceID = EInterfaceID::INF_CAN2;
    // subGantryMotor_Stretch_L_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_3;
    // subGantryMotor_Stretch_L_initparam.useAngleToPosit = true;
    // subGantryMotor_Stretch_L_initparam.useStallMonit = true;
    // subGantryMotor_Stretch_L_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    // subGantryMotor_Stretch_L.InitDevice(&subGantryMotor_Stretch_L_initparam);

    /******************************************
    * 底盘电机
    ******************************************/
    static CDevMtrM3508 chassisMotor_LF;
    CDevMtrM3508::SMtrInitParam_M3508 chassisMotor_LF_initparam;
    chassisMotor_LF_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_LF;
    chassisMotor_LF_initparam.interfaceID = EInterfaceID::INF_CAN1;
    chassisMotor_LF_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_1;
    chassisMotor_LF.InitDevice(&chassisMotor_LF_initparam);

    static CDevMtrM3508 chassisMotor_RF;
    CDevMtrM3508::SMtrInitParam_M3508 chassisMotor_RF_initparam;
    chassisMotor_RF_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_RF;
    chassisMotor_RF_initparam.interfaceID = EInterfaceID::INF_CAN1;
    chassisMotor_RF_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_2;
    chassisMotor_RF.InitDevice(&chassisMotor_RF_initparam);

    static CDevMtrM3508 chassisMotor_LB;
    CDevMtrM3508::SMtrInitParam_M3508 chassisMotor_LB_initparam;
    chassisMotor_LB_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_LB;
    chassisMotor_LB_initparam.interfaceID = EInterfaceID::INF_CAN1;
    chassisMotor_LB_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_4;
    chassisMotor_LB.InitDevice(&chassisMotor_LB_initparam);

    static CDevMtrM3508 chassisMotor_RB;
    CDevMtrM3508::SMtrInitParam_M3508 chassisMotor_RB_initparam;
    chassisMotor_RB_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_RB;
    chassisMotor_RB_initparam.interfaceID = EInterfaceID::INF_CAN1;
    chassisMotor_RB_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_3;
    chassisMotor_RB.InitDevice(&chassisMotor_RB_initparam);

    static CDevMtrDM_MIT chassisMotor_L_HIP;
    CDevMtrDM_MIT::SMtrInitParam_DM_MIT chassisMotor_L_HIP_initparam;
    chassisMotor_L_HIP_initparam.deviceID = EDeviceID::DEV_CHAS_L_HIP;
    chassisMotor_L_HIP_initparam.interfaceID = EInterfaceID::INF_CAN1;
    chassisMotor_L_HIP_initparam.MasterID = 0x32; ///< 接收节点
    chassisMotor_L_HIP_initparam.SlaveID = 0x33; ///< 发送节点
    chassisMotor_L_HIP_initparam.Q_MAX = 3.1416;
    chassisMotor_L_HIP_initparam.DQ_MAX = 30.0f;
    chassisMotor_L_HIP_initparam.TAU_MAX = 40.0f;
    chassisMotor_L_HIP_initparam.useAngleToPosit = true;
    chassisMotor_L_HIP_initparam.encoderResolution = 65536; ///< 不牺牲精度
    chassisMotor_L_HIP.InitDevice(&chassisMotor_L_HIP_initparam); ///< 左髋关节电机

    static CDevMtrDM_MIT chassisMotor_R_HIP;
    CDevMtrDM_MIT::SMtrInitParam_DM_MIT chassisMotor_R_HIP_initparam;
    chassisMotor_R_HIP_initparam.deviceID = EDeviceID::DEV_CHAS_R_HIP;
    chassisMotor_R_HIP_initparam.interfaceID = EInterfaceID::INF_CAN1;
    chassisMotor_R_HIP_initparam.MasterID = 0x34; ///< 接收节点
    chassisMotor_R_HIP_initparam.SlaveID = 0x35; ///< 发送节点
    chassisMotor_R_HIP_initparam.Q_MAX = 3.1416;
    chassisMotor_R_HIP_initparam.DQ_MAX = 30.0f;
    chassisMotor_R_HIP_initparam.TAU_MAX = 40.0f;
    chassisMotor_R_HIP_initparam.useAngleToPosit = true;
    chassisMotor_R_HIP_initparam.encoderResolution = 65536; ///< 不牺牲精度
    chassisMotor_R_HIP.InitDevice(&chassisMotor_R_HIP_initparam); ///< 右髋关节电机

    static CDevMtrM3508 chassisMotor_L_CRAWLER;
    CDevMtrM3508::SMtrInitParam_M3508 chassisMotor_L_Crawler_initparam;
    chassisMotor_L_Crawler_initparam.deviceID = EDeviceID::DEV_CHAS_CRAWLER_L;
    chassisMotor_L_Crawler_initparam.interfaceID = EInterfaceID::INF_CAN3;
    chassisMotor_L_Crawler_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_1;
    chassisMotor_L_CRAWLER.InitDevice(&chassisMotor_L_Crawler_initparam);

    static CDevMtrM3508 chassisMotor_R_CRAWLER;
    CDevMtrM3508::SMtrInitParam_M3508 chassisMotor_R_Crawler_initparam;
    chassisMotor_R_Crawler_initparam.deviceID = EDeviceID::DEV_CHAS_CRAWLER_R;
    chassisMotor_R_Crawler_initparam.interfaceID = EInterfaceID::INF_CAN3;
    chassisMotor_R_Crawler_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_2;
    chassisMotor_R_CRAWLER.InitDevice(&chassisMotor_R_Crawler_initparam);

    /******************************************
     * 机械臂电机
     *****************************************/
    static CDevMtrM2006 armMotor_End_L;
    CDevMtrM2006::SMtrInitParam_M2006 armMotor_End_L_initparam;
    armMotor_End_L_initparam.deviceID = EDeviceID::DEV_ARM_MTR_END_L;
    armMotor_End_L_initparam.interfaceID = EInterfaceID::INF_CAN2;
    armMotor_End_L_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_6;
    armMotor_End_L_initparam.useAngleToPosit = true;
    armMotor_End_L_initparam.useStallMonit = true;
    armMotor_End_L_initparam.stallThreshold = 1000.0f; // 设置堵转阈值
    armMotor_End_L_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    armMotor_End_L.InitDevice(&armMotor_End_L_initparam);

    static CDevMtrM2006 armMotor_End_R;
    CDevMtrM2006::SMtrInitParam_M2006 armMotor_End_R_initparam;
    armMotor_End_R_initparam.deviceID = EDeviceID::DEV_ARM_MTR_END_R;
    armMotor_End_R_initparam.interfaceID = EInterfaceID::INF_CAN2;
    armMotor_End_R_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_7;
    armMotor_End_R_initparam.useAngleToPosit = true;
    armMotor_End_R_initparam.useStallMonit = true;
    armMotor_End_R_initparam.stallThreshold = 1000.0f; // 设置堵转阈值
    armMotor_End_R_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE;
    armMotor_End_R.InitDevice(&armMotor_End_R_initparam);

    static CDevMtrM2006 armMotor_Grip;
    CDevMtrM2006::SMtrInitParam_M2006 armMotor_Grip_initparam;
    armMotor_Grip_initparam.deviceID = EDeviceID::DEV_ARM_MTR_GRIP;
    armMotor_Grip_initparam.interfaceID = EInterfaceID::INF_CAN2;
    armMotor_Grip_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_8;
    armMotor_Grip_initparam.useAngleToPosit = true;
    armMotor_Grip_initparam.useStallMonit = true;
    armMotor_Grip_initparam.stallThreshold = 1000.0f; // 设置堵转阈值
    armMotor_Grip_initparam.stallMonitDataSrc = CDevMtr::DATA_TORQUE; //堵转的选项设置为电流
    armMotor_Grip.InitDevice(&armMotor_Grip_initparam);

    static CDevMtrKT armMotor_Yaw;
    CDevMtrKT::SMtrInitParam_KT armMotor_Yaw_initparam;
    armMotor_Yaw_initparam.deviceID = EDeviceID::DEV_ARM_MTR_YAW;
    armMotor_Yaw_initparam.interfaceID = EInterfaceID::INF_CAN3;
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
    armMotor_Pitch1_initparam.interfaceID = EInterfaceID::INF_CAN3;
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
    armMotor_Pitch2_initparam.interfaceID = EInterfaceID::INF_CAN3;
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
    armMotor_Pitch3_initparam.interfaceID = EInterfaceID::INF_CAN3;
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
     * 云台电机
     *****************************************/
    static CDevMtrDM_MIT gimbalMotor_Visualyaw;
    CDevMtrDM_MIT::SMtrInitParam_DM_MIT gimbalMotor_Visualyaw_initparam;
    gimbalMotor_Visualyaw_initparam.deviceID = EDeviceID::DEV_GIMBAL_MTR_VISUAL_YAW;
    gimbalMotor_Visualyaw_initparam.interfaceID = EInterfaceID::INF_CAN2;
    gimbalMotor_Visualyaw_initparam.MasterID = 0x36;
    gimbalMotor_Visualyaw_initparam.SlaveID = 0x37;
    gimbalMotor_Visualyaw_initparam.Q_MAX = 12.5;
    gimbalMotor_Visualyaw_initparam.DQ_MAX = 280.0f;
    gimbalMotor_Visualyaw_initparam.TAU_MAX = 1.0f;
    gimbalMotor_Visualyaw_initparam.useAngleToPosit = true;
    gimbalMotor_Visualyaw.InitDevice(& gimbalMotor_Visualyaw_initparam);

    return APP_OK;
}

} // namespace my_engineer
