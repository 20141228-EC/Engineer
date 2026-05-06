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
    * 底盘电机
    ******************************************/
    static CDevMtrM3508 chassisMotor_LF;
    CDevMtrM3508::SMtrInitParam_M3508 chassisMotor_LF_initparam;
    chassisMotor_LF_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_LF;
    chassisMotor_LF_initparam.interfaceID = EInterfaceID::INF_CAN2;
    chassisMotor_LF_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_1;
    chassisMotor_LF.InitDevice(&chassisMotor_LF_initparam);

    static CDevMtrM3508 chassisMotor_RF;
    CDevMtrM3508::SMtrInitParam_M3508 chassisMotor_RF_initparam;
    chassisMotor_RF_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_RF;
    chassisMotor_RF_initparam.interfaceID = EInterfaceID::INF_CAN2;
    chassisMotor_RF_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_2;
    chassisMotor_RF.InitDevice(&chassisMotor_RF_initparam);

    static CDevMtrM3508 chassisMotor_LB;
    CDevMtrM3508::SMtrInitParam_M3508 chassisMotor_LB_initparam;
    chassisMotor_LB_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_LB;
    chassisMotor_LB_initparam.interfaceID = EInterfaceID::INF_CAN2;
    chassisMotor_LB_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_3;
    chassisMotor_LB.InitDevice(&chassisMotor_LB_initparam);

    static CDevMtrM3508 chassisMotor_RB;
    CDevMtrM3508::SMtrInitParam_M3508 chassisMotor_RB_initparam;
    chassisMotor_RB_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_RB;
    chassisMotor_RB_initparam.interfaceID = EInterfaceID::INF_CAN2;
    chassisMotor_RB_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_4;
    chassisMotor_RB.InitDevice(&chassisMotor_RB_initparam);

    static CDevMtrRM6020 chassisSteerMotor_LF;
    CDevMtrRM6020::SMtrInitParam_RM6020 chassisSteerMotor_LF_initparam;
    chassisSteerMotor_LF_initparam.deviceID = EDeviceID::DEV_CHAS_STEER_LF;
    chassisSteerMotor_LF_initparam.interfaceID = EInterfaceID::INF_CAN1;
    chassisSteerMotor_LF_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_5;
    chassisSteerMotor_LF_initparam.useAngleToPosit = true;
    chassisSteerMotor_LF.InitDevice(&chassisSteerMotor_LF_initparam);

    static CDevMtrRM6020 chassisSteerMotor_RF;
    CDevMtrRM6020::SMtrInitParam_RM6020 chassisSteerMotor_RF_initparam;
    chassisSteerMotor_RF_initparam.deviceID = EDeviceID::DEV_CHAS_STEER_RF;
    chassisSteerMotor_RF_initparam.interfaceID = EInterfaceID::INF_CAN1;
    chassisSteerMotor_RF_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_6;
    chassisSteerMotor_RF_initparam.useAngleToPosit = true;
    chassisSteerMotor_RF.InitDevice(&chassisSteerMotor_RF_initparam);

    static CDevMtrRM6020 chassisSteerMotor_LB;
    CDevMtrRM6020::SMtrInitParam_RM6020 chassisSteerMotor_LB_initparam;
    chassisSteerMotor_LB_initparam.deviceID = EDeviceID::DEV_CHAS_STEER_LB;
    chassisSteerMotor_LB_initparam.interfaceID = EInterfaceID::INF_CAN1;
    chassisSteerMotor_LB_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_7;
    chassisSteerMotor_LB_initparam.useAngleToPosit = true;
    chassisSteerMotor_LB.InitDevice(&chassisSteerMotor_LB_initparam);

    static CDevMtrRM6020 chassisSteerMotor_RB;
    CDevMtrRM6020::SMtrInitParam_RM6020 chassisSteerMotor_RB_initparam;
    chassisSteerMotor_RB_initparam.deviceID = EDeviceID::DEV_CHAS_STEER_RB;
    chassisSteerMotor_RB_initparam.interfaceID = EInterfaceID::INF_CAN1;
    chassisSteerMotor_RB_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_8;
    chassisSteerMotor_RB_initparam.useAngleToPosit = true;
    chassisSteerMotor_RB.InitDevice(&chassisSteerMotor_RB_initparam);

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

    return APP_OK;
}

} // namespace my_engineer
