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
#include "Device.hpp"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim12;

namespace my_engineer {

/**
 * @brief 配置并初始化所有设备
 * @return APP_OK - 初始化成功
 * @return APP_ERROR - 初始化失败
 */
EAppStatus InitAllDevice(){

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

    /*four button config*/
    static CDevFourButton fourButton;
    CDevFourButton::SDevInitParam_FourButton fourButton_initparam;
    fourButton_initparam.deviceID = EDeviceID::DEV_MULTI_BUTTON;
    fourButton_initparam.buttons_[0].buttonID = CDevFourButton::EButtonID::RESET_BUTTON;
    fourButton_initparam.buttons_[0].activeLevel = 1;
    fourButton_initparam.buttons_[0].halGpioPort = RESET_BUTTON_GPIO_Port;
    fourButton_initparam.buttons_[0].halGpioPin = RESET_BUTTON_Pin;
    fourButton_initparam.buttons_[1].buttonID = CDevFourButton::EButtonID::LEVEL_4_BUTTON;
    fourButton_initparam.buttons_[1].activeLevel = 1;
    fourButton_initparam.buttons_[1].halGpioPort = LEVEL4_BUTTON_GPIO_Port;
    fourButton_initparam.buttons_[1].halGpioPin = LEVEL4_BUTTON_Pin;
    fourButton_initparam.buttons_[2].buttonID = CDevFourButton::EButtonID::LEVEL_3_BUTTON;
    fourButton_initparam.buttons_[2].activeLevel = 1;
    fourButton_initparam.buttons_[2].halGpioPort = LEVEL3_BUTTON_GPIO_Port;
    fourButton_initparam.buttons_[2].halGpioPin = LEVEL3_BUTTON_Pin;
    fourButton_initparam.buttons_[3].buttonID = CDevFourButton::EButtonID::SELF_BUTTON;
    fourButton_initparam.buttons_[3].activeLevel = 1;
    fourButton_initparam.buttons_[3].halGpioPort = SELF_BUTTON_GPIO_Port;
    fourButton_initparam.buttons_[3].halGpioPin = SELF_BUTTON_Pin;
    fourButton.InitDevice(&fourButton_initparam);

    // 摇杆
    static CDevRocker rocker;
    CDevRocker::SDevInitParam_Rocker rocker_initparam;
    rocker_initparam.deviceID = EDeviceID::DEV_ROCKER;
    rocker_initparam.interfaceID = EInterfaceID::INF_ADC1;
    rocker_initparam.X_channel = CInfADC::EAdcChannel::CHANNEL_14;
    rocker_initparam.Y_channel = CInfADC::EAdcChannel::CHANNEL_16;
    // rocker_initparam.halGpioPort = rocker_KEY_GPIO_Port;
    // rocker_initparam.halGpioPin = rocker_KEY_Pin;
    rocker.InitDevice(&rocker_initparam);

    // 蜂鸣器
    static CDevBuzzer buzzer;
    CDevBuzzer::SDevInitParam_Buzzer buzzer_initparam;
    buzzer_initparam.deviceID = EDeviceID::DEV_BUZZER;
    buzzer_initparam.halTimerHandle = &htim12;
    buzzer_initparam.channel = TIM_CHANNEL_2;
    buzzer_initparam.base_frequency = 137500000 / 1375;
    buzzer.InitDevice(&buzzer_initparam);

    // 与机器人通信设备
    static CDevControllerLink controllerLink;
    CDevControllerLink::SDevInitParam_ControllerLink controllerLink_initparam;
    controllerLink_initparam.deviceID = EDeviceID::DEV_CONTROLLER_LINK;
    controllerLink_initparam.interfaceID = EInterfaceID::INF_UART7;
    controllerLink.InitDevice(&controllerLink_initparam);

    // 自定义控制器电机
    // yaw
    static CDevMtrM6020 controllerMotor_Yaw;
    CDevMtrM6020::SMtrInitParam_M6020 controllerMotor_Yaw_initparam;
    controllerMotor_Yaw_initparam.deviceID = EDeviceID::DEV_CONTROLLER_MTR_YAW;
    controllerMotor_Yaw_initparam.interfaceID = EInterfaceID::INF_CAN1;
    controllerMotor_Yaw_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_5;
    controllerMotor_Yaw_initparam.useAngleToPosit = true;
    controllerMotor_Yaw_initparam.useStallMonit = true;
    controllerMotor_Yaw_initparam.stallMonitDataSrc = CDevMtr::DATA_CURRENT;
    controllerMotor_Yaw.InitDevice(&controllerMotor_Yaw_initparam);

    // Pitch1
    static CDevMtrDM controllerMotor_Pitch1;
    CDevMtrDM::SMtrInitParam_DM controllerMotor_Pitch1_initparam;
    controllerMotor_Pitch1_initparam.deviceID = EDeviceID::DEV_CONTROLLER_MTR_PITCH1;
    controllerMotor_Pitch1_initparam.interfaceID = EInterfaceID::INF_CAN2;
    controllerMotor_Pitch1_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    controllerMotor_Pitch1_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    controllerMotor_Pitch1_initparam.useAngleToPosit = false;
    /*--------pleasue confige thie param,only when you use MIT mode--------*/
    controllerMotor_Pitch1_initparam.Kp = 10.0f;                                         ///<Proportional gain for MIT mode
    controllerMotor_Pitch1_initparam.Kd = 2.0f;                                         ///<Derivative gain for
    controllerMotor_Pitch1_initparam.MIT_RxCANID = 0x31;                              
    controllerMotor_Pitch1_initparam.MIT_TxCANID = 0x30;
    /*--------------------------------------------------------------------*/
    controllerMotor_Pitch1.InitDevice(&controllerMotor_Pitch1_initparam);

    // Pitch2
    static CDevMtrDM controllerMotor_Pitch2;
    CDevMtrDM::SMtrInitParam_DM controllerMotor_Pitch2_initparam;
    controllerMotor_Pitch2_initparam.deviceID = EDeviceID::DEV_CONTROLLER_MTR_PITCH2;
    controllerMotor_Pitch2_initparam.interfaceID = EInterfaceID::INF_CAN2;
    controllerMotor_Pitch2_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    controllerMotor_Pitch2_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    controllerMotor_Pitch2_initparam.useAngleToPosit = false;
    /*--------pleasue confige thie param,only when you use MIT mode--------*/
    controllerMotor_Pitch2_initparam.Kp = 10.0f;                                         ///<Proportional gain for MIT mode
    controllerMotor_Pitch2_initparam.Kd = 2.0f;                                         ///<Derivative gain for
    controllerMotor_Pitch2_initparam.MIT_RxCANID = 0x33;                              
    controllerMotor_Pitch2_initparam.MIT_TxCANID = 0x32;
    /*--------------------------------------------------------------------*/    
    controllerMotor_Pitch2.InitDevice(&controllerMotor_Pitch2_initparam);

    // Roll
    static CDevMtrM3508 controllerMotor_Roll;
    CDevMtrM3508::SMtrInitParam_M3508 controllerMotor_Roll_initparam;
    controllerMotor_Roll_initparam.deviceID = EDeviceID::DEV_CONTROLLER_MTR_ROLL;
    controllerMotor_Roll_initparam.interfaceID = EInterfaceID::INF_CAN1;
    controllerMotor_Roll_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_1;
    controllerMotor_Roll_initparam.useAngleToPosit = true;
    controllerMotor_Roll_initparam.useStallMonit = true;
    controllerMotor_Roll_initparam.stallMonitDataSrc = CDevMtr::DATA_CURRENT;
    controllerMotor_Roll_initparam.stallThreshold = 2000;
    controllerMotor_Roll.InitDevice(&controllerMotor_Roll_initparam);

    // pitch end
    static CDevMtrM3508 controllerMotor_PitchEnd;
    CDevMtrM3508::SMtrInitParam_M3508 controllerMotor_PitchEnd_initparam;
    controllerMotor_PitchEnd_initparam.deviceID = EDeviceID::DEV_CONTROLLER_MTR_PITCH_END;
    controllerMotor_PitchEnd_initparam.interfaceID = EInterfaceID::INF_CAN1;
    controllerMotor_PitchEnd_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_2;
    controllerMotor_PitchEnd_initparam.useAngleToPosit = true;
    controllerMotor_PitchEnd_initparam.useStallMonit = true;
    controllerMotor_PitchEnd_initparam.stallMonitDataSrc = CDevMtr::DATA_CURRENT;
    controllerMotor_PitchEnd_initparam.stallThreshold = 2000;
    controllerMotor_PitchEnd.InitDevice(&controllerMotor_PitchEnd_initparam);


    return APP_OK;
}

} // namespace my_engineer
