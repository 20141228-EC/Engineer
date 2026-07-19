/**
 * @file conf_device.cpp
 * @author Zoe, Ciallo
 * @brief 完成所有设备的配置
 * @version 1.1
 * @date 2024-11-01
 * @LastEditors Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-15
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

    /*four button config 按钮配置（连续索引，下拉输入高电平有效）*/
    static CDevButton Button;
    CDevButton::SDevInitParam_Button Button_initparam;
    Button_initparam.deviceID = EDeviceID::DEV_MULTI_BUTTON;
    // 按键1
    Button_initparam.buttons_[0].buttonID = CDevButton::EButtonID::LEVEL_1;
    Button_initparam.buttons_[0].activeLevel = 1;  // 高电平有效
    Button_initparam.buttons_[0].halGpioPort = LEVEL_1_GPIO_Port;
    Button_initparam.buttons_[0].halGpioPin = LEVEL_1_Pin;
    // 按键2
    Button_initparam.buttons_[1].buttonID = CDevButton::EButtonID::LEVEL_2;
    Button_initparam.buttons_[1].activeLevel = 1;  // 高电平有效
    Button_initparam.buttons_[1].halGpioPort = LEVEL_2_GPIO_Port;
    Button_initparam.buttons_[1].halGpioPin = LEVEL_2_Pin;
    // 按键3
    Button_initparam.buttons_[2].buttonID = CDevButton::EButtonID::LEVEL_3;
    Button_initparam.buttons_[2].activeLevel = 1;  // 高电平有效
    Button_initparam.buttons_[2].halGpioPort = LEVEL_4_GPIO_Port;
    Button_initparam.buttons_[2].halGpioPin = LEVEL_4_Pin;
    // 复位按键
    Button_initparam.buttons_[3].buttonID = CDevButton::EButtonID::RESET;
    Button_initparam.buttons_[3].activeLevel = 1;  // 高电平有效
    Button_initparam.buttons_[3].halGpioPort = RESET_GPIO_Port;
    Button_initparam.buttons_[3].halGpioPin = RESET_Pin;
    // 末端 roll 翻转
    Button_initparam.buttons_[4].buttonID = CDevButton::EButtonID::END_ROLL_TOGGLE;
    Button_initparam.buttons_[4].activeLevel = 1;
    Button_initparam.buttons_[4].halGpioPort = LEVEL_3_GPIO_Port;
    Button_initparam.buttons_[4].halGpioPin = LEVEL_3_Pin;
    Button.InitDevice(&Button_initparam);

    // 摇杆
    static CDevRocker rocker;
    CDevRocker::SDevInitParam_Rocker rocker_initparam;
    rocker_initparam.deviceID = EDeviceID::DEV_ROCKER;
    rocker_initparam.interfaceID = EInterfaceID::INF_ADC1;
    rocker_initparam.X_channel = CInfADC::EAdcChannel::CHANNEL_NULL;  // PA2 未使用（原 LEVEL_4 已移除）
    rocker_initparam.Y_channel = CInfADC::EAdcChannel::CHANNEL_NULL;  // PA5
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

    /*----------- 单臂电机 -----------*/
    // Yaw (M6020, CAN1 ID5)
    static CDevMtrM6020 mtr_Yaw;
    CDevMtrM6020::SMtrInitParam_M6020 mtr_Yaw_initparam;
    mtr_Yaw_initparam.deviceID = EDeviceID::DEV_MTR_YAW;
    mtr_Yaw_initparam.interfaceID = EInterfaceID::INF_CAN1;
    mtr_Yaw_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_5;
    mtr_Yaw_initparam.useAngleToPosit = true;
    mtr_Yaw_initparam.useStallMonit = true;
    mtr_Yaw_initparam.stallMonitDataSrc = CDevMtr::DATA_CURRENT;
    mtr_Yaw.InitDevice(&mtr_Yaw_initparam);

    // Pitch1 (DM4310, CAN3, CAN_ID=0x31, Master_ID=0x30)
    static CDevMtrDM mtr_Pitch1;
    CDevMtrDM::SMtrInitParam_DM mtr_Pitch1_initparam;
    mtr_Pitch1_initparam.deviceID = EDeviceID::DEV_MTR_PITCH1;
    mtr_Pitch1_initparam.interfaceID = EInterfaceID::INF_CAN2;
    mtr_Pitch1_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_Pitch1_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_Pitch1_initparam.useAngleToPosit = false;
    mtr_Pitch1_initparam.Kp = 15.0f;
    mtr_Pitch1_initparam.Kd = 2.0f;
    mtr_Pitch1_initparam.Q_MAX = 3.1416f;
    mtr_Pitch1_initparam.MIT_TxCANID = 0x31;  // 发送到电机的CAN_ID
    mtr_Pitch1_initparam.MIT_RxCANID = 0x30;  // 接收电机反馈的Master_ID
    mtr_Pitch1.InitDevice(&mtr_Pitch1_initparam);

    // Pitch2 (DM4310, CAN3, CAN_ID=0x33, Master_ID=0x32)
    static CDevMtrDM mtr_Pitch2;
    CDevMtrDM::SMtrInitParam_DM mtr_Pitch2_initparam;
    mtr_Pitch2_initparam.deviceID = EDeviceID::DEV_MTR_PITCH2;
    mtr_Pitch2_initparam.interfaceID = EInterfaceID::INF_CAN2;
    mtr_Pitch2_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_Pitch2_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_Pitch2_initparam.useAngleToPosit = false;
    mtr_Pitch2_initparam.Kp = 15.0f;
    mtr_Pitch2_initparam.Kd = 2.0f;
    mtr_Pitch2_initparam.Q_MAX = 3.1416f;
    mtr_Pitch2_initparam.MIT_TxCANID = 0x33;  // 发送到电机的CAN_ID
    mtr_Pitch2_initparam.MIT_RxCANID = 0x32;  // 接收电机反馈的Master_ID
    mtr_Pitch2.InitDevice(&mtr_Pitch2_initparam);

    // Roll (DM3510, CAN2, CAN_ID=0x35, Master_ID=0x34)
    static CDevMtrDM mtr_Roll;
    CDevMtrDM::SMtrInitParam_DM mtr_Roll_initparam;
    mtr_Roll_initparam.deviceID = EDeviceID::DEV_MTR_ROLL;
    mtr_Roll_initparam.interfaceID = EInterfaceID::INF_CAN3; 
    mtr_Roll_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_Roll_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_Roll_initparam.useAngleToPosit = false;
    mtr_Roll_initparam.Kp = 0.123f;
    mtr_Roll_initparam.Kd = 0.015f;
    mtr_Roll_initparam.MIT_TxCANID = 0x35;  // 发送到电机的CAN_ID
    mtr_Roll_initparam.MIT_RxCANID = 0x34;  // 接收电机反馈的Master_ID
    mtr_Roll_initparam.TAU_MAX = 1.0f;      // DM3510 力矩范围 ±1 N·m
    mtr_Roll_initparam.DQ_MAX = 280.0f;     // DM3510 速度范围 ±280 rad/s
    mtr_Roll.InitDevice(&mtr_Roll_initparam);

    // PitchEnd (DM3510, CAN2, CAN_ID=0x37, Master_ID=0x36)
    static CDevMtrDM mtr_PitchEnd;
    CDevMtrDM::SMtrInitParam_DM mtr_PitchEnd_initparam;
    mtr_PitchEnd_initparam.deviceID = EDeviceID::DEV_MTR_PITCH_END;
    mtr_PitchEnd_initparam.interfaceID = EInterfaceID::INF_CAN3; 
    mtr_PitchEnd_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_PitchEnd_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_PitchEnd_initparam.useAngleToPosit = false;
    mtr_PitchEnd_initparam.Kp = 0.123f;
    mtr_PitchEnd_initparam.Kd = 0.015f;
    mtr_PitchEnd_initparam.MIT_TxCANID = 0x37;  // 发送到电机的CAN_ID
    mtr_PitchEnd_initparam.MIT_RxCANID = 0x36;  // 接收电机反馈的Master_ID
    mtr_PitchEnd_initparam.TAU_MAX = 1.0f;      // DM3510 力矩范围 ±1 N·m
    mtr_PitchEnd_initparam.DQ_MAX = 280.0f;     // DM3510 速度范围 ±280 rad/s
    mtr_PitchEnd.InitDevice(&mtr_PitchEnd_initparam);

    return APP_OK;
}

} // namespace my_engineer
