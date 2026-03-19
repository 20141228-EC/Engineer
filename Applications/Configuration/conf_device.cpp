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

    /*four button config - 4按钮配置（连续索引，下拉输入高电平有效）*/
    static CDevFourButton fourButton;
    CDevFourButton::SDevInitParam_FourButton fourButton_initparam;
    fourButton_initparam.deviceID = EDeviceID::DEV_MULTI_BUTTON;
    // 槽位0: 拨杆右档 - 底盘模式 (PE13)
    fourButton_initparam.buttons_[0].buttonID = CDevFourButton::EButtonID::SWITCH_CHASSIS;
    fourButton_initparam.buttons_[0].activeLevel = 1;  // 高电平有效（下拉输入）
    fourButton_initparam.buttons_[0].halGpioPort = SWITCH_CHASSIS_GPIO_Port;
    fourButton_initparam.buttons_[0].halGpioPin = SWITCH_CHASSIS_Pin;
    // 槽位1: 拨杆左档 - 臂Roll末端模式 (PE9)
    fourButton_initparam.buttons_[1].buttonID = CDevFourButton::EButtonID::SWITCH_ARM_ROLL_END;
    fourButton_initparam.buttons_[1].activeLevel = 1;  // 高电平有效（下拉输入）
    fourButton_initparam.buttons_[1].halGpioPort = SWITCH_ARM_ROLL_END_GPIO_Port;
    fourButton_initparam.buttons_[1].halGpioPin = SWITCH_ARM_ROLL_END_Pin;
    // 槽位2: 左手夹爪 (PB8)
    fourButton_initparam.buttons_[2].buttonID = CDevFourButton::EButtonID::GRIPPER_LEFT;
    fourButton_initparam.buttons_[2].activeLevel = 0;  // 低电平有效（上拉输入，按下接GND）
    fourButton_initparam.buttons_[2].halGpioPort = GRIPPER_LEFT_GPIO_Port;
    fourButton_initparam.buttons_[2].halGpioPin = GRIPPER_LEFT_Pin;
    // 槽位3: 右手夹爪 (PB9)
    fourButton_initparam.buttons_[3].buttonID = CDevFourButton::EButtonID::GRIPPER_RIGHT;
    fourButton_initparam.buttons_[3].activeLevel = 0;  // 低电平有效（上拉输入，按下接GND）
    fourButton_initparam.buttons_[3].halGpioPort = GRIPPER_RIGHT_GPIO_Port;
    fourButton_initparam.buttons_[3].halGpioPin = GRIPPER_RIGHT_Pin;
    fourButton.InitDevice(&fourButton_initparam);

    // 左臂摇杆（单轴模式）- PA0 = CHANNEL_16
    static CDevRocker rocker_left;
    CDevRocker::SDevInitParam_Rocker rocker_left_initparam;
    rocker_left_initparam.deviceID = EDeviceID::DEV_ROCKER_LEFT;
    rocker_left_initparam.interfaceID = EInterfaceID::INF_ADC1;
    rocker_left_initparam.X_channel = CInfADC::EAdcChannel::CHANNEL_16;  // PA0
    rocker_left_initparam.Y_channel = CInfADC::EAdcChannel::CHANNEL_NULL; // 不使用Y轴
    rocker_left.InitDevice(&rocker_left_initparam);

    // 右臂摇杆（双轴模式）- PA2 = CHANNEL_14, PA5 = CHANNEL_19
    static CDevRocker rocker_right;
    CDevRocker::SDevInitParam_Rocker rocker_right_initparam;
    rocker_right_initparam.deviceID = EDeviceID::DEV_ROCKER_RIGHT;
    rocker_right_initparam.interfaceID = EInterfaceID::INF_ADC1;
    rocker_right_initparam.X_channel = CInfADC::EAdcChannel::CHANNEL_14;  // PA2
    rocker_right_initparam.Y_channel = CInfADC::EAdcChannel::CHANNEL_19;  // PA5
    rocker_right.InitDevice(&rocker_right_initparam);

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

    /*----------- 左臂电机 -----------*/
    // 左臂Yaw (M6020, CAN1 ID6)
    static CDevMtrM6020 mtr_Yaw_L;
    CDevMtrM6020::SMtrInitParam_M6020 mtr_Yaw_L_initparam;
    mtr_Yaw_L_initparam.deviceID = EDeviceID::DEV_MTR_YAW_L;
    mtr_Yaw_L_initparam.interfaceID = EInterfaceID::INF_CAN1;
    mtr_Yaw_L_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_6;
    mtr_Yaw_L_initparam.useAngleToPosit = true;
    mtr_Yaw_L_initparam.useStallMonit = true;
    mtr_Yaw_L_initparam.stallMonitDataSrc = CDevMtr::DATA_CURRENT;
    mtr_Yaw_L.InitDevice(&mtr_Yaw_L_initparam);

    // 左臂Pitch1 (DM4310, CAN2, CAN_ID=0x31, Master_ID=0x30)
    static CDevMtrDM mtr_Pitch1_L;
    CDevMtrDM::SMtrInitParam_DM mtr_Pitch1_L_initparam;
    mtr_Pitch1_L_initparam.deviceID = EDeviceID::DEV_MTR_PITCH1_L;
    mtr_Pitch1_L_initparam.interfaceID = EInterfaceID::INF_CAN2;
    mtr_Pitch1_L_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_Pitch1_L_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_Pitch1_L_initparam.useAngleToPosit = false;
    mtr_Pitch1_L_initparam.Kp = 10.0f;
    mtr_Pitch1_L_initparam.Kd = 2.0f;
    mtr_Pitch1_L_initparam.MIT_TxCANID = 0x31;  // 发送到电机的CAN_ID
    mtr_Pitch1_L_initparam.MIT_RxCANID = 0x30;  // 接收电机反馈的Master_ID
    mtr_Pitch1_L.InitDevice(&mtr_Pitch1_L_initparam);

    // 左臂Pitch2 (DM4310, CAN2, CAN_ID=0x33, Master_ID=0x32)
    static CDevMtrDM mtr_Pitch2_L;
    CDevMtrDM::SMtrInitParam_DM mtr_Pitch2_L_initparam;
    mtr_Pitch2_L_initparam.deviceID = EDeviceID::DEV_MTR_PITCH2_L;
    mtr_Pitch2_L_initparam.interfaceID = EInterfaceID::INF_CAN2;
    mtr_Pitch2_L_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_Pitch2_L_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_Pitch2_L_initparam.useAngleToPosit = false;
    mtr_Pitch2_L_initparam.Kp = 10.0f;
    mtr_Pitch2_L_initparam.Kd = 2.0f;
    mtr_Pitch2_L_initparam.MIT_TxCANID = 0x33;  // 发送到电机的CAN_ID
    mtr_Pitch2_L_initparam.MIT_RxCANID = 0x32;  // 接收电机反馈的Master_ID
    mtr_Pitch2_L.InitDevice(&mtr_Pitch2_L_initparam);

    // 左臂Roll (DM3510, CAN2, CAN_ID=0x35, Master_ID=0x34)
    static CDevMtrDM mtr_Roll_L;
    CDevMtrDM::SMtrInitParam_DM mtr_Roll_L_initparam;
    mtr_Roll_L_initparam.deviceID = EDeviceID::DEV_MTR_ROLL_L;
    mtr_Roll_L_initparam.interfaceID = EInterfaceID::INF_CAN2;
    mtr_Roll_L_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_Roll_L_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_Roll_L_initparam.useAngleToPosit = false;
    mtr_Roll_L_initparam.Kp = 0.123f;
    mtr_Roll_L_initparam.Kd = 0.0074f;
    mtr_Roll_L_initparam.MIT_TxCANID = 0x35;  // 发送到电机的CAN_ID
    mtr_Roll_L_initparam.MIT_RxCANID = 0x34;  // 接收电机反馈的Master_ID
    mtr_Roll_L_initparam.TAU_MAX = 1.0f;      // DM3510 力矩范围 ±1 N·m
    mtr_Roll_L_initparam.DQ_MAX = 280.0f;     // DM3510 速度范围 ±280 rad/s
    mtr_Roll_L.InitDevice(&mtr_Roll_L_initparam);

    // 左臂PitchEnd (DM3510, CAN2, CAN_ID=0x37, Master_ID=0x36)
    static CDevMtrDM mtr_PitchEnd_L;
    CDevMtrDM::SMtrInitParam_DM mtr_PitchEnd_L_initparam;
    mtr_PitchEnd_L_initparam.deviceID = EDeviceID::DEV_MTR_PITCH_END_L;
    mtr_PitchEnd_L_initparam.interfaceID = EInterfaceID::INF_CAN2;
    mtr_PitchEnd_L_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_PitchEnd_L_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_PitchEnd_L_initparam.useAngleToPosit = false;
    mtr_PitchEnd_L_initparam.Kp = 0.123f;
    mtr_PitchEnd_L_initparam.Kd = 0.0074f;
    mtr_PitchEnd_L_initparam.MIT_TxCANID = 0x37;  // 发送到电机的CAN_ID
    mtr_PitchEnd_L_initparam.MIT_RxCANID = 0x36;  // 接收电机反馈的Master_ID
    mtr_PitchEnd_L_initparam.TAU_MAX = 1.0f;      // DM3510 力矩范围 ±1 N·m
    mtr_PitchEnd_L_initparam.DQ_MAX = 280.0f;     // DM3510 速度范围 ±280 rad/s
    mtr_PitchEnd_L.InitDevice(&mtr_PitchEnd_L_initparam);

    /*----------- 右臂电机 -----------*/
    // 右臂Yaw (M6020, CAN1 ID5)
    static CDevMtrM6020 mtr_Yaw_R;
    CDevMtrM6020::SMtrInitParam_M6020 mtr_Yaw_R_initparam;
    mtr_Yaw_R_initparam.deviceID = EDeviceID::DEV_MTR_YAW_R;
    mtr_Yaw_R_initparam.interfaceID = EInterfaceID::INF_CAN1;
    mtr_Yaw_R_initparam.djiMtrID = CDevMtrDJI::EDjiMtrID::ID_5;
    mtr_Yaw_R_initparam.useAngleToPosit = true;
    mtr_Yaw_R_initparam.useStallMonit = true;
    mtr_Yaw_R_initparam.stallMonitDataSrc = CDevMtr::DATA_CURRENT;
    mtr_Yaw_R.InitDevice(&mtr_Yaw_R_initparam);

    // 右臂Pitch1 (DM4310, CAN3, CAN_ID=0x31, Master_ID=0x30)
    static CDevMtrDM mtr_Pitch1_R;
    CDevMtrDM::SMtrInitParam_DM mtr_Pitch1_R_initparam;
    mtr_Pitch1_R_initparam.deviceID = EDeviceID::DEV_MTR_PITCH1_R;
    mtr_Pitch1_R_initparam.interfaceID = EInterfaceID::INF_CAN3;
    mtr_Pitch1_R_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_Pitch1_R_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_Pitch1_R_initparam.useAngleToPosit = false;
    mtr_Pitch1_R_initparam.Kp = 10.0f;
    mtr_Pitch1_R_initparam.Kd = 2.0f;
    mtr_Pitch1_R_initparam.MIT_TxCANID = 0x31;  // 发送到电机的CAN_ID
    mtr_Pitch1_R_initparam.MIT_RxCANID = 0x30;  // 接收电机反馈的Master_ID
    mtr_Pitch1_R.InitDevice(&mtr_Pitch1_R_initparam);

    // 右臂Pitch2 (DM4310, CAN3, CAN_ID=0x33, Master_ID=0x32)
    static CDevMtrDM mtr_Pitch2_R;
    CDevMtrDM::SMtrInitParam_DM mtr_Pitch2_R_initparam;
    mtr_Pitch2_R_initparam.deviceID = EDeviceID::DEV_MTR_PITCH2_R;
    mtr_Pitch2_R_initparam.interfaceID = EInterfaceID::INF_CAN3;
    mtr_Pitch2_R_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_Pitch2_R_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_Pitch2_R_initparam.useAngleToPosit = false;
    mtr_Pitch2_R_initparam.Kp = 10.0f;
    mtr_Pitch2_R_initparam.Kd = 2.0f;
    mtr_Pitch2_R_initparam.MIT_TxCANID = 0x33;  // 发送到电机的CAN_ID
    mtr_Pitch2_R_initparam.MIT_RxCANID = 0x32;  // 接收电机反馈的Master_ID
    mtr_Pitch2_R.InitDevice(&mtr_Pitch2_R_initparam);

    // 右臂Roll (DM3510, CAN3, CAN_ID=0x35, Master_ID=0x34)
    static CDevMtrDM mtr_Roll_R;
    CDevMtrDM::SMtrInitParam_DM mtr_Roll_R_initparam;
    mtr_Roll_R_initparam.deviceID = EDeviceID::DEV_MTR_ROLL_R;
    mtr_Roll_R_initparam.interfaceID = EInterfaceID::INF_CAN3;
    mtr_Roll_R_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_Roll_R_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_Roll_R_initparam.useAngleToPosit = false;
    mtr_Roll_R_initparam.Kp = 0.123f;
    mtr_Roll_R_initparam.Kd = 0.015f;
    mtr_Roll_R_initparam.MIT_TxCANID = 0x35;  // 发送到电机的CAN_ID
    mtr_Roll_R_initparam.MIT_RxCANID = 0x34;  // 接收电机反馈的Master_ID
    mtr_Roll_R_initparam.TAU_MAX = 1.0f;      // DM3510 力矩范围 ±1 N·m
    mtr_Roll_R_initparam.DQ_MAX = 280.0f;     // DM3510 速度范围 ±280 rad/s
    mtr_Roll_R.InitDevice(&mtr_Roll_R_initparam);

    // 右臂PitchEnd (DM3510, CAN3, CAN_ID=0x37, Master_ID=0x36)
    static CDevMtrDM mtr_PitchEnd_R;
    CDevMtrDM::SMtrInitParam_DM mtr_PitchEnd_R_initparam;
    mtr_PitchEnd_R_initparam.deviceID = EDeviceID::DEV_MTR_PITCH_END_R;
    mtr_PitchEnd_R_initparam.interfaceID = EInterfaceID::INF_CAN3;
    mtr_PitchEnd_R_initparam.dmMtrID = CDevMtrDM::EDmMtrID::ID_MIT;
    mtr_PitchEnd_R_initparam.dmMtrMode = CDevMtrDM::EMotorControlMode::MODE_MIT;
    mtr_PitchEnd_R_initparam.useAngleToPosit = false;
    mtr_PitchEnd_R_initparam.Kp = 0.123f;//0.123
    mtr_PitchEnd_R_initparam.Kd = 0.015f;
    mtr_PitchEnd_R_initparam.MIT_TxCANID = 0x37;  // 发送到电机的CAN_ID
    mtr_PitchEnd_R_initparam.MIT_RxCANID = 0x36;  // 接收电机反馈的Master_ID
    mtr_PitchEnd_R_initparam.TAU_MAX = 1.0f;      // DM3510 力矩范围 ±1 N·m
    mtr_PitchEnd_R_initparam.DQ_MAX = 280.0f;     // DM3510 速度范围 ±280 rad/s
    mtr_PitchEnd_R.InitDevice(&mtr_PitchEnd_R_initparam);

    return APP_OK;
}

} // namespace my_engineer
