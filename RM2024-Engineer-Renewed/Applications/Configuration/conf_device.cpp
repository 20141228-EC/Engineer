/**
 * @file        conf_device.cpp
 * @version     1.0
 * @date        2024-03-15
 * @author      Morthine Xiang
 * @email       xiang@morthine.com
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-15   <td>1.0         <td>Morthine Xiang  <td>First Create.
 * </table>
 */

#include "conf_device.hpp"
#include "Device.hpp"

extern TIM_HandleTypeDef htim3;

namespace robotpilots {

/**
 * @brief Initialize Device
 *
 * @return RP_OK - Success
 * @return RP_ERROR - Error
 */
ERpStatus InitializeDevice() {

  static CRc_DR16 dr16Remote;
  CRc_DR16::SRcDR16InitParam dr16RemoteInitParam;
  dr16RemoteInitParam.deviceID = EDeviceID::DEV_RC_DR16;
  dr16RemoteInitParam.interfaceId = EInterfaceID::INF_DBUS;
  dr16Remote.InitDevice(&dr16RemoteInitParam);

  static CMems_BMI088 bmi088Mems;
  CMems_BMI088::SBMI088InitParam bmi088MemsInitParam;
  bmi088MemsInitParam.deviceID = EDeviceID::DEV_MEMS_BMI088;
  bmi088MemsInitParam.interfaceId = EInterfaceID::INF_SPI2;
  bmi088MemsInitParam.AccelUnitCsPort = BMI_ACC_CS_GPIO_Port;
  bmi088MemsInitParam.AccelUnitCsPin = BMI_ACC_CS_Pin;
  bmi088MemsInitParam.GyroUnitCsPort = BMI_GYRO_CS_GPIO_Port;
  bmi088MemsInitParam.GyroUnitCsPin = BMI_GYRO_CS_Pin;
  bmi088MemsInitParam.useTempControl = false;
  bmi088MemsInitParam.tempTarget = 40.0f;
  bmi088MemsInitParam.halTimHandle = &htim3;
  bmi088MemsInitParam.halTimChannel = TIM_CHANNEL_4;
  bmi088MemsInitParam.tempPidParam.kp = 10.0f;
  bmi088MemsInitParam.tempPidParam.ki = 3.0f;
  bmi088MemsInitParam.tempPidParam.kd = 0.5f;
  bmi088MemsInitParam.tempPidParam.maxInteger = 20;
  bmi088MemsInitParam.tempPidParam.maxOutput = 100;
  bmi088Mems.InitDevice(&bmi088MemsInitParam);

  /***************************************************
   * Referee Device Initialize
   ***************************************************/
  static CDevReferee rmReferee;
  CDevReferee::SDevRefereeInitParam rmRefereeInitParam;
  rmRefereeInitParam.deviceID = EDeviceID::DEV_RM_REFEREE;
  rmRefereeInitParam.interfaceId = EInterfaceID::INF_UART1;
  rmReferee.InitDevice(&rmRefereeInitParam);

  /***************************************************
   * Vision Device Initialize
   ***************************************************/
  static CDevVision rpVision;
  CDevVision::SDevVisionInitParam rpVisionInitParam;
  rpVisionInitParam.deviceID = EDeviceID::DEV_RP_VISION;
  rpVisionInitParam.interfaceId = EInterfaceID::INF_UART7;
  rpVision.InitDevice(&rpVisionInitParam);

  /***************************************************
   * Chassis Motor Initialize
   ***************************************************/
  static CMtr_M3508 chassisMotor_LF;
  CMtr_M3508::SMtrM3508InitParam chassisMotorInitParam_LF;
  chassisMotorInitParam_LF.deviceID = EDeviceID::DEV_CHAS_MTR_LF;
  chassisMotorInitParam_LF.interfaceId = EInterfaceID::INF_CAN3;
  chassisMotorInitParam_LF.djiMtrID = CMtrDji::EDjiMtrID::ID_2;
  chassisMotor_LF.InitDevice(&chassisMotorInitParam_LF);

  static CMtr_M3508 chassisMotor_RF;
  CMtr_M3508::SMtrM3508InitParam chassisMotorInitParam_RF;
  chassisMotorInitParam_RF.deviceID = EDeviceID::DEV_CHAS_MTR_RF;
  chassisMotorInitParam_RF.interfaceId = EInterfaceID::INF_CAN3;
  chassisMotorInitParam_RF.djiMtrID = CMtrDji::EDjiMtrID::ID_1;
  chassisMotor_RF.InitDevice(&chassisMotorInitParam_RF);

  static CMtr_M3508 chassis_mtr_lb;
  CMtr_M3508::SMtrM3508InitParam chassis_mtr_lb_initparam;
  chassis_mtr_lb_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_LB;
  chassis_mtr_lb_initparam.interfaceId = EInterfaceID::INF_CAN3;
  chassis_mtr_lb_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_3;
  // chassis_mtr_lf_initparam.useAngleToPosit = true;
  chassis_mtr_lb.InitDevice(&chassis_mtr_lb_initparam);

  static CMtr_M3508 chassis_mtr_rb;
  CMtr_M3508::SMtrM3508InitParam chassis_mtr_rb_initparam;
  chassis_mtr_rb_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_RB;
  chassis_mtr_rb_initparam.interfaceId = EInterfaceID::INF_CAN3;
  chassis_mtr_rb_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_4;
  // chassis_mtr_lf_initparam.useAngleToPosit = true;
  chassis_mtr_rb.InitDevice(&chassis_mtr_rb_initparam);


  /***************************************************
   * Gimbal Motor Initialize
   ***************************************************/
  static CMtr_M2006 gimbal_mtr_l;
  CMtr_M2006::SMtrM2006InitParam gimbal_mtr_l_initparam;
  gimbal_mtr_l_initparam.deviceID = EDeviceID::DEV_GIMB_MTR_L;
  gimbal_mtr_l_initparam.interfaceId = EInterfaceID::INF_CAN3;
  gimbal_mtr_l_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_5;
  gimbal_mtr_l_initparam.useAngleToPosit = true;
  gimbal_mtr_l_initparam.useStallMonit = true;
  gimbal_mtr_l_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  gimbal_mtr_l.InitDevice(&gimbal_mtr_l_initparam);

  static CMtr_M2006 gimbal_mtr_r;
  CMtr_M2006::SMtrM2006InitParam gimbal_mtr_r_initparam;
  gimbal_mtr_r_initparam.deviceID = EDeviceID::DEV_GIMB_MTR_R;
  gimbal_mtr_r_initparam.interfaceId = EInterfaceID::INF_CAN3;
  gimbal_mtr_r_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_6;
  gimbal_mtr_r_initparam.useAngleToPosit = true;
  gimbal_mtr_r_initparam.useStallMonit = true;
  gimbal_mtr_r_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  gimbal_mtr_r.InitDevice(&gimbal_mtr_r_initparam);

  static CMtr_M2006 gimbal_mtr_pitch;
  CMtr_M2006::SMtrM2006InitParam gimbal_mtr_pitch_initparam;
  gimbal_mtr_pitch_initparam.deviceID = EDeviceID::DEV_GIMB_MTR_PITCH;
  gimbal_mtr_pitch_initparam.interfaceId = EInterfaceID::INF_CAN3;
  gimbal_mtr_pitch_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_7;
  gimbal_mtr_pitch_initparam.useAngleToPosit = true;
  gimbal_mtr_pitch_initparam.useStallMonit = true;
  gimbal_mtr_pitch_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  gimbal_mtr_pitch.InitDevice(&gimbal_mtr_pitch_initparam);

  /***************************************************
   * Gantry Motor Initialize
   ***************************************************/
  static CMtr_M3508 gantry_uplift_mtr_l;
  CMtr_M3508::SMtrM3508InitParam gantry_uplift_mtr_l_initparam;
  gantry_uplift_mtr_l_initparam.deviceID = EDeviceID::DEV_LIFT_MTR_L;
  gantry_uplift_mtr_l_initparam.interfaceId = EInterfaceID::INF_CAN1;
  gantry_uplift_mtr_l_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_2;
  gantry_uplift_mtr_l_initparam.useAngleToPosit = true;
  gantry_uplift_mtr_l_initparam.useStallMonit = true;
  gantry_uplift_mtr_l.InitDevice(&gantry_uplift_mtr_l_initparam);

  static CMtr_M3508 gantry_uplift_mtr_r;
  CMtr_M3508::SMtrM3508InitParam gantry_uplift_mtr_r_initparam;
  gantry_uplift_mtr_r_initparam.deviceID = EDeviceID::DEV_LIFT_MTR_R;
  gantry_uplift_mtr_r_initparam.interfaceId = EInterfaceID::INF_CAN1;
  gantry_uplift_mtr_r_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_1;
  gantry_uplift_mtr_r_initparam.useAngleToPosit = true;
  gantry_uplift_mtr_r_initparam.useStallMonit = true;
  gantry_uplift_mtr_r.InitDevice(&gantry_uplift_mtr_r_initparam);

  static CMtr_M3508 gantry_strch_mtr_l;
  CMtr_M3508::SMtrM3508InitParam gantry_strch_mtr_l_initparam;
  gantry_strch_mtr_l_initparam.deviceID = EDeviceID::DEV_STRCH_MTR_L;
  gantry_strch_mtr_l_initparam.interfaceId = EInterfaceID::INF_CAN1;
  gantry_strch_mtr_l_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_3;
  gantry_strch_mtr_l_initparam.useAngleToPosit = true;
  gantry_strch_mtr_l_initparam.useStallMonit = true;
  gantry_strch_mtr_l.InitDevice(&gantry_strch_mtr_l_initparam);

  static CMtr_M3508 gantry_strch_mtr_r;
  CMtr_M3508::SMtrM3508InitParam gantry_strch_mtr_r_initparam;
  gantry_strch_mtr_r_initparam.deviceID = EDeviceID::DEV_STRCH_MTR_R;
  gantry_strch_mtr_r_initparam.interfaceId = EInterfaceID::INF_CAN1;
  gantry_strch_mtr_r_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_4;
  gantry_strch_mtr_r_initparam.useAngleToPosit = true;
  gantry_strch_mtr_r_initparam.useStallMonit = true;
  gantry_strch_mtr_r.InitDevice(&gantry_strch_mtr_r_initparam);

  static CMtr_M2006 gantryTravMotor;
  CMtr_M2006::SMtrM2006InitParam gantryTravMotorInitParam;
  gantryTravMotorInitParam.deviceID = EDeviceID::DEV_TRAV_MTR;
  gantryTravMotorInitParam.interfaceId = EInterfaceID::INF_CAN2;
  gantryTravMotorInitParam.djiMtrID = CMtrDji::EDjiMtrID::ID_5;
  gantryTravMotorInitParam.useAngleToPosit = true;
  gantryTravMotorInitParam.useStallMonit = true;
  gantryTravMotorInitParam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  gantryTravMotor.InitDevice(&gantryTravMotorInitParam);

  static CMtr_M2006 gantryFlipMotor_F;
  CMtr_M2006::SMtrM2006InitParam gantryFlipMotorInitParam_F;
  gantryFlipMotorInitParam_F.deviceID = EDeviceID::DEV_FLIP_MTR_F;
  gantryFlipMotorInitParam_F.interfaceId = EInterfaceID::INF_CAN1;
  gantryFlipMotorInitParam_F.djiMtrID = CMtrDji::EDjiMtrID::ID_5;
//  gantryFlipMotorInitParam_F.useAngleToPosit = true;
//  gantryFlipMotorInitParam_F.useStallMonit = true;
//  gantryFlipMotorInitParam_F.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  gantryFlipMotor_F.InitDevice(&gantryFlipMotorInitParam_F);

  static CMtr_M2006 gantryFlipMotor_B;
  CMtr_M2006::SMtrM2006InitParam gantryFlipMotorInitParam_B;
  gantryFlipMotorInitParam_B.deviceID = EDeviceID::DEV_FLIP_MTR_B;
  gantryFlipMotorInitParam_B.interfaceId = EInterfaceID::INF_CAN1;
  gantryFlipMotorInitParam_B.djiMtrID = CMtrDji::EDjiMtrID::ID_6;
//  gantryFlipMotorInitParam_B.useAngleToPosit = true;
//  gantryFlipMotorInitParam_B.useStallMonit = true;
//  gantryFlipMotorInitParam_B.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  gantryFlipMotor_B.InitDevice(&gantryFlipMotorInitParam_B);

  /***************************************************
   * Manipulator Motor Initialize
   ***************************************************/
  static CMtr_M2006 jointYawMotor;
  CMtr_M2006::SMtrM2006InitParam jointYawMotorInitParam;
  jointYawMotorInitParam.deviceID = EDeviceID::DEV_MANIP_MTR_YAW;
  jointYawMotorInitParam.interfaceId = EInterfaceID::INF_CAN2;
  jointYawMotorInitParam.djiMtrID = CMtrDji::EDjiMtrID::ID_4;
  jointYawMotorInitParam.useAngleToPosit = true;
  jointYawMotorInitParam.useStallMonit = true;
  jointYawMotorInitParam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  jointYawMotor.InitDevice(&jointYawMotorInitParam);

  static CMtr_M2006 jointRollMotor;
  CMtr_M2006::SMtrM2006InitParam jointRollMotorInitParam;
  jointRollMotorInitParam.deviceID = EDeviceID::DEV_MANIP_MTR_ROLL;
  jointRollMotorInitParam.interfaceId = EInterfaceID::INF_CAN2;
  jointRollMotorInitParam.djiMtrID = CMtrDji::EDjiMtrID::ID_3;
  jointRollMotorInitParam.useAngleToPosit = true;
  jointRollMotorInitParam.useStallMonit = true;
  jointRollMotorInitParam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  jointRollMotor.InitDevice(&jointRollMotorInitParam);

  static CMtr_M2006 manipMotorEnd_L;
  CMtr_M2006::SMtrM2006InitParam manipMotorEndInitParam_L;
  manipMotorEndInitParam_L.deviceID = EDeviceID::DEV_MANIP_MTR_END_L;
  manipMotorEndInitParam_L.interfaceId = EInterfaceID::INF_CAN2;
  manipMotorEndInitParam_L.djiMtrID = CMtrDji::EDjiMtrID::ID_1;
  manipMotorEndInitParam_L.useAngleToPosit = true;
  manipMotorEndInitParam_L.useStallMonit = true;
  manipMotorEndInitParam_L.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  manipMotorEnd_L.InitDevice(&manipMotorEndInitParam_L);

  static CMtr_M2006 manipMotorEnd_R;
  CMtr_M2006::SMtrM2006InitParam manipMotorEndInitParam_R;
  manipMotorEndInitParam_R.deviceID = EDeviceID::DEV_MANIP_MTR_END_R;
  manipMotorEndInitParam_R.interfaceId = EInterfaceID::INF_CAN2;
  manipMotorEndInitParam_R.djiMtrID = CMtrDji::EDjiMtrID::ID_2;
  manipMotorEndInitParam_R.useAngleToPosit = true;
  manipMotorEndInitParam_R.useStallMonit = true;
  manipMotorEndInitParam_R.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  manipMotorEnd_R.InitDevice(&manipMotorEndInitParam_R);


  /***************************************************
   * Flip Motor Initialize
   ***************************************************/
  static CMtr_M2006 flipMotor_F;
  CMtr_M2006::SMtrM2006InitParam flipMotorInitParam_F;
  flipMotorInitParam_F.deviceID = EDeviceID::DEV_FLIP_MTR_F;
  flipMotorInitParam_F.interfaceId = EInterfaceID::INF_CAN1;
  flipMotorInitParam_F.djiMtrID = CMtrDji::EDjiMtrID::ID_5;
  flipMotor_F.InitDevice(&flipMotorInitParam_F);

  static CMtr_M2006 flipMotor_B;
  CMtr_M2006::SMtrM2006InitParam flipMotorInitParam_B;
  flipMotorInitParam_B.deviceID = EDeviceID::DEV_FLIP_MTR_B;
  flipMotorInitParam_B.interfaceId = EInterfaceID::INF_CAN1;
  flipMotorInitParam_B.djiMtrID = CMtrDji::EDjiMtrID::ID_6;
  flipMotor_B.InitDevice(&flipMotorInitParam_B);

  return RP_OK;
}

} // namespace robotpilots
