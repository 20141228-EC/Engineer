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
  chassisMotorInitParam_LF.interfaceId = EInterfaceID::INF_CAN1;
  chassisMotorInitParam_LF.djiMtrID = CMtrDji::EDjiMtrID::ID_1;
  chassisMotor_LF.InitDevice(&chassisMotorInitParam_LF);

  static CMtr_M3508 chassisMotor_RF;
  CMtr_M3508::SMtrM3508InitParam chassisMotorInitParam_RF;
  chassisMotorInitParam_RF.deviceID = EDeviceID::DEV_CHAS_MTR_RF;
  chassisMotorInitParam_RF.interfaceId = EInterfaceID::INF_CAN1;
  chassisMotorInitParam_RF.djiMtrID = CMtrDji::EDjiMtrID::ID_4;
  chassisMotor_RF.InitDevice(&chassisMotorInitParam_RF);

  static CMtr_M3508 chassis_mtr_lb;
  CMtr_M3508::SMtrM3508InitParam chassis_mtr_lb_initparam;
  chassis_mtr_lb_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_LB;
  chassis_mtr_lb_initparam.interfaceId = EInterfaceID::INF_CAN1;
  chassis_mtr_lb_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_2;
  // chassis_mtr_lf_initparam.useAngleToPosit = true;
  chassis_mtr_lb.InitDevice(&chassis_mtr_lb_initparam);

  static CMtr_M3508 chassis_mtr_rb;
  CMtr_M3508::SMtrM3508InitParam chassis_mtr_rb_initparam;
  chassis_mtr_rb_initparam.deviceID = EDeviceID::DEV_CHAS_MTR_RB;
  chassis_mtr_rb_initparam.interfaceId = EInterfaceID::INF_CAN1;
  chassis_mtr_rb_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_3;
  // chassis_mtr_lf_initparam.useAngleToPosit = true;
  chassis_mtr_rb.InitDevice(&chassis_mtr_rb_initparam);


  /***************************************************
   * Gimbal Motor Initialize
   ***************************************************/
  static CMtr_M2006 gimbal_mtr_l;
  CMtr_M2006::SMtrM2006InitParam gimbal_mtr_l_initparam;
  gimbal_mtr_l_initparam.deviceID = EDeviceID::DEV_GIMB_MTR_L;
  gimbal_mtr_l_initparam.interfaceId = EInterfaceID::INF_CAN1;
  gimbal_mtr_l_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_5;
  gimbal_mtr_l_initparam.useAngleToPosit = true;
  gimbal_mtr_l_initparam.useStallMonit = true;
  gimbal_mtr_l_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  gimbal_mtr_l.InitDevice(&gimbal_mtr_l_initparam);

  static CMtr_M2006 gimbal_mtr_r;
  CMtr_M2006::SMtrM2006InitParam gimbal_mtr_r_initparam;
  gimbal_mtr_r_initparam.deviceID = EDeviceID::DEV_GIMB_MTR_R;
  gimbal_mtr_r_initparam.interfaceId = EInterfaceID::INF_CAN1;
  gimbal_mtr_r_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_6;
  gimbal_mtr_r_initparam.useAngleToPosit = true;
  gimbal_mtr_r_initparam.useStallMonit = true;
  gimbal_mtr_r_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  gimbal_mtr_r.InitDevice(&gimbal_mtr_r_initparam);

  // TODO: Gimbal Pitch Motor Init

  /***************************************************
   * Gantry Motor Initialize
   ***************************************************/
  static CMtr_M3508 gantry_uplift_mtr_l;
  CMtr_M3508::SMtrM3508InitParam gantry_uplift_mtr_l_initparam;
  gantry_uplift_mtr_l_initparam.deviceID = EDeviceID::DEV_LIFT_MTR_L;
  gantry_uplift_mtr_l_initparam.interfaceId = EInterfaceID::INF_CAN2;
  gantry_uplift_mtr_l_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_1;
  gantry_uplift_mtr_l_initparam.useAngleToPosit = true;
  gantry_uplift_mtr_l_initparam.useStallMonit = true;
  gantry_uplift_mtr_l.InitDevice(&gantry_uplift_mtr_l_initparam);

  static CMtr_M3508 gantry_uplift_mtr_r;
  CMtr_M3508::SMtrM3508InitParam gantry_uplift_mtr_r_initparam;
  gantry_uplift_mtr_r_initparam.deviceID = EDeviceID::DEV_LIFT_MTR_R;
  gantry_uplift_mtr_r_initparam.interfaceId = EInterfaceID::INF_CAN2;
  gantry_uplift_mtr_r_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_2;
  gantry_uplift_mtr_r_initparam.useAngleToPosit = true;
  gantry_uplift_mtr_r_initparam.useStallMonit = true;
  gantry_uplift_mtr_r.InitDevice(&gantry_uplift_mtr_r_initparam);

  static CMtr_M3508 gantry_strch_mtr;
  CMtr_M3508::SMtrM3508InitParam gantry_strch_mtr_initparam;
  gantry_strch_mtr_initparam.deviceID = EDeviceID::DEV_STRCH_MTR;
  gantry_strch_mtr_initparam.interfaceId = EInterfaceID::INF_CAN3;
  gantry_strch_mtr_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_1;
  gantry_strch_mtr_initparam.useAngleToPosit = true;
  gantry_strch_mtr_initparam.useStallMonit = true;
  gantry_strch_mtr.InitDevice(&gantry_strch_mtr_initparam);

  static CMtr_M2006 gantry_trav_mtr;
  CMtr_M2006::SMtrM2006InitParam gantry_trav_mtr_initparam;
  gantry_trav_mtr_initparam.deviceID = EDeviceID::DEV_TRAV_MTR;
  gantry_trav_mtr_initparam.interfaceId = EInterfaceID::INF_CAN3;
  gantry_trav_mtr_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_2;
  gantry_trav_mtr_initparam.useAngleToPosit = true;
  gantry_trav_mtr_initparam.useStallMonit = true;
  gantry_trav_mtr_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  gantry_trav_mtr.InitDevice(&gantry_trav_mtr_initparam);


  /***************************************************
   * Sub-Gantry Motor Initialize
   ***************************************************/
  static CMtr_M2006 subgantry_uplift_mtr_l;
  CMtr_M2006::SMtrM2006InitParam subgantry_uplift_mtr_l_initparam;
  subgantry_uplift_mtr_l_initparam.deviceID = EDeviceID::DEV_SUBLIFT_MTR_L;
  subgantry_uplift_mtr_l_initparam.interfaceId = EInterfaceID::INF_CAN2;
  subgantry_uplift_mtr_l_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_5;
  subgantry_uplift_mtr_l_initparam.useAngleToPosit = true;
  subgantry_uplift_mtr_l_initparam.useStallMonit = true;
  subgantry_uplift_mtr_l_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  subgantry_uplift_mtr_l.InitDevice(&subgantry_uplift_mtr_l_initparam);

  static CMtr_M2006 subgantry_uplift_mtr_r;
  CMtr_M2006::SMtrM2006InitParam subgantry_uplift_mtr_r_initparam;
  subgantry_uplift_mtr_r_initparam.deviceID = EDeviceID::DEV_SUBLIFT_MTR_R;
  subgantry_uplift_mtr_r_initparam.interfaceId = EInterfaceID::INF_CAN2;
  subgantry_uplift_mtr_r_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_6;
  subgantry_uplift_mtr_r_initparam.useAngleToPosit = true;
  subgantry_uplift_mtr_r_initparam.useStallMonit = true;
  subgantry_uplift_mtr_r_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  subgantry_uplift_mtr_r.InitDevice(&subgantry_uplift_mtr_r_initparam);

  static CMtr_M2006 subgantry_strch_mtr_l;
  CMtr_M2006::SMtrM2006InitParam subgantry_strch_mtr_l_initparam;
  subgantry_strch_mtr_l_initparam.deviceID = EDeviceID::DEV_SUBSTRCH_MTR_L;
  subgantry_strch_mtr_l_initparam.interfaceId = EInterfaceID::INF_CAN2;
  subgantry_strch_mtr_l_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_7;
  subgantry_strch_mtr_l_initparam.useAngleToPosit = true;
  subgantry_strch_mtr_l_initparam.useStallMonit = true;
  subgantry_strch_mtr_l_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  subgantry_strch_mtr_l.InitDevice(&subgantry_strch_mtr_l_initparam);

  static CMtr_M2006 subgantry_strch_mtr_r;
  CMtr_M2006::SMtrM2006InitParam subgantry_strch_mtr_r_initparam;
  subgantry_strch_mtr_r_initparam.deviceID = EDeviceID::DEV_SUBSTRCH_MTR_R;
  subgantry_strch_mtr_r_initparam.interfaceId = EInterfaceID::INF_CAN2;
  subgantry_strch_mtr_r_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_8;
  subgantry_strch_mtr_r_initparam.useAngleToPosit = true;
  subgantry_strch_mtr_r_initparam.useStallMonit = true;
  subgantry_strch_mtr_r_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
  subgantry_strch_mtr_r.InitDevice(&subgantry_strch_mtr_r_initparam);

  /***************************************************
   * Manipulator Motor Initialize
   ***************************************************/
  static CMtrDm jointYawMotor;
  CMtrDm::SMtrDmInitParam jointYawMotorInitParam;
  jointYawMotorInitParam.deviceID = EDeviceID::DEV_MANIP_MTR_YAW;
  jointYawMotorInitParam.interfaceId = EInterfaceID::INF_CAN3;
  jointYawMotorInitParam.dmMtrID = CMtrDm::EDmMtrID::ID_1;
  jointYawMotorInitParam.useAngleToPosit = true;
  jointYawMotor.InitDevice(&jointYawMotorInitParam);

  static CMtrDm jointRollMotor;
  CMtrDm::SMtrDmInitParam jointRollMotorInitParam;
  jointRollMotorInitParam.deviceID = EDeviceID::DEV_MANIP_MTR_ROLL;
  jointRollMotorInitParam.interfaceId = EInterfaceID::INF_CAN3;
  jointRollMotorInitParam.dmMtrID = CMtrDm::EDmMtrID::ID_2;
  jointRollMotorInitParam.useAngleToPosit = true;
  jointRollMotor.InitDevice(&jointRollMotorInitParam);

  static CMtr_M2006 manip_mtr_end_l;
  CMtr_M2006::SMtrM2006InitParam manip_mtr_end_l_initparam;
  manip_mtr_end_l_initparam.deviceID = EDeviceID::DEV_MANIP_MTR_END_L;
  manip_mtr_end_l_initparam.interfaceId = EInterfaceID::INF_CAN3;
  manip_mtr_end_l_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_3;
  manip_mtr_end_l_initparam.useAngleToPosit = true;
  manip_mtr_end_l_initparam.useStallMonit = true;
  manip_mtr_end_l_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
//  manip_mtr_end_l_initparam.stallThreshold = 500;
  manip_mtr_end_l.InitDevice(&manip_mtr_end_l_initparam);

  static CMtr_M2006 manip_mtr_end_r;
  CMtr_M2006::SMtrM2006InitParam manip_mtr_end_r_initparam;
  manip_mtr_end_r_initparam.deviceID = EDeviceID::DEV_MANIP_MTR_END_R;
  manip_mtr_end_r_initparam.interfaceId = EInterfaceID::INF_CAN3;
  manip_mtr_end_r_initparam.djiMtrID = CMtrDji::EDjiMtrID::ID_4;
  manip_mtr_end_r_initparam.useAngleToPosit = true;
  manip_mtr_end_r_initparam.useStallMonit = true;
  manip_mtr_end_r_initparam.stallMonitDataSrc = CMtrInstance::DATA_TORQUE;
//  manip_mtr_end_r_initparam.stallThreshold = 500;
  manip_mtr_end_r.InitDevice(&manip_mtr_end_r_initparam);

  return RP_OK;
}

} // namespace robotpilots
