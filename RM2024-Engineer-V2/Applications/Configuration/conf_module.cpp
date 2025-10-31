/**
 * @file        conf_module.cpp
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

#include "conf_module.hpp"
#include "Module.hpp"

extern TIM_HandleTypeDef htim2;

namespace robotpilots {

ERpStatus InitializeModule() {

  /* Initialize Chassis Module */
  CModChassis::SModChassisInitParam chassisInitParam;
  chassisInitParam.moduleId = EModuleID::MOD_CHASSIS;
  chassisInitParam.memsDevID = EDeviceID::DEV_MEMS_BMI088;
  chassisInitParam.chassisMtrCanStdID = 0x200;
  chassisInitParam.chassisMtrCanInfID = EInterfaceID::INF_CAN1;
  chassisInitParam.wheelsetMotorID_LF = EDeviceID::DEV_CHAS_MTR_LF;
  chassisInitParam.wheelsetMotorID_RF = EDeviceID::DEV_CHAS_MTR_RF;
  chassisInitParam.wheelsetMotorID_LB = EDeviceID::DEV_CHAS_MTR_LB;
  chassisInitParam.wheelsetMotorID_RB = EDeviceID::DEV_CHAS_MTR_RB;
  chassisInitParam.wheelsetSpdPidParam.kp = 15.0f;
  chassisInitParam.wheelsetSpdPidParam.ki = 1.0f;
  chassisInitParam.wheelsetSpdPidParam.kd = 0.0f;
  chassisInitParam.wheelsetSpdPidParam.maxInteger = 2000;
  chassisInitParam.wheelsetSpdPidParam.maxOutput = 10000;
  chassisInitParam.yawCorrectionPidParam.kp = 5.0f;
  chassisInitParam.yawCorrectionPidParam.ki = 5.0f;
  chassisInitParam.yawCorrectionPidParam.kd = 0.0f;
  chassisInitParam.yawCorrectionPidParam.deadband = 1;
  chassisInitParam.yawCorrectionPidParam.maxInteger = 20;
  chassisInitParam.yawCorrectionPidParam.maxOutput = 360;
  static auto chassisModule = CModChassis(chassisInitParam);

  /* Initialize Gimbal Module */
  CModGimbal::SModGimbalInitParam gimbalInitParam;
  gimbalInitParam.moduleId = EModuleID::MOD_GIMBAL;
  gimbalInitParam.gimbalPitchHalTimHandle = &htim2;
  gimbalInitParam.gimbalPitchTimChannel = TIM_CHANNEL_1;
  gimbalInitParam.gimbalMtrCanStdID = 0x1FF;
  gimbalInitParam.gimbalMtrCanInfID = EInterfaceID::INF_CAN1;
  gimbalInitParam.liftMotorID_L = EDeviceID::DEV_GIMB_MTR_L;
  gimbalInitParam.liftMotorID_R = EDeviceID::DEV_GIMB_MTR_R;
  gimbalInitParam.liftPosPidParam.kp = 0.2f;
  gimbalInitParam.liftPosPidParam.ki = 0.0f;
  gimbalInitParam.liftPosPidParam.kd = 0.0f;
  gimbalInitParam.liftPosPidParam.maxOutput = 6000;
  gimbalInitParam.liftSpdPidParam.kp = 2.0f;
  gimbalInitParam.liftSpdPidParam.ki = 0.4f;
  gimbalInitParam.liftSpdPidParam.kd = 0.0f;
  gimbalInitParam.liftSpdPidParam.maxInteger = 3000;
  gimbalInitParam.liftSpdPidParam.maxOutput = 4000;
  static auto gimbalModule = CModGimbal(gimbalInitParam);

  /* Initialize Gantry Module */
  CModGantry::SModGantryInitParam gantryInitParam;
  gantryInitParam.moduleId = EModuleID::MOD_GANTRY;
  gantryInitParam.pumpGpioPort_L = PUMP_EN1_GPIO_Port;
  gantryInitParam.pumpGpioPin_L = PUMP_EN1_Pin;
  gantryInitParam.pumpGpioPort_C = PUMP_EN3_GPIO_Port;
  gantryInitParam.pumpGpioPin_C = PUMP_EN3_Pin;
  gantryInitParam.pumpGpioPort_R = PUMP_EN2_GPIO_Port;
  gantryInitParam.pumpGpioPin_R = PUMP_EN2_Pin;
  gantryInitParam.gantryMtrCanStdID = 0x200;
  gantryInitParam.gantryMtrCanInfID = EInterfaceID::INF_CAN3;
  gantryInitParam.gantryLiftMtrCanStdID = 0x200;
  gantryInitParam.gantryLiftMtrCanInfID = EInterfaceID::INF_CAN2;
  gantryInitParam.gantryJointMtrCanStdID = 0x3FE;
  gantryInitParam.gantryJointMtrCanInfID = EInterfaceID::INF_CAN3;
  gantryInitParam.liftMotorID_L = EDeviceID::DEV_LIFT_MTR_L;
  gantryInitParam.liftMotorID_R = EDeviceID::DEV_LIFT_MTR_R;
  gantryInitParam.liftPosPidParam.kp = 0.3f;
  gantryInitParam.liftPosPidParam.ki = 0.0f;
  gantryInitParam.liftPosPidParam.kd = 8.0f;
  gantryInitParam.liftPosPidParam.maxOutput = 3000;
  gantryInitParam.liftSpdPidParam.kp = 10.0f;
  gantryInitParam.liftSpdPidParam.ki = 0.8f;
  gantryInitParam.liftSpdPidParam.kd = 0.0f;
  gantryInitParam.liftSpdPidParam.maxInteger = 5000;
  gantryInitParam.liftSpdPidParam.maxOutput = 8000;
  gantryInitParam.stretchMotorID = EDeviceID::DEV_STRCH_MTR;
  gantryInitParam.stretchPosPidParam.kp = 0.2f;
  gantryInitParam.stretchPosPidParam.ki = 0.0f;
  gantryInitParam.stretchPosPidParam.kd = 2.0f;
  gantryInitParam.stretchPosPidParam.maxOutput = 4000;
  gantryInitParam.stretchSpdPidParam.kp = 5.0f;
  gantryInitParam.stretchSpdPidParam.ki = 0.4f;
  gantryInitParam.stretchSpdPidParam.kd = 0.0f;
  gantryInitParam.stretchSpdPidParam.maxInteger = 5000;
  gantryInitParam.stretchSpdPidParam.maxOutput = 6000;
  gantryInitParam.traverseMotorID = EDeviceID::DEV_TRAV_MTR;
  gantryInitParam.traversePosPidParam.kp = 0.4f;
  gantryInitParam.traversePosPidParam.ki = 0.0f;
  gantryInitParam.traversePosPidParam.kd = 2.0f;
  gantryInitParam.traversePosPidParam.maxOutput = 6000;
  gantryInitParam.traverseSpdPidParam.kp = 5.0f;
  gantryInitParam.traverseSpdPidParam.ki = 0.3f;
  gantryInitParam.traverseSpdPidParam.kd = 0.0f;
  gantryInitParam.traverseSpdPidParam.maxInteger = 5000;
  gantryInitParam.traverseSpdPidParam.maxOutput = 4000;
  gantryInitParam.jointMotorID_Yaw = EDeviceID::DEV_MANIP_MTR_YAW;
  gantryInitParam.jointMotorID_Roll = EDeviceID::DEV_MANIP_MTR_ROLL;
  gantryInitParam.jointPosPidParam.kp = 0.4f;
  gantryInitParam.jointPosPidParam.ki = 0.0f;
  gantryInitParam.jointPosPidParam.kd = 1.0f;
  gantryInitParam.jointPosPidParam.maxOutput = 50;
  gantryInitParam.jointSpdPidParam.kp = 0.4f;
  gantryInitParam.jointSpdPidParam.ki = 0.3f;
  gantryInitParam.jointSpdPidParam.kd = 0.0f;
  gantryInitParam.jointSpdPidParam.maxInteger = 50;
  gantryInitParam.jointSpdPidParam.maxOutput = 50;
  gantryInitParam.endMotorID_L = EDeviceID::DEV_MANIP_MTR_END_L;
  gantryInitParam.endMotorID_R = EDeviceID::DEV_MANIP_MTR_END_R;
  gantryInitParam.endPosPidParam.kp = 0.5f;
  gantryInitParam.endPosPidParam.ki = 0.0f;
  gantryInitParam.endPosPidParam.kd = 0.0f;
  gantryInitParam.endPosPidParam.maxOutput = 4000;
  gantryInitParam.endSpdPidParam.kp = 2.0f;
  gantryInitParam.endSpdPidParam.ki = 0.5f;
  gantryInitParam.endSpdPidParam.kd = 0.0f;
  gantryInitParam.endSpdPidParam.maxInteger = 6000;
  gantryInitParam.endSpdPidParam.maxOutput = 4000;
  static auto gantryModule = CModGantry(gantryInitParam);

  /* Initialize Sub-Gantry Module */
  CModSubGantry::SModSubGantryInitParam subGantryInitParam;
  subGantryInitParam.moduleId = EModuleID::MOD_SUBGANTRY;
  subGantryInitParam.subGantryMtrCanStdID = 0x1FF;
  subGantryInitParam.subGantryMtrCanInfID = EInterfaceID::INF_CAN2;
  subGantryInitParam.liftMotorID_L = EDeviceID::DEV_SUBLIFT_MTR_L;
  subGantryInitParam.liftMotorID_R = EDeviceID::DEV_SUBLIFT_MTR_R;
  subGantryInitParam.liftPosPidParam.kp = 0.3f;
  subGantryInitParam.liftPosPidParam.ki = 0.0f;
  subGantryInitParam.liftPosPidParam.kd = 0.0f;
  subGantryInitParam.liftPosPidParam.maxOutput = 6000;
  subGantryInitParam.liftSpdPidParam.kp = 5.0f;
  subGantryInitParam.liftSpdPidParam.ki = 0.3f;
  subGantryInitParam.liftSpdPidParam.kd = 0.0f;
  subGantryInitParam.liftSpdPidParam.maxInteger = 5000;
  subGantryInitParam.liftSpdPidParam.maxOutput = 4000;
  subGantryInitParam.stretchMotorID_L = EDeviceID::DEV_SUBSTRCH_MTR_L;
  subGantryInitParam.stretchMotorID_R = EDeviceID::DEV_SUBSTRCH_MTR_R;
  subGantryInitParam.stretchPosPidParam.kp = 0.3f;
  subGantryInitParam.stretchPosPidParam.ki = 0.0f;
  subGantryInitParam.stretchPosPidParam.kd = 0.0f;
  subGantryInitParam.stretchPosPidParam.maxOutput = 6000;
  subGantryInitParam.stretchSpdPidParam.kp = 5.0f;
  subGantryInitParam.stretchSpdPidParam.ki = 0.3f;
  subGantryInitParam.stretchSpdPidParam.kd = 0.0f;
  subGantryInitParam.stretchSpdPidParam.maxInteger = 5000;
  subGantryInitParam.stretchSpdPidParam.maxOutput = 4000;
  static auto subGantryModule = CModSubGantry(subGantryInitParam);

  return RP_OK;
}

} // namespace robotpilots
