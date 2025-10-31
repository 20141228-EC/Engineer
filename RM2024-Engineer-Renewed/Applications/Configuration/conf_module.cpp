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
  chassisInitParam.chassisMtrCanInfID = EInterfaceID::INF_CAN3;
  chassisInitParam.wheelsetMotorID_LF = EDeviceID::DEV_CHAS_MTR_LF;
  chassisInitParam.wheelsetMotorID_RF = EDeviceID::DEV_CHAS_MTR_RF;
  chassisInitParam.wheelsetMotorID_LB = EDeviceID::DEV_CHAS_MTR_LB;
  chassisInitParam.wheelsetMotorID_RB = EDeviceID::DEV_CHAS_MTR_RB;
  chassisInitParam.wheelsetSpdPidParam.kp = 15.0f;
  chassisInitParam.wheelsetSpdPidParam.ki = 0.6f;
  chassisInitParam.wheelsetSpdPidParam.kd = 0.0f;
  chassisInitParam.wheelsetSpdPidParam.maxInteger = 3000;
  chassisInitParam.wheelsetSpdPidParam.maxOutput = 8000;
  chassisInitParam.yawCorrectionPidParam.kp = 2.5f;
  chassisInitParam.yawCorrectionPidParam.ki = 2.0f;
  chassisInitParam.yawCorrectionPidParam.kd = 0.0f;
  chassisInitParam.yawCorrectionPidParam.deadband = 5;
  chassisInitParam.yawCorrectionPidParam.maxInteger = 20;
  chassisInitParam.yawCorrectionPidParam.maxOutput = 360;
  static auto chassisModule = CModChassis(chassisInitParam);

  /* Initialize Gimbal Module */
  CModGimbal::SModGimbalInitParam gimbalInitParam;
  gimbalInitParam.moduleId = EModuleID::MOD_GIMBAL;
  gimbalInitParam.gimbalMtrCanStdID = 0x1FF;
  gimbalInitParam.gimbalMtrCanInfID = EInterfaceID::INF_CAN3;
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
  gimbalInitParam.pitchMotorID = EDeviceID::DEV_GIMB_MTR_PITCH;
  gimbalInitParam.pitchPosPidParam.kp = 0.3f;
  gimbalInitParam.pitchPosPidParam.ki = 0.0f;
  gimbalInitParam.pitchPosPidParam.kd = 0.0f;
  gimbalInitParam.pitchPosPidParam.maxOutput = 3000;
  gimbalInitParam.pitchSpdPidParam.kp = 3.0f;
  gimbalInitParam.pitchSpdPidParam.ki = 0.3f;
  gimbalInitParam.pitchSpdPidParam.kd = 0.0f;
  gimbalInitParam.pitchSpdPidParam.maxInteger = 2000;
  gimbalInitParam.pitchSpdPidParam.maxOutput = 3000;
  static auto gimbalModule = CModGimbal(gimbalInitParam);

  /* Initialize Gantry Module */
  CModGantry::SModGantryInitParam gantryInitParam;
  gantryInitParam.moduleId = EModuleID::MOD_GANTRY;
  gantryInitParam.pumpGpioPort_1 = PUMP_EN1_GPIO_Port;
  gantryInitParam.pumpGpioPin_1 = PUMP_EN1_Pin;
  gantryInitParam.pumpGpioPort_2 = PUMP_EN2_GPIO_Port;
  gantryInitParam.pumpGpioPin_2 = PUMP_EN2_Pin;
  gantryInitParam.gantryMtrCanStdID = 0x200;
  gantryInitParam.gantryMtrCanInfID = EInterfaceID::INF_CAN1;
  gantryInitParam.gantryTravMtrCanStdID = 0x1FF;
  gantryInitParam.gantryTravMtrCanInfID = EInterfaceID::INF_CAN2;
  gantryInitParam.gantryManipMtrCanStdID = 0x200;
  gantryInitParam.gantryManipMtrCanInfID = EInterfaceID::INF_CAN2;
  gantryInitParam.gantryFlipMtrCanStdID = 0x1FF;
  gantryInitParam.gantryFlipMtrCanInfID = EInterfaceID::INF_CAN1;
  gantryInitParam.liftMotorID_L = EDeviceID::DEV_LIFT_MTR_L;
  gantryInitParam.liftMotorID_R = EDeviceID::DEV_LIFT_MTR_R;
  gantryInitParam.liftPosPidParam.kp = 0.3f;
  gantryInitParam.liftPosPidParam.ki = 0.0f;
  gantryInitParam.liftPosPidParam.kd = 0.0f;
  gantryInitParam.liftPosPidParam.maxOutput = 3000;
  gantryInitParam.liftSpdPidParam.kp = 10.0f;
  gantryInitParam.liftSpdPidParam.ki = 0.8f;
  gantryInitParam.liftSpdPidParam.kd = 0.0f;
  gantryInitParam.liftSpdPidParam.maxInteger = 3000;
  gantryInitParam.liftSpdPidParam.maxOutput = 8000;
  gantryInitParam.stretchMotorID_L = EDeviceID::DEV_STRCH_MTR_L;
  gantryInitParam.stretchMotorID_R = EDeviceID::DEV_STRCH_MTR_R;
  gantryInitParam.stretchPosPidParam.kp = 0.2f;
  gantryInitParam.stretchPosPidParam.ki = 0.0f;
  gantryInitParam.stretchPosPidParam.kd = 0.0f;
  gantryInitParam.stretchPosPidParam.maxOutput = 3000;
  gantryInitParam.stretchSpdPidParam.kp = 10.0f;
  gantryInitParam.stretchSpdPidParam.ki = 0.5f;
  gantryInitParam.stretchSpdPidParam.kd = 0.0f;
  gantryInitParam.stretchSpdPidParam.maxInteger = 3000;
  gantryInitParam.stretchSpdPidParam.maxOutput = 6000;
  gantryInitParam.traverseMotorID = EDeviceID::DEV_TRAV_MTR;
  gantryInitParam.traversePosPidParam.kp = 0.2f;
  gantryInitParam.traversePosPidParam.ki = 0.0f;
  gantryInitParam.traversePosPidParam.kd = 0.0f;
  gantryInitParam.traversePosPidParam.maxOutput = 4000;
  gantryInitParam.traverseSpdPidParam.kp = 5.0f;
  gantryInitParam.traverseSpdPidParam.ki = 0.3f;
  gantryInitParam.traverseSpdPidParam.kd = 0.0f;
  gantryInitParam.traverseSpdPidParam.maxInteger = 5000;
  gantryInitParam.traverseSpdPidParam.maxOutput = 4000;
  gantryInitParam.jointMotorID_Yaw = EDeviceID::DEV_MANIP_MTR_YAW;
  gantryInitParam.jointMotorID_Roll = EDeviceID::DEV_MANIP_MTR_ROLL;
  gantryInitParam.jointPosPidParam.kp = 0.3f;
  gantryInitParam.jointPosPidParam.ki = 0.0f;
  gantryInitParam.jointPosPidParam.kd = 0.0f;
  gantryInitParam.jointPosPidParam.maxOutput = 4000;
  gantryInitParam.jointSpdPidParam.kp = 5.0f;
  gantryInitParam.jointSpdPidParam.ki = 0.5f;
  gantryInitParam.jointSpdPidParam.kd = 0.0f;
  gantryInitParam.jointSpdPidParam.maxInteger = 6000;
  gantryInitParam.jointSpdPidParam.maxOutput = 4000;
  gantryInitParam.endMotorID_L = EDeviceID::DEV_MANIP_MTR_END_L;
  gantryInitParam.endMotorID_R = EDeviceID::DEV_MANIP_MTR_END_R;
  gantryInitParam.endPosPidParam.kp = 0.2f;
  gantryInitParam.endPosPidParam.ki = 0.0f;
  gantryInitParam.endPosPidParam.kd = 0.0f;
  gantryInitParam.endPosPidParam.maxOutput = 4000;
  gantryInitParam.endSpdPidParam.kp = 2.0f;
  gantryInitParam.endSpdPidParam.ki = 0.5f;
  gantryInitParam.endSpdPidParam.kd = 0.0f;
  gantryInitParam.endSpdPidParam.maxInteger = 6000;
  gantryInitParam.endSpdPidParam.maxOutput = 4000;
  gantryInitParam.flipMotorID_F = EDeviceID::DEV_FLIP_MTR_F;
  gantryInitParam.flipMotorID_B = EDeviceID::DEV_FLIP_MTR_B;
  gantryInitParam.flipSpdPidParam.kp = 10.0f;
  gantryInitParam.flipSpdPidParam.ki = 0.5f;
  gantryInitParam.flipSpdPidParam.kd = 0.0f;
  gantryInitParam.flipSpdPidParam.maxInteger = 6000;
  gantryInitParam.flipSpdPidParam.maxOutput = 4000;
  static auto gantryModule = CModGantry(gantryInitParam);

  return RP_OK;
}

} // namespace robotpilots
