/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-26
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-26   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "Core.hpp"

namespace robotpilots {

/**
 * @brief
 * @param arg
 */
void CSystemCore::StartReturnOriginTask(void *arg) {

  if (arg == nullptr) proc_return();

  /* Get System Core Handle */
  auto &core         = *reinterpret_cast<CSystemCore *>(arg);
  auto cnt           = 0;
  const auto timeout = 60000 / 5;   // unit: ms
  CSysVision::SOreTankInfo oreTankInfo;

  /* Set Auto Control Flag */
  core.gimbal_->gimbalCmd.isAutoCtrl = true;
  core.gantry_->gantryCmd.isAutoCtrl = true;

  core.gimbal_->gimbalCmd.setPosit_Lift = 80.0f;
  core.gimbal_->gimbalCmd.setStep_Pitch = 0;

  core.gantry_->gantryCmd.setPumpOn = false;
  core.gantry_->gantryCmd.setPosit_Lift = 0.0f;
  core.gantry_->gantryCmd.setPosit_Traverse = 0.0f;
  if (abs(core.gantry_->gantryInfo.angle_Joint_Yaw - 180.0f) > 10.0f)
    core.gantry_->gantryCmd.setPosit_Stretch = 110.0f;
  if (core.gantry_->gantryInfo.isPositArrived_Joint_Roll)
    core.gantry_->gantryCmd.setAngle_Joint_Yaw = 60.0f;
  core.gantry_->gantryCmd.setAngle_Joint_Roll = 180.0f;
  proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Joint_Roll);
  core.gantry_->gantryCmd.setAngle_Joint_Yaw = 180.0f;
  core.gantry_->gantryCmd.setAngle_End_Pitch = 90.0f;
  core.gantry_->gantryCmd.setAngle_End_Roll = 0.0f;
  proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Joint_Yaw);
  core.gantry_->gantryCmd.setPosit_Stretch = 0.0f;

  /* Process Exit */
proc_exit:
  core.gimbal_->gimbalCmd.isAutoCtrl = false;
  core.gantry_->gantryCmd.isAutoCtrl = false;
  core.autoCtrlTaskHandle_           = nullptr;
  core.currentAutoCtrlProcess_       = EAutoCtrlProcess::NONE;
  proc_return();
}

}
