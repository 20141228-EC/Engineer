/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-20
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-20   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "Core.hpp"

namespace robotpilots {

/**
 * @brief
 * @param arg
 */
void CSystemCore::StartReturnDriveTask(void *arg) {

  if (arg == nullptr) proc_return();

  /* Get System Core Handle */
  auto &core = *reinterpret_cast<CSystemCore *>(arg);
  auto cnt           = 0;
  const auto timeout = 60000 / 5;// unit: ms

  /* Set Auto Control Flag */
  core.gantry_->gantryCmd.isAutoCtrl       = true;
  core.subgantry_->subGantryCmd.isAutoCtrl = true;

  /* Set Gimbal */
  core.gimbal_->gimbalCmd.setPosit_Lift = 150.0f;
  core.gimbal_->gimbalCmd.setStep_Pitch = 0;

  /* Set Sub-Gantry */
  core.subgantry_->subGantryCmd.setPosit_Lift_L = 0.0f;
  core.subgantry_->subGantryCmd.setPosit_Lift_R = 0.0f;
  core.subgantry_->subGantryCmd.setPosit_Stretch_L = 0.0f;
  core.subgantry_->subGantryCmd.setPosit_Stretch_R = 0.0f;

  /* Set Gantry */
  core.gantry_->gantryCmd.setPumpOn_C = true;
  core.gantry_->gantryCmd.setPosit_Lift = 180.0f;
  core.gantry_->gantryCmd.setPosit_Stretch = 0.0f;
  core.gantry_->gantryCmd.setPosit_Traverse = 190.0f / 2;
  core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
  core.gantry_->gantryCmd.setAngle_Joint_Roll = 0.0f;
  core.gantry_->gantryCmd.setAngle_End_Pitch = 0.0f;
  proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Joint_Yaw
                 && core.gantry_->gantryInfo.isPositArrived_Joint_Roll);
  proc_waitMs(500);
  core.gantry_->gantryCmd.setAngle_End_Pitch = -90.0f;

  /* Process Exit */
proc_exit:
  core.gimbal_->gimbalCmd.isAutoCtrl = false;
  core.gantry_->gantryCmd.isAutoCtrl = false;
  core.subgantry_->subGantryCmd.isAutoCtrl = false;
  core.autoCtrlTaskHandle_ = nullptr;
  core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
  proc_return();
}

} // namespace robotpilots
