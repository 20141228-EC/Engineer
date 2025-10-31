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
void CSystemCore::StartSilverOreTask(void *arg) {

  if (arg == nullptr) proc_return();

  /* Get System Core Handle */
  auto &core         = *reinterpret_cast<CSystemCore *>(arg);
  auto cnt           = 0;
  const auto timeout = 60000 / 5;// unit: ms
  CSysVision::SOreTankInfo oreTankInfo;

  /* Set Auto Control Flag */
  core.gimbal_->gimbalCmd.isAutoCtrl = true;
  core.gantry_->gantryCmd.isAutoCtrl = true;

  /* Step 1 */
  core.gimbal_->gimbalCmd.setPosit_Lift = 240.0f;
  core.gimbal_->gimbalCmd.setStep_Pitch = 0;
  core.gantry_->gantryCmd.setPosit_Lift = 290.0f;
  core.gantry_->gantryCmd.setPosit_Stretch = 120.0f;
  core.gantry_->gantryCmd.setPosit_Traverse = 0.0f;
  proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Stretch);
  core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
  proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Joint_Yaw);
  core.gantry_->gantryCmd.setAngle_Joint_Roll = 0.0f;
  core.gantry_->gantryCmd.setAngle_End_Pitch = 0.0f;
  core.gantry_->gantryCmd.setAngle_End_Roll = 0.0f;
  proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Joint_Roll);
  core.gantry_->gantryCmd.setPumpOn = true;

  /* Wait for User Confirmation */
  proc_waitMs(500);
  cnt = timeout;
  core.gantry_->gantryCmd.isAutoCtrl = false;      // Unlock Gantry
  while (cnt--) {
    if (SysRemote.remoteInfo.keyboard.key_Ctrl) {
      if (SysRemote.remoteInfo.keyboard.mouse_L) break;
      if (SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
    }
    proc_waitMs(5);
  }
  if (cnt == 0) goto proc_exit;

  /* Step 2 */
  core.gantry_->gantryCmd.setPumpOn = true;
  core.gantry_->gantryCmd.setPosit_Lift += 200.0f;
  core.gantry_->gantryCmd.setPosit_Stretch =
    core.gantry_->gantryInfo.posit_Stretch - 40.0f;

  /* Process Exit */
proc_exit:
  core.gimbal_->gimbalCmd.isAutoCtrl = false;
  core.gantry_->gantryCmd.isAutoCtrl = false;
  core.autoCtrlTaskHandle_           = nullptr;
  core.currentAutoCtrlProcess_       = EAutoCtrlProcess::NONE;
  proc_return();
}

} // namespace robotpilots
