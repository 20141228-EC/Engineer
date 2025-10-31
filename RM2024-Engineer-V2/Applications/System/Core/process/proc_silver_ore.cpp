/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-18
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-18   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "Core.hpp"

namespace robotpilots {

void CSystemCore::StartSilverOreTask(void *arg) {

  if (arg == nullptr) proc_return();

  /* Get System Core Handle */
  auto &core = *reinterpret_cast<CSystemCore *>(arg);
  const auto timeout = 60000 / 5;    // unit: ms
  auto cnt = 0;

  /* Set Auto Control Flag */
  core.gantry_->gantryCmd.isAutoCtrl = true;
  core.subgantry_->subGantryCmd.isAutoCtrl = true;

  while (SysRemote.remoteInfo.keyboard.key_Ctrl) {

    /* Get Single Silver Ore */
    if (SysRemote.remoteInfo.keyboard.mouse_R) {

      /* Set Gimbal */
      core.gimbal_->gimbalCmd.setPosit_Lift = 280.0f;
      core.gimbal_->gimbalCmd.setStep_Pitch = 0;

      /* Set Sub-Gantry */
      core.subgantry_->subGantryCmd.setPosit_Lift_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_R = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_R = 0.0f;

      /* Step 1 */
      core.gantry_->gantryCmd.setPumpOn_C = true;
      core.gantry_->gantryCmd.setPosit_Lift = 320.0f;
      core.gantry_->gantryCmd.setPosit_Stretch = 100.0f;
      core.gantry_->gantryCmd.setPosit_Traverse = 190.0f / 2;
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Roll = 0.0f;
      core.gantry_->gantryCmd.setAngle_End_Pitch = 0.0f;
      core.gantry_->gantryCmd.setAngle_End_Roll = 0.0f;
      proc_waitMs(500);

      /* Wait for User Confirmation */
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
      core.gantry_->gantryCmd.isAutoCtrl = true;      // Lock Gantry
      core.gantry_->gantryCmd.setPumpOn_C = true;
      core.gantry_->gantryCmd.setAngle_End_Pitch = 3.0f;
      core.gantry_->gantryCmd.setPosit_Stretch =
        core.gantry_->gantryInfo.posit_Stretch - 40.0f;
      proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Stretch);
      core.gantry_->gantryCmd.setPosit_Lift += 200.0f;
      proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Lift);

      break;
    }

    /* Get Triple Silver Ore */
    if (SysRemote.remoteInfo.keyboard.mouse_L) {

      /* Set Gimbal */
      core.gimbal_->gimbalCmd.setPosit_Lift = 280.0f;
      core.gimbal_->gimbalCmd.setStep_Pitch = 0;

      /* Step 1 */
      core.gantry_->gantryCmd.setPumpOn_C = true;
      core.gantry_->gantryCmd.setPosit_Lift = 330.0f;
      core.gantry_->gantryCmd.setPosit_Stretch = 60.0f;
      core.gantry_->gantryCmd.setPosit_Traverse = 190.0f / 2;
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Roll = 0.0f;
      core.gantry_->gantryCmd.setAngle_End_Pitch = 0.0f;
      core.gantry_->gantryCmd.setAngle_End_Roll = 0.0f;
      core.gantry_->gantryCmd.setPumpOn_L = true;
      core.gantry_->gantryCmd.setPumpOn_R = true;
      core.subgantry_->subGantryCmd.setPosit_Lift_L = 180.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_R = 180.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_L = 270.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_R = 270.0f;
      proc_waitMs(500);

      /* Wait for User Confirmation */
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
      core.gantry_->gantryCmd.isAutoCtrl = true;      // Lock Gantry
      core.gimbal_->gimbalCmd.setPosit_Lift = 150.0f;
      core.gimbal_->gimbalCmd.setStep_Pitch = 0;
      core.gantry_->gantryCmd.setPumpOn_L = true;
      core.gantry_->gantryCmd.setPumpOn_C = true;
      core.gantry_->gantryCmd.setPumpOn_R = true;
      core.gantry_->gantryCmd.setAngle_End_Pitch = 5.0f;
      core.gantry_->gantryCmd.setPosit_Stretch =
        core.gantry_->gantryInfo.posit_Stretch - 30.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_L =
        core.subgantry_->subGantryInfo.posit_Stretch_L - 20.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_R =
        core.subgantry_->subGantryInfo.posit_Stretch_R - 20.0f;
//      proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Stretch
//                     && core.subgantry_->subGantryInfo.isPositArrived_Stretch_L
//                     && core.subgantry_->subGantryInfo.isPositArrived_Stretch_R);
      core.gantry_->gantryCmd.setPosit_Lift += 200.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_L = 295.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_R = 295.0f;
      proc_waitMs(500);

      break;
    }

    proc_waitMs(5);
  }

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