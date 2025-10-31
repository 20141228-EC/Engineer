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

void CSystemCore::StartPopOreTask(void *arg) {

  if (arg == nullptr) proc_return();

  /* Get System Core Handle */
  auto &core = *reinterpret_cast<CSystemCore *>(arg);
  auto cnt = 0;
  const auto timeout = 60000 / 5;    // unit: ms

  /* Set Auto Control Flag */
  core.gantry_->gantryCmd.isAutoCtrl = true;
  core.subgantry_->subGantryCmd.isAutoCtrl = true;

  while (SysRemote.remoteInfo.keyboard.key_Ctrl) {

    /* Pop From Left */
    if (SysRemote.remoteInfo.keyboard.mouse_L) {

      /* Step 1 */
      core.gimbal_->gimbalCmd.setPosit_Lift = 150.0f;
      core.gimbal_->gimbalCmd.setStep_Pitch = 0;
      core.subgantry_->subGantryCmd.setPosit_Stretch_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_R = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_R = 0.0f;
      core.gantry_->gantryCmd.setPumpOn_C = false;
      core.gantry_->gantryCmd.setPosit_Lift = 30.0f;
      core.gantry_->gantryCmd.setPosit_Traverse = 190.0f / 2;
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Roll = 90.0f;
      proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Joint_Yaw);
      core.gantry_->gantryCmd.setPosit_Stretch = 410.0;
      core.gantry_->gantryCmd.setPosit_Traverse = 10.0f;
      core.gantry_->gantryCmd.setAngle_End_Pitch = -90.0f;
      core.gantry_->gantryCmd.setAngle_End_Roll = 0.0f;
      proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Lift
                     && core.gantry_->gantryInfo.isPositArrived_Stretch
                     && core.gantry_->gantryInfo.isPositArrived_Traverse);
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = 90.0f;
      proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Joint_Yaw);

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
      core.gantry_->gantryCmd.isAutoCtrl = true;
      core.gantry_->gantryCmd.setPumpOn_C = true;
      while (core.gantry_->gantryInfo.posit_Stretch > 360.0f) {
        core.gantry_->gantryCmd.setPosit_Stretch -= 100.0f / core.freq;
        proc_waitMs(1);
      }
      proc_waitMs(500);
      core.gantry_->gantryCmd.setPumpOn_L = false;
      proc_waitMs(500);
      core.gantry_->gantryCmd.setPosit_Stretch = 410.0;
//      core.gantry_->gantryCmd.setAngle_End_Roll -= 90.0f;

      break;
    }

    /* Pop From Right */
    if (SysRemote.remoteInfo.keyboard.mouse_R) {

      /* Step 1 */
      core.gimbal_->gimbalCmd.setPosit_Lift = 150.0f;
      core.gimbal_->gimbalCmd.setStep_Pitch = 0;
      core.subgantry_->subGantryCmd.setPosit_Stretch_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_R = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_R = 0.0f;
      core.gantry_->gantryCmd.setPumpOn_C = false;
      core.gantry_->gantryCmd.setPosit_Lift = 30.0f;
      core.gantry_->gantryCmd.setPosit_Traverse = 190.0f / 2;
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Roll = -90.0f;
      proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Joint_Yaw);
      core.gantry_->gantryCmd.setPosit_Stretch = 410.0;
      core.gantry_->gantryCmd.setPosit_Traverse = 180.0f;
      core.gantry_->gantryCmd.setAngle_End_Pitch = -90.0f;
      core.gantry_->gantryCmd.setAngle_End_Roll = 0.0f;
      proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Lift
                     && core.gantry_->gantryInfo.isPositArrived_Stretch
                     && core.gantry_->gantryInfo.isPositArrived_Traverse);
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = -90.0f;

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
      core.gantry_->gantryCmd.isAutoCtrl = true;
      core.gantry_->gantryCmd.setPumpOn_C = true;
      while (core.gantry_->gantryInfo.posit_Stretch > 360.0f) {
        core.gantry_->gantryCmd.setPosit_Stretch -= 100.0f / core.freq;
        proc_waitMs(1);
      }
      proc_waitMs(500);
      core.gantry_->gantryCmd.setPumpOn_R = false;
      proc_waitMs(500);
      core.gantry_->gantryCmd.setPosit_Stretch = 410.0;
//      core.gantry_->gantryCmd.setAngle_End_Roll += 90.0f;
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
