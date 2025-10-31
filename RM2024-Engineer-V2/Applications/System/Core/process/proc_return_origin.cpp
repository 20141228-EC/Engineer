/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-19
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-19   <td>1.0         <td>qianchen  <td>First Create.
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
  auto &core = *reinterpret_cast<CSystemCore *>(arg);

  proc_waitMs(500);

  /* Set Auto Control Flag */
  core.gantry_->gantryCmd.isAutoCtrl = true;
  core.subgantry_->subGantryCmd.isAutoCtrl = true;

  /* Gimbal Return Origin */
  core.gimbal_->gimbalCmd.setPosit_Lift = 150.0f;
  core.gimbal_->gimbalCmd.setStep_Pitch = 0;

  /* Sub-Gantry Return Origin */
  core.subgantry_->subGantryCmd.setPosit_Lift_L = 0.0f;
  core.subgantry_->subGantryCmd.setPosit_Lift_R = 0.0f;
  core.subgantry_->subGantryCmd.setPosit_Stretch_L = 0.0f;
  core.subgantry_->subGantryCmd.setPosit_Stretch_R = 0.0f;

  /* Gantry Return Origin */
  core.gantry_->gantryCmd.setPumpOn_L = false;
  core.gantry_->gantryCmd.setPumpOn_C = false;
  core.gantry_->gantryCmd.setPumpOn_R = false;
  proc_waitMs(500);
  core.gantry_->gantryCmd.setPosit_Lift = 0.0f;
  core.gantry_->gantryCmd.setPosit_Traverse = 0.0;
  core.gantry_->gantryCmd.setAngle_Joint_Roll = 0.0f;
  core.gantry_->gantryCmd.setAngle_End_Pitch = 0.0f;

  if (core.gantry_->gantryInfo.posit_Traverse < 100.0f
      && core.gantry_->gantryInfo.angle_Joint_Yaw > 0.0f) {
    core.gantry_->gantryCmd.setAngle_Joint_Yaw = -90.0f;
    core.gantry_->gantryCmd.setPosit_Stretch = 0.0f;
  } else {
    core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
    core.gantry_->gantryCmd.setPosit_Stretch = 100.0f;
    proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Traverse);
    core.gantry_->gantryCmd.setAngle_Joint_Yaw = -90.0f;
    core.gantry_->gantryCmd.setPosit_Stretch = 0.0f;
  }
  proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_End_Pitch);
  core.gantry_->gantryCmd.setAngle_End_Roll = 0.0f;

  /* Process Exit */
  core.gimbal_->gimbalCmd.isAutoCtrl = false;
  core.gantry_->gantryCmd.isAutoCtrl = false;
  core.subgantry_->subGantryCmd.isAutoCtrl = false;
  core.autoCtrlTaskHandle_ = nullptr;
  core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
  proc_return();
}

} // namespace robotpilots