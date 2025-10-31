/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-03-30
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-30   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "Core.hpp"

namespace robotpilots {

/**
 * @brief
 */
void CSystemCore::ControlFromRemote_() {

  enum { HIG = 1, LOW = 2, MID = 3 };
  const auto freq = 1000.0f;
  auto &remote = SysRemote.remoteInfo.remote;

  /* Module Start */
  if (SysRemote.systemState == RP_OK) {

    if (!chassis_->chassisInfo.isModuleAvailable
        && chassis_->moduleState == RP_OK)
      chassis_->StartModule();

    if (!gimbal_->gimbalInfo.isModuleAvailable
        && gimbal_->moduleState == RP_OK)
      gimbal_->StartModule();

    if (!gantry_->gantryInfo.isModuleAvailable
        && gantry_->moduleState == RP_OK
        && remote.switch_L == HIG)
      gantry_->StartModule();

    if (!subgantry_->subGantryInfo.isModuleAvailable
        && subgantry_->moduleState == RP_OK
        && remote.switch_L == HIG)
      subgantry_->StartModule();
  }

  /* LOW + MID Chassis Control */
  if (remote.switch_L == LOW && remote.switch_R == MID) {
    chassis_->chassisCmd.speed_X = remote.joystick_LX/2;  // Limit -50%~50%
    chassis_->chassisCmd.speed_Y = remote.joystick_LY;
    chassis_->chassisCmd.speed_W = remote.joystick_RX;
    gimbal_->gimbalCmd.setPosit_Lift +=
      (remote.joystick_RY / 100.0f) * 100.0f / freq;
  }

  /* MID + HIG Gantry Control */
  if (remote.switch_L == MID && remote.switch_R == HIG) {
    gantry_->gantryCmd.setPosit_Lift +=
      (remote.joystick_LY / 100.0f) * 200.0f / freq;
    gantry_->gantryCmd.setPosit_Traverse +=
      (remote.joystick_RX / 100.0f) * 100.0f / freq;
    gantry_->gantryCmd.setPosit_Stretch +=
      (remote.joystick_RY / 100.0f) * 200.0f / freq;
  }

  /* MID + MID Manipulator Control */
  if (remote.switch_L == MID && remote.switch_R == MID) {
    gantry_->gantryCmd.setAngle_Joint_Yaw +=
      (remote.joystick_LX / 100.0f) * -90.0f / freq;
    gantry_->gantryCmd.setAngle_Joint_Roll +=
      (remote.joystick_RX / 100.0f) * 90.0f / freq;
    gantry_->gantryCmd.setAngle_End_Pitch +=
      (remote.joystick_LY / 100.0f) * 90.0f / freq;
    gantry_->gantryCmd.setAngle_End_Roll +=
      (remote.joystick_RY / 100.0f) * 90.0f / freq;
  }

  /* MID + LOW Sub-Gantry Control */
  if (remote.switch_L == MID && remote.switch_R == LOW) {
    subgantry_->subGantryCmd.setPosit_Lift_L +=
      (remote.joystick_LX / 100.0f) * 80.0f / freq;
    subgantry_->subGantryCmd.setPosit_Lift_R +=
      (remote.joystick_RX / 100.0f) * 80.0f / freq;
    subgantry_->subGantryCmd.setPosit_Stretch_L +=
      (remote.joystick_LY / 100.0f) * 150.0f / freq;
    subgantry_->subGantryCmd.setPosit_Stretch_R +=
      (remote.joystick_RY / 100.0f) * 150.0f / freq;
  }

}


/**
 * @brief
 */
void CSystemCore::ControlFromKeyboard_() {

  const auto freq = 1000.0f;
  auto &keyboard  = SysRemote.remoteInfo.keyboard;
  static bool lastMouseState_L = false, lastMouseState_R = false;

  /* Module Start */
  if (SysRemote.systemState == RP_OK) {

    if (!chassis_->chassisInfo.isModuleAvailable && chassis_->moduleState == RP_OK)
      chassis_->StartModule();

    if (!gimbal_->gimbalInfo.isModuleAvailable && gimbal_->moduleState == RP_OK)
      gimbal_->StartModule();

    if (!gantry_->gantryInfo.isModuleAvailable && gantry_->moduleState == RP_OK && keyboard.key_Ctrl && keyboard.key_R)
      gantry_->StartModule();

    if (!subgantry_->subGantryInfo.isModuleAvailable && subgantry_->moduleState == RP_OK && keyboard.key_Ctrl && keyboard.key_R)
      subgantry_->StartModule();
  }

  /* Chassis Control */
  chassis_->chassisCmd.speed_X *= 0.97f;
  chassis_->chassisCmd.speed_Y *= 0.98f;
  if (abs(chassis_->chassisCmd.speed_X) < 0.5f) chassis_->chassisCmd.speed_X = 0.0f;
  if (abs(chassis_->chassisCmd.speed_Y) < 0.5f) chassis_->chassisCmd.speed_Y = 0.0f;

  chassis_->chassisCmd.speed_W = chassis_->chassisCmd.speed_W + 0.03f*(keyboard.mouse_X - chassis_->chassisCmd.speed_W);
  if (keyboard.key_Shift) {
    chassis_->chassisCmd.speed_X += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 5.0f;
    chassis_->chassisCmd.speed_Y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 5.0f;
    chassis_->chassisCmd.speed_X =
      std::clamp(chassis_->chassisCmd.speed_X, -50.0f, 50.0f);
    chassis_->chassisCmd.speed_Y =
      std::clamp(chassis_->chassisCmd.speed_Y, -100.0f, 100.0f);
  } else {
    chassis_->chassisCmd.speed_X += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 0.8f;
    chassis_->chassisCmd.speed_Y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 0.8f;
    chassis_->chassisCmd.speed_X =
      std::clamp(chassis_->chassisCmd.speed_X, -20.0f, 20.0f);
    chassis_->chassisCmd.speed_Y =
      std::clamp(chassis_->chassisCmd.speed_Y, -30.0f, 30.0f);
  }

  if (!keyboard.key_Ctrl
      && keyboard.key_G
      && currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE) {
    chassis_->chassisCmd.speed_W = static_cast<float_t>((keyboard.mouse_R - keyboard.mouse_L)) * 100.0f;
  }

  /* Gantry Manual Control */
  if (!keyboard.key_Ctrl
      && !gantry_->gantryCmd.isAutoCtrl) {
    if (keyboard.key_Q)
      gantry_->gantryCmd.setPosit_Lift += static_cast<float_t>((keyboard.mouse_L - keyboard.mouse_R)) * 150.0f / freq;
    if (keyboard.key_E)
      gantry_->gantryCmd.setPosit_Stretch += static_cast<float_t>((keyboard.mouse_L - keyboard.mouse_R)) * 150.0f / freq;
    if (keyboard.key_R)
      gantry_->gantryCmd.setPosit_Traverse -= static_cast<float_t>((keyboard.mouse_L - keyboard.mouse_R)) * 100.0f / freq;
    if (keyboard.key_Z)
      gantry_->gantryCmd.setAngle_Joint_Yaw += static_cast<float_t>((keyboard.mouse_L - keyboard.mouse_R)) * 90.0f / freq;
    if (keyboard.key_X)
      gantry_->gantryCmd.setAngle_Joint_Roll += static_cast<float_t>((keyboard.mouse_L - keyboard.mouse_R)) * 90.0f / freq;
    if (keyboard.key_C)
      gantry_->gantryCmd.setAngle_End_Pitch += static_cast<float_t>((keyboard.mouse_L - keyboard.mouse_R)) * 90.0f / freq;
    if (keyboard.key_V)
      gantry_->gantryCmd.setAngle_End_Roll += static_cast<float_t>((keyboard.mouse_L - keyboard.mouse_R)) * 90.0f / freq;
    if (keyboard.key_B) {
      if (!lastMouseState_L && keyboard.mouse_L) gantry_->gantryCmd.setPumpOn_C = 1 - gantry_->gantryCmd.setPumpOn_C;
      if (!lastMouseState_R && keyboard.mouse_R) {
        if (gantry_->gantryCmd.setPumpOn_L || gantry_->gantryCmd.setPumpOn_R) {
          gantry_->gantryCmd.setPumpOn_L = false;
          gantry_->gantryCmd.setPumpOn_R = false;
        } else {
          gantry_->gantryCmd.setPumpOn_L = true;
          gantry_->gantryCmd.setPumpOn_R = true;
        }
      }
    }
  }

  /* Gimbal Manual Control */
  if (!keyboard.key_Ctrl
      && !gimbal_->gimbalCmd.isAutoCtrl) {

    if (keyboard.key_F)
      gimbal_->gimbalCmd.setPosit_Lift += static_cast<float_t>((keyboard.mouse_L - keyboard.mouse_R)) * 120.0f / freq;
  }

  /* Gantry Auto Control */
  if (keyboard.key_Ctrl
//      && gimbal_->gimbalInfo.isModuleAvailable
      && gantry_->gantryInfo.isModuleAvailable
      && subgantry_->subGantryInfo.isModuleAvailable) {

    if (keyboard.key_R) {
      StopAutoCtrlTask_();
      StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_ORIGIN);
    }

    if (keyboard.key_Z)
      StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_DRIVE);

    if (keyboard.key_X)
      StartAutoCtrlTask_(EAutoCtrlProcess::GROUND_ORE);

    if (keyboard.key_C)
      StartAutoCtrlTask_(EAutoCtrlProcess::SILVER_ORE);

    if (keyboard.key_V)
      StartAutoCtrlTask_(EAutoCtrlProcess::GOLD_ORE);

    if (keyboard.key_B)
      StartAutoCtrlTask_(EAutoCtrlProcess::EXCHANGE);

    if (keyboard.key_F)
      StartAutoCtrlTask_(EAutoCtrlProcess::PUSH_ORE);

    if (keyboard.key_G)
      StartAutoCtrlTask_(EAutoCtrlProcess::POP_ORE);

//    if (keyboard.mouse_R)
//      StopAutoCtrlTask_();
  }

  lastMouseState_L = keyboard.mouse_L;
  lastMouseState_R = keyboard.mouse_R;
}

} // namespace robotpilots
