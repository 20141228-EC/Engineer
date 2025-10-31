/**
 * @file        proc_gantry.cpp
 * @version     1.0
 * @date        2024-05-10
 * @author      Morthine Xiang
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-10   <td>1.0         <td>Morthine Xiang  <td>First Create.
 * </table>
 */

#include "mod_gantry.hpp"

namespace robotpilots {

/**
 * @brief
 * @param arg
 */
void CModGantry::StartGantryModuleTask(void *arg) {

  if (arg == nullptr) proc_return();

  /* Get Gantry Module Handle */
  auto &gantry = *static_cast<CModGantry *>(arg);

  /* FSM State Enum */
  enum {
    GANTRY_RESET = 0,
    GANTRY_INIT  = 1,
    GANTRY_CTRL,
  };

  /* Gantry Module Task */
  while (true) {

    /* FSM */
    switch (gantry.processFlag_) {

      case GANTRY_RESET: {

        gantry.gantryInfo.isModuleAvailable = false;
        gantry.comPump_.StopComponent();
        gantry.comLift_.StopComponent();
        gantry.comStretch_.StopComponent();
        gantry.comTraverse_.StopComponent();
        gantry.comJointYaw_.StopComponent();
        gantry.comJointRoll_.StopComponent();
        gantry.comEnd_.StopComponent();
        gantry.comFlip_.StopComponent();

        proc_waitMs(20);  // 50Hz
        continue;
      }

      case GANTRY_INIT: {

        gantry.gantryCmd = SGantryCmd();        // Reset Gantry Command

        gantry.comFlip_.StartComponent();
        gantry.comPump_.StartComponent();
        proc_waitUntil(gantry.comFlip_.componentState == RP_OK);
        gantry.comFlip_.flipCmd.setSpeed_Y = 100;

        proc_waitMs(250);   // Wait 250ms for Stable
        gantry.comLift_.StartComponent();

        proc_waitUntil(gantry.comLift_.componentState == RP_OK);
        gantry.comFlip_.flipCmd.setSpeed_Y = 0;
        gantry.comStretch_.StartComponent();

        proc_waitUntil(gantry.comStretch_.componentState == RP_OK);
        gantry.comStretch_.stretchCmd.setPosit = 300000;

        proc_waitUntil(gantry.comStretch_.stretchInfo.isPositArrived);
        gantry.comTraverse_.StartComponent();
        gantry.comJointYaw_.StartComponent();

        proc_waitMs(500);
        gantry.comJointRoll_.StartComponent();

        proc_waitUntil(gantry.comTraverse_.componentState == RP_OK);
        gantry.comTraverse_.traverseCmd.setPosit = 330000;

        proc_waitUntil(gantry.comJointRoll_.componentState == RP_OK);
        gantry.comJointRoll_.jointRollCmd.setPosit = 96748;

        proc_waitUntil(gantry.comJointYaw_.componentState == RP_OK);
        gantry.comJointYaw_.jointYawCmd.setPosit = 446970;
        gantry.comEnd_.StartComponent();

        proc_waitUntil(gantry.comJointYaw_.jointYawInfo.isPositArrived);
        gantry.comStretch_.stretchCmd.setPosit = 0;

        proc_waitUntil(gantry.comEnd_.componentState == RP_OK);
        gantry.comEnd_.endCmd.setPosit_Pitch = 41753;

        proc_waitUntil(gantry.comStretch_.stretchInfo.isPositArrived);

        gantry.gantryInfo.isModuleAvailable = true;
        gantry.processFlag_ = GANTRY_CTRL;      // Enter Control Mode
        gantry.moduleState = RP_OK;
        continue;
      }

      case GANTRY_CTRL: {

        gantry.RestrictGantryCommand_();

        gantry.comPump_.pumpCmd.setPumpOn = gantry.gantryCmd.setPumpOn;
        gantry.comLift_.liftCmd.setPosit =
          CComLift::PhyPositToMtrPosit(gantry.gantryCmd.setPosit_Lift);
        gantry.comStretch_.stretchCmd.setPosit =
          CComStretch::PhyPositToMtrPosit(gantry.gantryCmd.setPosit_Stretch);
        gantry.comTraverse_.traverseCmd.setPosit =
          CComTraverse::PhyPositToMtrPosit(gantry.gantryCmd.setPosit_Traverse);
        gantry.comJointYaw_.jointYawCmd.setPosit =
          CComJointYaw::PhyPositToMtrPosit(gantry.gantryCmd.setAngle_Joint_Yaw);
        gantry.comJointRoll_.jointRollCmd.setPosit =
          CComJointRoll::PhyPositToMtrPosit(gantry.gantryCmd.setAngle_Joint_Roll);
        gantry.comEnd_.endCmd.setPosit_Pitch =
          CComEnd::PhyPositToMtrPosit_Pitch(gantry.gantryCmd.setAngle_End_Pitch);
        gantry.comEnd_.endCmd.setPosit_Roll =
          CComEnd::PhyPositToMtrPosit_Roll(gantry.gantryCmd.setAngle_End_Roll);
        gantry.comFlip_.flipCmd.setSpeed_Y = gantry.gantryCmd.setSpeed_Flip_Y;
        gantry.comFlip_.flipCmd.setSpeed_W = gantry.gantryCmd.setSpeed_Flip_W;

        proc_waitMs(1);   // 1000Hz
        continue;
      }

      default: { gantry.StopModule(); }
    }
  }

  /* Task Exit */
  gantry.moduleTaskHandle = nullptr;
  proc_return();
}

} // namespace robotpilots
