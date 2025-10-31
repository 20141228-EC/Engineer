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
        gantry.comJoint_.StopComponent();
        gantry.comEnd_.StopComponent();

        proc_waitMs(20);  // 50Hz
        break;
      }

      case GANTRY_INIT: {

        proc_waitMs(250);   // Wait 250ms for Stable

        gantry.comPump_.StartComponent();       // Start Pump Component
        gantry.comLift_.StartComponent();       // Start Lift Component
        proc_waitUntil(gantry.comLift_.componentState == RP_OK);
        gantry.comLift_.liftCmd.setPosit = 8192 * 3;
        gantry.comStretch_.StartComponent();    // Start Stretch Component
        gantry.comTraverse_.StartComponent();   // Start Traverse Component
        gantry.comEnd_.StartComponent();        // Start End Component
        proc_waitUntil(gantry.comStretch_.componentState == RP_OK
                       && gantry.comTraverse_.componentState == RP_OK
                       && gantry.comEnd_.componentState == RP_OK);

        gantry.comJoint_.StartComponent();      // Start Joint Component
        proc_waitUntil(gantry.comJoint_.componentState == RP_OK);

        gantry.gantryCmd = SGantryCmd();        // Reset Gantry Command
        gantry.gantryInfo.isModuleAvailable = true;
        gantry.processFlag_ = GANTRY_CTRL;      // Enter Control Mode
        gantry.moduleState = RP_OK;
        break;
      }

      case GANTRY_CTRL: {

        gantry.RestrictGantryCommand_();

        gantry.comPump_.pumpCmd.setPumpOn_L = gantry.gantryCmd.setPumpOn_L;
        gantry.comPump_.pumpCmd.setPumpOn_C = gantry.gantryCmd.setPumpOn_C;
        gantry.comPump_.pumpCmd.setPumpOn_R = gantry.gantryCmd.setPumpOn_R;
        gantry.comLift_.liftCmd.setPosit =
          CComLift::PhyPositToMtrPosit(gantry.gantryCmd.setPosit_Lift);
        gantry.comStretch_.stretchCmd.setPosit =
          CComStretch::PhyPositToMtrPosit(gantry.gantryCmd.setPosit_Stretch);
        gantry.comTraverse_.traverseCmd.setPosit =
          CComTraverse::PhyPositToMtrPosit(gantry.gantryCmd.setPosit_Traverse);
        gantry.comJoint_.jointCmd.setPosit_Yaw =
          CComJoint::PhyPositToMtrPosit_Yaw(gantry.gantryCmd.setAngle_Joint_Yaw);
        gantry.comJoint_.jointCmd.setPosit_Roll =
          CComJoint::PhyPositToMtrPosit_Roll(gantry.gantryCmd.setAngle_Joint_Roll);
        gantry.comEnd_.endCmd.setPosit_Pitch =
          CComEnd::PhyPositToMtrPosit_Pitch(gantry.gantryCmd.setAngle_End_Pitch);
        gantry.comEnd_.endCmd.setPosit_Roll =
          CComEnd::PhyPositToMtrPosit_Roll(gantry.gantryCmd.setAngle_End_Roll);

        proc_waitMs(1);   // 1000Hz
        break;
      }

      default: { gantry.StopModule(); }
    }
  }

  /* Task Exit */
  gantry.moduleTaskHandle = nullptr;
  proc_return();
}

} // namespace robotpilots
