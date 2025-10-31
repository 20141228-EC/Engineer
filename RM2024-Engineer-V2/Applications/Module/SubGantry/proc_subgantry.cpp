/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-10
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-10   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "mod_subgantry.hpp"

namespace robotpilots {

/**
 * @brief
 * @param arg
 */
void CModSubGantry::StartSubGantryModuleTask(void *arg) {

  if (arg == nullptr) proc_return();

  /* Get SubGantry Module Handle */
  auto &subGantry = *static_cast<CModSubGantry *>(arg);

  /* FSM State Enum */
  enum {
    SUBGANTRY_RESET = 0,
    SUBGANTRY_INIT  = 1,
    SUBGANTRY_CTRL,
  };

  /* SubGantry Module Task */
  while (true) {

    /* SubGantry Module Control FSM */
    switch (subGantry.processFlag_) {

      case SUBGANTRY_RESET: {

        subGantry.subGantryInfo.isModuleAvailable = false;
        subGantry.comLift_.StopComponent();
        subGantry.comStretch_.StopComponent();

        proc_waitMs(20);  // 50Hz
        break;
      }

      case SUBGANTRY_INIT: {

        proc_waitMs(250);

        subGantry.comLift_.StartComponent();
        subGantry.comStretch_.StartComponent();
        proc_waitUntil(subGantry.comLift_.componentState == RP_OK
                       && subGantry.comStretch_.componentState == RP_OK);

        subGantry.subGantryCmd = SSubGantryCmd();
        subGantry.subGantryInfo.isModuleAvailable = true;
        subGantry.processFlag_ = SUBGANTRY_CTRL;
        subGantry.moduleState = RP_OK;
        break;
      }

      case SUBGANTRY_CTRL: {

        subGantry.RestrictGantryCommand_();

        subGantry.comLift_.liftCmd.setPosit_L =
          CComLift::PhyPositToMtrPosit(subGantry.subGantryCmd.setPosit_Lift_L);
        subGantry.comLift_.liftCmd.setPosit_R =
          CComLift::PhyPositToMtrPosit(subGantry.subGantryCmd.setPosit_Lift_R);
        subGantry.comStretch_.stretchCmd.setPosit_L =
          CComStretch::PhyPositToMtrPosit(subGantry.subGantryCmd.setPosit_Stretch_L);
        subGantry.comStretch_.stretchCmd.setPosit_R =
          CComStretch::PhyPositToMtrPosit(subGantry.subGantryCmd.setPosit_Stretch_R);

        proc_waitMs(1);  // 1000Hz
        break;
      }

      default: { subGantry.StopModule(); }
    }
  }

  /* Task Exit */
  subGantry.moduleTaskHandle = nullptr;
  proc_return();
}

} // namespace robotpilots
