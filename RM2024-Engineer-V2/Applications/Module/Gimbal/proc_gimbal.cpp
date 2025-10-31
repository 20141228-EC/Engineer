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

#include "mod_gimbal.hpp"

namespace robotpilots {

void CModGimbal::StartGimbalModuleTask(void *arg) {

  if (arg == nullptr) proc_return();

  /* Get Gimbal Module Handle */
  auto &gimbal = *static_cast<CModGimbal *>(arg);

  /* FSM Status Enum */
  enum {
    GIMBAL_RESET = 0,   // Set From StopModule() Function
    GIMBAL_INIT  = 1,   // Set From StartModule() Function
    GIMBAL_CTRL,
  };

  /* Gimbal Module Task */
  while (true) {

    /* Gimbal Module Control FSM */
    switch (gimbal.processFlag_) {

      case GIMBAL_RESET: {

        gimbal.gimbalInfo.isModuleAvailable = false;
        gimbal.comLift_.StopComponent();
        gimbal.comPitch_.StopComponent();

        proc_waitMs(20);  // 50Hz
        break;
      }

      case GIMBAL_INIT: {

        gimbal.comLift_.StartComponent();
        gimbal.comPitch_.StartComponent();
        proc_waitUntil(gimbal.comLift_.componentState == RP_OK);

        gimbal.gimbalCmd = SGimbalCmd();
        gimbal.gimbalInfo.isModuleAvailable = true;
        gimbal.processFlag_ = GIMBAL_CTRL;
        gimbal.moduleState = RP_OK;
        break;
      }

      case GIMBAL_CTRL: {

        gimbal.RestrictGimbalCommand_();

        gimbal.comPitch_.pitchCmd.setStep = gimbal.gimbalCmd.setStep_Pitch;
        gimbal.comLift_.liftCmd.setPosit =
          CComLift::PhyPositToMtrPosit(gimbal.gimbalCmd.setPosit_Lift);

        proc_waitMs(1);   // 1000Hz
        break;
      }

      default: { gimbal.StopModule(); }
    }
  }

  /* Task Exit */
  gimbal.moduleTaskHandle = nullptr;
  proc_return();
}

} // namespace robotpilots
