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

#include "mod_chassis.hpp"

namespace robotpilots {

/**
 * @brief
 * @param arg
 */
void CModChassis::StartChassisModuleTask(void *arg) {

  if (arg == nullptr) proc_return();

  /* Get Chassis Module Handle */
  auto &chassis = *static_cast<CModChassis *>(arg);

  /* FSM Status Enum */
  enum {
    CHASSIS_RESET = 0,  // Set From StopModule() Function
    CHASSIS_INIT  = 1,  // Set From StartModule() Function
    CHASSIS_CTRL,
  };

  /* Chassis Module Task */
  while (true) {

    /* FSM */
    switch (chassis.processFlag_) {

      case CHASSIS_RESET: {

        chassis.chassisInfo.isModuleAvailable = false;
        chassis.comWheelset_.StopComponent();

        proc_waitMs(20);  // 50Hz
        continue;
      }

      case CHASSIS_INIT: {

        chassis.comWheelset_.StartComponent();

        chassis.chassisCmd = SChassisCmd();
        chassis.chassisInfo.isModuleAvailable = true;
        chassis.processFlag_ = CHASSIS_CTRL;    // Enter Control Mode
        chassis.moduleState = RP_OK;
        continue;
      }

      case CHASSIS_CTRL: {

        chassis.RestrictChassisCommand_();

        chassis.comWheelset_.wheelsetCmd.speed_Y
          = chassis.chassisCmd.speed_Y * 80;
        chassis.comWheelset_.wheelsetCmd.speed_X
          = chassis.chassisCmd.speed_X * 80;
        chassis.comWheelset_.wheelsetCmd.speed_W
          = chassis.chassisCmd.speed_W * 40;

        proc_waitMs(1);   // 1000Hz
        continue;
      }

      default: { chassis.StopModule(); }
    }
  }

  /* Task Exit */
  chassis.moduleTaskHandle = nullptr;
  proc_return();
}

} // namespace robotpilots