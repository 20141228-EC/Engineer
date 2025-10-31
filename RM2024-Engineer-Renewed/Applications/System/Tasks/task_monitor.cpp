/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-25
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-25   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "Core.hpp"

namespace robotpilots {

/**
 * @brief Monitor Task Entrypoint
 * @param arg[in] Not Used
 */
void StartMonitorTask(void *arg) {

//  auto *uart = reinterpret_cast<CUartInterface *>(InterfaceMap.at(EInterfaceID::INF_UART10));
//  auto *vision = reinterpret_cast<CSysVision *>(SystemMap.at(ESystemID::SYS_VISION));

  /* Monitor Task */
  while (true) {

//    uart->FormatTransmit("Robot ID: %d, Camp ID: %d\n",
//                         SysReferee.refereeInfo.robot.robotID,
//                         SysReferee.refereeInfo.robot.robotCamp);
//
//    if (SysVision.systemState != RP_OK) {
//      uart->FormatTransmit("Vision System Error!\n");
//    } else {
//      if (vision->visionInfo.oreTank.isFoundOreTank) {
//        uart->FormatTransmit("YPR: %d, %d, %d\n",
//                             static_cast<int32_t>(vision->visionInfo.oreTank.atti_YAW),
//                             static_cast<int32_t>(vision->visionInfo.oreTank.atti_PITCH),
//                             static_cast<int32_t>(vision->visionInfo.oreTank.atti_ROLL));
//        uart->FormatTransmit("XYZ: %d, %d, %d\n",
//                             static_cast<int32_t>(vision->visionInfo.oreTank.posit_X),
//                             static_cast<int32_t>(vision->visionInfo.oreTank.posit_Y),
//                             static_cast<int32_t>(vision->visionInfo.oreTank.posit_Z));
//      } else {
//        uart->FormatTransmit("Ore Tank Not Found!\n");
//      }
//    }

    proc_waitMs(100);  // 10Hz
  }
}

} // namespace robotpilots
