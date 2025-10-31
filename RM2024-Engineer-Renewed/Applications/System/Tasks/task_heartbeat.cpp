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

extern IWDG_HandleTypeDef hiwdg1;

namespace robotpilots {

/**
 * @brief Heartbeat Task Entrypoint
 * @param arg[in] Not Used
 */
void StartHeartbeatTask(void *arg) {

  /* Heartbeat Task */
  while (true) {

    /* Interface Heartbeat */
    for (const auto &item : InterfaceMap)
      item.second->HeartbeatHandler_();

    /* Device Heartbeat */
    for (const auto &item : DeviceMap)
      item.second->HeartbeatHandler_();

    /* System Heartbeat */
    for (const auto &item: SystemMap)
      item.second->HeartbeatHandler_();

    /* System Core Heartbeat */
    SystemCore.HeartbeatHandler_();

    /* Module Heartbeat */
    for (const auto &item : ModuleMap)
      item.second->HeartbeatHandler_();

    HAL_IWDG_Refresh(&hiwdg1);

    proc_waitMs(10);  // 100Hz
  }
}

} // namespace robotpilots
