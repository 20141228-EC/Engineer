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
 * @brief Update Task Entrypoint
 * @param arg[in] Not Used
 */
void StartUpdateTask(void *arg) {

  /* System Core Initialization */
  SystemCore.InitSystemCore();

  /* Update Task */
  while (true) {

    /* Device Update */
    for (const auto &item : DeviceMap)
      item.second->UpdateHandler_();

    /* System Core Update */
    SystemCore.UpdateHandler_();

    /* Module Update */
    for (const auto &item : ModuleMap)
      item.second->UpdateHandler_();

    proc_waitMs(1);   // 1000Hz
  }
}

} // namespace robotpilots
