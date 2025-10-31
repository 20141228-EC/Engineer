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
 * @brief System Update Task Entrypoint
 * @param arg[in] Not Used
 */
void StartSystemUpdateTask(void *arg) {

  /* System Update Task */
  while (true) {

    /* System Update */
    for (const auto &item: SystemMap)
      item.second->UpdateHandler_();

    proc_waitMs(4);   // 250Hz
  }
}

} // namespace robotpilots
