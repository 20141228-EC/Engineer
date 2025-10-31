/**
 * @file        Configuration.cpp
 * @version     1.0
 * @date        2024-03-15
 * @author      Morthine Xiang
 * @email       xiang@morthine.com
 * @brief       User Application Configuration File
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-15   <td>1.0         <td>Morthine Xiang  <td>First Create.
 * </table>
 */

#include "Configuration.hpp"

namespace robotpilots {

/**
 * @brief User Application Entry Point
 *
 * @return None
 */
void ApplicationEntryPoint() {

  InitializeInterface();
  InitializeDevice();
  InitializeModule();
  InitializeProcess();

  vTaskStartScheduler();
}

} // namespace robotpilots
