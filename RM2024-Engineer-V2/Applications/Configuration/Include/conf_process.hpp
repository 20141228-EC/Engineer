/**
 * @file        conf_process.hpp
 * @version     1.0
 * @date        2024-05-04
 * @author      Morthine Xiang
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-04   <td>1.0         <td>Morthine Xiang  <td>First Create.
 * </table>
 */

#ifndef RP_CONF_PROCESS_HPP
#define RP_CONF_PROCESS_HPP

#include "conf_common.hpp"

#define proc_DeviceInitTaskPriority     30
#define proc_UpdateTaskPriority         24
#define proc_SystemTaskPriority         20
#define proc_SystemCoreTaskPriority     18
#define proc_ModuleTaskPriority         16
#define proc_HeartbeatTaskPriority      4
#define proc_MonitorTaskPriority        4

#define proc_return() vTaskDelete(nullptr)
#define proc_waitMs(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define proc_waitUntil(cond) do { vTaskDelay(pdMS_TO_TICKS(10)); } while (!(cond))

namespace robotpilots {

  void InitializeProcess();

}

#endif // RP_CONF_PROCESS_HPP
