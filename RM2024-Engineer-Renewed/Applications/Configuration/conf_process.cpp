/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-06
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-06   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "conf_process.hpp"

namespace robotpilots {

TaskHandle_t MonitorTaskHandle;
TaskHandle_t UpdateTaskHandle;
TaskHandle_t HeartbeatTaskHandle;
TaskHandle_t SystemUpdateTaskHandle;

void StartUpdateTask(void* arg);
void StartHeartbeatTask(void* arg);
void StartMonitorTask(void* arg);
void StartSystemUpdateTask(void *arg);

/**
 * @brief
 */
void InitializeProcess() {

  /* Create Monitor Task */
  xTaskCreate(StartMonitorTask, "Monitor Task",
              1024, nullptr, proc_MonitorTaskPriority,
              &MonitorTaskHandle);

  /* Create Update Task */
  xTaskCreate(StartUpdateTask, "Update Task",
              4096, nullptr, proc_UpdateTaskPriority,
              &UpdateTaskHandle);

  /* Create Heartbeat Task */
  xTaskCreate(StartHeartbeatTask, "Heartbeat Task",
              2048, nullptr, proc_HeartbeatTaskPriority,
              &HeartbeatTaskHandle);

  /* Create System Update Task */
  xTaskCreate(StartSystemUpdateTask, "System Update Task",
              4096, nullptr, proc_SystemTaskPriority,
              &SystemUpdateTaskHandle);
}

} // namespace robotpilots
