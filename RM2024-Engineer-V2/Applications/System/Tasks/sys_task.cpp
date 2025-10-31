/**
 * @file        sys_task.cpp
 * @version     1.0
 * @date        2024-03-31
 * @author      Morthine Xiang
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-31   <td>1.0         <td>Morthine Xiang  <td>First Create.
 * </table>
 */

#include "Core.hpp"

#define deg2rad (3.1415926f / 180.0f)

extern IWDG_HandleTypeDef hiwdg1;

namespace robotpilots {

void JointAngleToEulerAngle (const float32_t *jointAngle, float32_t *eulerAngle);


/**
 * @brief Monitor Task Entrypoint
 * @param arg[in] Not Used
 */
void StartMonitorTask(void *arg) {

  auto *uart = reinterpret_cast<CUartInterface *>(InterfaceMap.at(EInterfaceID::INF_UART10));
  auto *vision = reinterpret_cast<CSysVision *>(SystemMap.at(ESystemID::SYS_VISION));
  auto *motor_1 = MotorMap.at(EDeviceID::DEV_MANIP_MTR_YAW);
  auto *motor_2 = MotorMap.at(EDeviceID::DEV_MANIP_MTR_ROLL);
//  CModGantry *gantry = reinterpret_cast<CModGantry *>(ModuleMap.at(EModuleID::MOD_GANTRY));

  /* Monitor Task */
  while (true) {

//    if (SysVision.systemState != RP_OK) {
//      uart->FormatTransmit("Vision System Error!\r\n");
//    } else {
//      if (vision->visionInfo.oreTank.isFoundOreTank) {
//        uart->FormatTransmit("YPR: %d, %d, %d\r\n",
//                             static_cast<int32_t>(vision->visionInfo.oreTank.atti_YAW),
//                             static_cast<int32_t>(vision->visionInfo.oreTank.atti_PITCH),
//                             static_cast<int32_t>(vision->visionInfo.oreTank.atti_ROLL));
//        uart->FormatTransmit("XYZ: %d, %d, %d\r\n",
//                             static_cast<int32_t>(vision->visionInfo.oreTank.posit_X),
//                             static_cast<int32_t>(vision->visionInfo.oreTank.posit_Y),
//                             static_cast<int32_t>(vision->visionInfo.oreTank.posit_Z));
//      } else {
//        uart->FormatTransmit("Ore Tank Not Found!\r\n");
//      }
//    }

    uart->FormatTransmit("Yaw: %d, Roll %d\r\n", motor_1->motorData[CMtrInstance::DATA_ANGLE], motor_2->motorData[CMtrInstance::DATA_ANGLE]);

    proc_waitMs(100);  // 10Hz
  }
}


/**
 * @brief System Update Task Entrypoint
 * @param arg[in] Not Used
 */
void StartSystemUpdateTask(void *arg) {

//  SystemCore.InitSystemCore();

  /* System Update Task */
  while (true) {

    /* System Update */
    for (const auto &item: SystemMap)
      item.second->UpdateHandler_();

    proc_waitMs(4);   // 250Hz
  }
}


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