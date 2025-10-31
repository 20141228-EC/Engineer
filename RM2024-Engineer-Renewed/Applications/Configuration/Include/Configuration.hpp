/**
 * @file        Configuration.hpp
 * @version     1.0
 * @date        2024-03-15
 * @author      Morthine Xiang
 * @email       xiang@morthine.com
 * @brief       User Application Configuration Header
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-15   <td>1.0         <td>Morthine Xiang  <td>First Create.
 * </table>
 */

#ifndef RP_CONFIGURATION_HPP
#define RP_CONFIGURATION_HPP

#include "conf_common.hpp"
#include "conf_interface.hpp"
#include "conf_process.hpp"
#include "conf_device.hpp"
#include "conf_module.hpp"

namespace robotpilots {

/**
 * @brief Interface ID Enum
 */
enum class EInterfaceID {
  INF_NULL = -1,      ///< NULL
  INF_TEST = 0,       ///< Only Use for Test Interface
  INF_DBUS,           ///< DBUS Interface (Using USART5)
  INF_UART1,          ///< UART1 Interface (Used for Referee System)
  INF_UART7,          ///< UART7 Interface (Used for Vision)
  INF_UART10,         ///< UART10 Interface (Used for Debugging)
  INF_CAN1,           ///< CAN1 Interface (Used for Chassis Motors)
  INF_CAN2,           ///< CAN2 Interface (Used for Gantry Motors)
  INF_CAN3,           ///< CAN3 Interface (Used for Manipulator Motors)
  INF_SPI2,           ///< SPI2 Interface (Used for BMI-088 MEMS)
  INF_SPI6,           ///< SPI6 Interface (Used for WS2312 LED)
};


/**
 * @brief Device ID Enum
 */
enum class EDeviceID {
  DEV_NULL = -1,          ///< NULL
  DEV_TEST = 0,           ///< Only Use for Test Device
  DEV_RC_DR16,            ///< Remote Control Receiver (DR16)
  DEV_RM_REFEREE,         ///< RoboMaster Referee System Device
  DEV_RP_VISION,          ///< RoboPilots Vision System Device
  DEV_MEMS_BMI088,        ///< 6-axis Mems (BMI-088)
  DEV_CHAS_MTR_LF,        ///< Chassis Motor Front-Left (M3508)
  DEV_CHAS_MTR_RF,        ///< Chassis Motor Front-Right (M3508)
  DEV_CHAS_MTR_LB,        ///< Chassis Motor Back-Left (M3508)
  DEV_CHAS_MTR_RB,        ///< Chassis Motor Back-Right (M3508)
  DEV_GIMB_MTR_L,         ///< Gimbal Lift Motor Left (M2006)
  DEV_GIMB_MTR_R,         ///< Gimbal Lift Motor Right (M2006)
  DEV_GIMB_MTR_PITCH,     ///< Gimbal Pitch Motor (Servo)
  DEV_LIFT_MTR_L,         ///< Gantry Lift Motor Left (M3508)
  DEV_LIFT_MTR_R,         ///< Gantry Lift Motor Right (M3508)
  DEV_STRCH_MTR_L,        ///< Gantry Stretch Motor Left (M3508)
  DEV_STRCH_MTR_R,        ///< Gantry Stretch Motor Right (M3508)
  DEV_TRAV_MTR,           ///< Gantry Traverse Motor (M2006)
  DEV_FLIP_MTR_F,         ///< Gantry Flip Motor Front (M2006)
  DEV_FLIP_MTR_B,         ///< Gantry Flip Motor Back (M2006)
  DEV_MANIP_MTR_YAW,      ///< Manipulator Motor Yaw (M2006)
  DEV_MANIP_MTR_ROLL,     ///< Manipulator Motor Roll (M2006)
  DEV_MANIP_MTR_END_L,    ///< Manipulator Motor End-Left (M2006)
  DEV_MANIP_MTR_END_R,    ///< Manipulator Motor End-Right (M2006)
};


/**
 * @brief Module ID Enum
 */
enum class EModuleID {
  MOD_NULL = -1,      ///< NULL
  MOD_TEST = 0,       ///< Only Use for Test Module
  MOD_CHASSIS,        ///< Chassis Module
  MOD_GIMBAL,         ///< Gimbal Module
  MOD_GANTRY,         ///< Gantry Module
};


enum class ESystemID {
  SYS_NULL = -1,      ///< NULL
  SYS_TEST = 0,       ///< Only Use for Test System
  SYS_REMOTE,         ///< Remote Control System
  SYS_REFEREE,        ///< RoboMaster Referee System
  SYS_VISION,         ///< RobotPilots Vision System
};


/**
 * @brief Initialize User Application
 *
 * @return None
 */
void ApplicationEntryPoint();

} // namespace robotpilots

#endif  // RP_CONFIGURATION_HPP
