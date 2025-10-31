/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-16
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-16   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#ifndef RP_CORE_HPP
#define RP_CORE_HPP

#include "Interface.hpp"
#include "Device.hpp"
#include "Module.hpp"
#include "System.hpp"

namespace robotpilots {

/**
 * @brief System Core Class
 */
class CSystemCore {
  friend void StartUpdateTask(void *arg);
  friend void StartHeartbeatTask(void *arg);
public:

  enum class EAutoCtrlProcess {
    NONE,
    RETURN_ORIGIN,
    RETURN_DRIVE,
    GROUND_ORE,
    SILVER_ORE,
    GOLD_ORE,
    EXCHANGE,
    PUSH_ORE,
    POP_ORE,
  } currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

  ERpStatus InitSystemCore();

private:

  ERpStatus coreState = RP_RESET;

  const float_t freq = 1000.0f;

  CModChassis *chassis_ = nullptr;

  CModGimbal *gimbal_ = nullptr;

  CModGantry *gantry_ = nullptr;

  CModSubGantry *subgantry_ = nullptr;

  TaskHandle_t autoCtrlTaskHandle_ = nullptr;

  void UpdateHandler_();

  void HeartbeatHandler_();

  void ControlFromRemote_();

  void ControlFromKeyboard_();

  ERpStatus StartAutoCtrlTask_(EAutoCtrlProcess process);

  ERpStatus StopAutoCtrlTask_();

  static void StartReturnOriginTask(void *arg);

  static void StartReturnDriveTask(void *arg);

  static void StartGroundOreTask(void *arg);

  static void StartSilverOreTask(void *arg);

  static void StartGoldOreTask(void *arg);

  static void StartExchangeTask(void *arg);

  static void StartPopOreTask(void *arg);

  static void StartPushOreTask(void *arg);

};

extern CSystemCore SystemCore;

} // namespace robotpilots

#endif// RP_CORE_HPP
