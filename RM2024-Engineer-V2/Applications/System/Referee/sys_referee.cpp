/**
 * @file        sys_referee.cpp
 * @version     1.0
 * @date        2024-05-06
 * @author      Morthine Xiang
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-06   <td>1.0         <td>Morthine Xiang  <td>First Create.
 * </table>
 */

#include "System.hpp"

namespace robotpilots {

/**
 * @brief
 */
CSysReferee SysReferee;


/**
 * @brief
 * @param pStruct
 * @return
 */
ERpStatus CSysReferee::InitSystem(SSystemInitParam *pStruct) {

  if (pStruct == nullptr) return RP_ERROR;
  if (pStruct->systemId == ESystemID::SYS_NULL) return RP_ERROR;

  auto &param = *reinterpret_cast<SSysRefereeInitParam *>(pStruct);
  systemID = param.systemId;
  referee_ = reinterpret_cast<CDevReferee *>(DeviceMap.at(param.refereeDevID));
  interface_ = reinterpret_cast<CUartInterface *>(InterfaceMap.at(EInterfaceID::INF_UART1));

  if (systemTaskHandle != nullptr)
    vTaskDelete(systemTaskHandle);
  xTaskCreate(StartSysRefereeUiTask, "SysReferee UI Task",
              512, nullptr, 5,
              &systemTaskHandle);

  RegisterSystem_();

  systemState = RP_ERROR;
  return RP_OK;
}


/**
 * @brief
 */
void CSysReferee::UpdateHandler_() {

  if (systemState != RP_OK) return;

  UpdateRaceInfo_();
  UpdateRobotInfo_();
}


/**
 * @brief
 */
void CSysReferee::HeartbeatHandler_() {

  if (systemState == RP_RESET) return;

  if (referee_->refereeState == CDevReferee::ERefereeStatus::ONLINE)
    systemState = RP_OK;
  else
    systemState = RP_ERROR;
}


/**
 * @brief
 * @return
 */
ERpStatus CSysReferee::UpdateRaceInfo_() {


  refereeInfo.unixTimestamp =
    referee_->raceStatusPkg.timestamp;

  refereeInfo.race.raceType =
    referee_->raceStatusPkg.raceType;

  refereeInfo.race.raceStage =
    referee_->raceStatusPkg.raceStage;

  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CSysReferee::UpdateRobotInfo_() {

  if (referee_->robotStatusPkg.robotId == 0)
    refereeInfo.robot.robotCamp = 0;
  else
    refereeInfo.robot.robotCamp = (referee_->robotStatusPkg.robotId < 100) ? 1 : 2;
  refereeInfo.robot.robotID =
    referee_->robotStatusPkg.robotId % 100;

  return RP_OK;
}

} // namespace robotpilots
