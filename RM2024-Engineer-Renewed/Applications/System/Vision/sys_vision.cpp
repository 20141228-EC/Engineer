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

#include "System.hpp"

namespace robotpilots {

/**
 * @brief
 */
CSysVision SysVision;


/**
 * @brief
 * @param pStruct
 * @return
 */
ERpStatus CSysVision::InitSystem(SSystemInitParam *pStruct) {

  if (pStruct == nullptr) return RP_ERROR;
  if (pStruct->systemId == ESystemID::SYS_NULL) return RP_ERROR;

  auto &param = *reinterpret_cast<SSysVisionInitParam *>(pStruct);
  systemID = param.systemId;
  vision_ = reinterpret_cast<CDevVision *>(DeviceMap.at(param.visionDevID));

  if (systemTaskHandle != nullptr)
    vTaskDelete(systemTaskHandle);
  xTaskCreate(StartSysVisionTask, "System Vision Task",
              512, nullptr, proc_SystemTaskPriority,
              &systemTaskHandle);

  RegisterSystem_();

  systemState = RP_ERROR;
  return RP_OK;
}


/**
 * @brief
 */
void CSysVision::UpdateHandler_() {

  if (systemState != RP_OK) return;

  UpdateOreTankInfo_();
  UpdateUiPointInfo_();
}


/**
 * @brief
 */
void CSysVision::HeartbeatHandler_() {

  if (systemState == RP_RESET) return;

  if (vision_->visionState == CDevVision::EVisionStatus::ONLINE)
    systemState = RP_OK;
  else
    systemState = RP_ERROR;
}


/**
 * @brief
 * @return
 */
ERpStatus CSysVision::UpdateOreTankInfo_() {

  visionInfo.oreTank.isFoundOreTank = vision_->oretankinfoPkg.isFoundOreTank;
  visionInfo.oreTank.atti_YAW = vision_->oretankinfoPkg.atti[0];
  visionInfo.oreTank.atti_PITCH = vision_->oretankinfoPkg.atti[1];
  visionInfo.oreTank.atti_ROLL = vision_->oretankinfoPkg.atti[2];
  visionInfo.oreTank.posit_X = vision_->oretankinfoPkg.posit[0];
  visionInfo.oreTank.posit_Y = vision_->oretankinfoPkg.posit[1];
  visionInfo.oreTank.posit_Z = vision_->oretankinfoPkg.posit[2];

  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CSysVision::UpdateUiPointInfo_() {

  visionInfo.uiPoint.isFoundOreTank = vision_->oretankinfoPkg.isFoundOreTank;
  visionInfo.uiPoint.pointPosit_X[0] = vision_->oretankinfoPkg.uiPoint_X[0];
  visionInfo.uiPoint.pointPosit_Y[0] = vision_->oretankinfoPkg.uiPoint_Y[0];
  visionInfo.uiPoint.pointPosit_X[1] = vision_->oretankinfoPkg.uiPoint_X[1];
  visionInfo.uiPoint.pointPosit_Y[1] = vision_->oretankinfoPkg.uiPoint_Y[1];
  visionInfo.uiPoint.pointPosit_X[2] = vision_->oretankinfoPkg.uiPoint_X[2];
  visionInfo.uiPoint.pointPosit_Y[2] = vision_->oretankinfoPkg.uiPoint_Y[2];
  visionInfo.uiPoint.pointPosit_X[3] = vision_->oretankinfoPkg.uiPoint_X[3];
  visionInfo.uiPoint.pointPosit_Y[3] = vision_->oretankinfoPkg.uiPoint_Y[3];
  visionInfo.uiPoint.pointPosit_X[4] = vision_->oretankinfoPkg.uiPoint_X[4];
  visionInfo.uiPoint.pointPosit_Y[4] = vision_->oretankinfoPkg.uiPoint_Y[4];

  return RP_OK;
}


/**
 * @brief
 * @param arg
 */
void CSysVision::StartSysVisionTask(void *arg) {

  while (true) {

//    if (SysVision.systemState != RP_OK) {
//      proc_waitMs(100);
//      continue;
//    }

    CDevVision::SRaceinfoPkg raceInfo;
    raceInfo.raceCamp = SysReferee.refereeInfo.robot.robotCamp;
    raceInfo.raceState = 0;
    SysVision.vision_->SendPackage(CDevVision::ID_RACE_INFO, raceInfo.header);

    proc_waitMs(3);    // 20Hz
  }

}

} // namespace robotpilots
