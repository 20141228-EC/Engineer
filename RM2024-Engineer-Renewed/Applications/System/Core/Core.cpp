/**
 * @file        ${PACKAGE_NAME}
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

#include "Core.hpp"

namespace robotpilots {

/**
 * @brief
 */
CSystemCore SystemCore;


/**
 * @brief
 * @return
 */
ERpStatus CSystemCore::InitSystemCore() {

  /* Start Interfaces Transfer */
  for (const auto &item: InterfaceMap)
    item.second->StartTransfer();

  /* Start Device */
  DeviceMap.at(EDeviceID::DEV_MEMS_BMI088)->StartDevice();

  /* Start System */
  CSysRemote::SSysRemoteInitParam sysRemoteInitParam;
  sysRemoteInitParam.systemId = ESystemID::SYS_REMOTE;
  sysRemoteInitParam.remoteDevID = EDeviceID::DEV_RC_DR16;
  SysRemote.InitSystem(&sysRemoteInitParam);

  CSysReferee::SSysRefereeInitParam sysRefereeInitParam;
  sysRefereeInitParam.systemId = ESystemID::SYS_REFEREE;
  sysRefereeInitParam.refereeDevID = EDeviceID::DEV_RM_REFEREE;
  SysReferee.InitSystem(&sysRefereeInitParam);

  CSysVision::SSysVisionInitParam sysVisionInitParam;
  sysVisionInitParam.systemId = ESystemID::SYS_VISION;
  sysVisionInitParam.visionDevID = EDeviceID::DEV_RP_VISION;
  SysVision.InitSystem(&sysVisionInitParam);

  /* Get Module Handle */
  chassis_   = reinterpret_cast<CModChassis *>(ModuleMap.at(EModuleID::MOD_CHASSIS));
  gimbal_    = reinterpret_cast<CModGimbal *>(ModuleMap.at(EModuleID::MOD_GIMBAL));
  gantry_    = reinterpret_cast<CModGantry *>(ModuleMap.at(EModuleID::MOD_GANTRY));

  proc_waitMs(2000);

  return RP_OK;
}


/**
 * @brief
 */
void CSystemCore::UpdateHandler_() {

  if (SysRemote.remoteInfo.remote.switch_L == 2
      && SysRemote.remoteInfo.remote.switch_R == 1)
    ControlFromKeyboard_();
  else
    ControlFromRemote_();

  if (SysRemote.remoteInfo.remote.thumbWheel > 50)
    gantry_->gantryCmd.setPumpOn = true;

  if (SysRemote.remoteInfo.remote.thumbWheel < -50)
    gantry_->gantryCmd.setPumpOn = false;

}


/**
 * @brief
 */
void CSystemCore::HeartbeatHandler_() {

  static auto lastRemoteState = RP_RESET;
  auto currentRemoteState = SysRemote.systemState;

  if (currentRemoteState != RP_OK
      && lastRemoteState == RP_OK) {
    chassis_->StopModule();
    gimbal_->StopModule();
    gantry_->StopModule();
    StopAutoCtrlTask_();
  }

  lastRemoteState = currentRemoteState;
}


/**
 * @brief
 * @param process
 * @return
 */
ERpStatus CSystemCore::StartAutoCtrlTask_(EAutoCtrlProcess process) {

  if (currentAutoCtrlProcess_ != EAutoCtrlProcess::NONE)
    return RP_BUSY;

  StopAutoCtrlTask_();

  switch (process) {

    case EAutoCtrlProcess::NONE: {

      return RP_ERROR;
    }

    case EAutoCtrlProcess::RETURN_ORIGIN: {

      currentAutoCtrlProcess_ = EAutoCtrlProcess::RETURN_ORIGIN;
      xTaskCreate(StartReturnOriginTask, "Return Origin Task",
                  512, this, proc_SystemCoreTaskPriority,
                  &autoCtrlTaskHandle_);
      return RP_OK;
    }

    case EAutoCtrlProcess::RETURN_DRIVE: {

      currentAutoCtrlProcess_ = EAutoCtrlProcess::RETURN_DRIVE;
      xTaskCreate(StartReturnDriveTask, "Return Drive Task",
                  512, this, proc_SystemCoreTaskPriority,
                  &autoCtrlTaskHandle_);
      return RP_OK;
    }

    case EAutoCtrlProcess::GROUND_ORE: {

      currentAutoCtrlProcess_ = EAutoCtrlProcess::GROUND_ORE;
      xTaskCreate(StartGroundOreTask, "Ground Ore Task",
                  512, this, proc_SystemCoreTaskPriority,
                  &autoCtrlTaskHandle_);
      return RP_OK;
    }

    case EAutoCtrlProcess::SILVER_ORE: {

      currentAutoCtrlProcess_ = EAutoCtrlProcess::SILVER_ORE;
      xTaskCreate(StartSilverOreTask, "Silver Ore Task",
                  512, this, proc_SystemCoreTaskPriority,
                  &autoCtrlTaskHandle_);
      return RP_OK;
    }

    case EAutoCtrlProcess::GOLD_ORE: {

      currentAutoCtrlProcess_ = EAutoCtrlProcess::GOLD_ORE;
      xTaskCreate(StartGoldOreTask, "Gold Ore Task",
                  512, this, proc_SystemCoreTaskPriority,
                  &autoCtrlTaskHandle_);
      return RP_OK;
    }

    case EAutoCtrlProcess::EXCHANGE: {

      currentAutoCtrlProcess_ = EAutoCtrlProcess::EXCHANGE;
      xTaskCreate(StartExchangeTask, "Exchange Task",
                  512, this, proc_SystemCoreTaskPriority,
                  &autoCtrlTaskHandle_);
      return RP_OK;
    }

    case EAutoCtrlProcess::PUSH_ORE: {

//      currentAutoCtrlProcess_ = EAutoCtrlProcess::PUSH_ORE;
//      xTaskCreate(StartPushOreTask, "Push Ore Task",
//                  512, this, proc_SystemCoreTaskPriority,
//                  &autoCtrlTaskHandle_);
      return RP_OK;
    }

    case EAutoCtrlProcess::POP_ORE: {

//      currentAutoCtrlProcess_ = EAutoCtrlProcess::POP_ORE;
//      xTaskCreate(StartPopOreTask, "Pop Ore Task",
//                  512, this, proc_SystemCoreTaskPriority,
//                  &autoCtrlTaskHandle_);
      return RP_OK;
    }

    default: return RP_ERROR;
  }

  return RP_ERROR;
}


/**
 * @brief
 * @return
 */
ERpStatus CSystemCore::StopAutoCtrlTask_() {

  if (autoCtrlTaskHandle_ == nullptr) return RP_ERROR;

  vTaskDelete(autoCtrlTaskHandle_);
  autoCtrlTaskHandle_     = nullptr;
  currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

  /* Clear AutoCtrl Flag */
  gimbal_->gimbalCmd.isAutoCtrl = false;
  gantry_->gantryCmd.isAutoCtrl = false;

  /* Stop Movement */
//  gimbal_->gimbalCmd.setPosit_Lift = gimbal_->gimbalInfo.posit_Lift;
//  gimbal_->gimbalCmd.setStep_Pitch = gimbal_->gimbalInfo.step_pitch;
//  gantry_->gantryCmd.setPosit_Lift = gantry_->gantryInfo.posit_Lift;
//  gantry_->gantryCmd.setPosit_Stretch = gantry_->gantryInfo.posit_Stretch;
//  gantry_->gantryCmd.setPosit_Traverse = gantry_->gantryInfo.posit_Traverse;
//  gantry_->gantryCmd.setAngle_Joint_Yaw = gantry_->gantryInfo.angle_Joint_Yaw;
//  gantry_->gantryCmd.setAngle_Joint_Roll = gantry_->gantryInfo.angle_Joint_Roll;
//  gantry_->gantryCmd.setAngle_End_Pitch = gantry_->gantryInfo.angle_End_Pitch;
//  gantry_->gantryCmd.setAngle_End_Roll = gantry_->gantryInfo.angle_End_Roll;
//  subgantry_->subGantryCmd.setPosit_Lift_L = subgantry_->subGantryInfo.posit_Lift_L;
//  subgantry_->subGantryCmd.setPosit_Lift_R = subgantry_->subGantryInfo.posit_Lift_R;
//  subgantry_->subGantryCmd.setPosit_Stretch_L = subgantry_->subGantryInfo.posit_Stretch_L;
//  subgantry_->subGantryCmd.setPosit_Stretch_R = subgantry_->subGantryInfo.posit_Stretch_R;

  return RP_OK;
}

} // namespace robotpilots
