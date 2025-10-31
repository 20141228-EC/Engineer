/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-10
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-10   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "mod_gantry.hpp"

namespace robotpilots {

/**
 * @brief Initialize Gantry Module
 *
 * @param param[in] Module Initialization Parameter
 * @return RP_OK - Init Success
 * @return RP_ERROR - Init Failed
 */
ERpStatus CModGantry::InitModule(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Module */
  auto &gantryParam = static_cast<SModGantryInitParam &>(param);
  moduleId = gantryParam.moduleId;

  gantryMtrCanTxNode_.InitTxNode(gantryParam.gantryMtrCanInfID,
                                 gantryParam.gantryMtrCanStdID,
                                 CCanInterface::ECanFrameType::DATA,
                                 CCanInterface::ECanFrameDlc::DLC_8);

  gantryTravMtrCanTxNode_.InitTxNode(gantryParam.gantryTravMtrCanInfID,
                                     gantryParam.gantryTravMtrCanStdID,
                                     CCanInterface::ECanFrameType::DATA,
                                     CCanInterface::ECanFrameDlc::DLC_8);

  gantryManipMtrCanTxNode_.InitTxNode(gantryParam.gantryManipMtrCanInfID,
                                      gantryParam.gantryManipMtrCanStdID,
                                      CCanInterface::ECanFrameType::DATA,
                                      CCanInterface::ECanFrameDlc::DLC_8);

  gantryFlipMtrCanTxNode_.InitTxNode(gantryParam.gantryFlipMtrCanInfID,
                                     gantryParam.gantryFlipMtrCanStdID,
                                     CCanInterface::ECanFrameType::DATA,
                                     CCanInterface::ECanFrameDlc::DLC_8);

  /* Initialize Components */
  comPump_.InitComponent(param);
  comLift_.InitComponent(param);
  comStretch_.InitComponent(param);
  comTraverse_.InitComponent(param);
  comJointRoll_.InitComponent(param);
  comJointYaw_.InitComponent(param);
  comEnd_.InitComponent(param);
  comFlip_.InitComponent(param);

  /* Register Module */
  CreateModuleTask_();
  RegisterModule_();

  /* Set Component Flags */
  processFlag_ = 0;
  moduleState = RP_OK;

  return RP_OK;
}


/**
 * @brief
 */
void CModGantry::UpdateHandler_() {

  if (moduleState == RP_RESET) return;

  /* Update Gantry Component */
  comPump_.UpdateComponent();
  comLift_.UpdateComponent();
  comStretch_.UpdateComponent();
  comTraverse_.UpdateComponent();
  comJointYaw_.UpdateComponent();
  comJointRoll_.UpdateComponent();
  comEnd_.UpdateComponent();
  comFlip_.UpdateComponent();

  /* Update Gantry Info */
  gantryInfo.isPumpOn = comPump_.pumpInfo.isPumpOn;
  gantryInfo.posit_Lift = CComLift::MtrPositToPhyPosit(comLift_.liftInfo.posit);
  gantryInfo.posit_Stretch = CComStretch::MtrPositToPhyPosit(comStretch_.stretchInfo.posit);
  gantryInfo.posit_Traverse = CComTraverse::MtrPositToPhyPosit(comTraverse_.traverseInfo.posit);
  gantryInfo.angle_Joint_Yaw = CComJointYaw::MtrPositToPhyPosit(comJointYaw_.jointYawInfo.posit);
  gantryInfo.angle_Joint_Roll = CComJointRoll::MtrPositToPhyPosit(comJointRoll_.jointRollInfo.posit);
  gantryInfo.angle_End_Pitch = CComEnd::MtrPositToPhyPosit_Pitch(comEnd_.endInfo.posit_Pitch);
  gantryInfo.angle_End_Roll = CComEnd::MtrPositToPhyPosit_Roll(comEnd_.endInfo.posit_Roll);
  gantryInfo.isPositArrived_Lift = comLift_.liftInfo.isPositArrived;
  gantryInfo.isPositArrived_Stretch = comStretch_.stretchInfo.isPositArrived;
  gantryInfo.isPositArrived_Traverse = comTraverse_.traverseInfo.isPositArrived;
  gantryInfo.isPositArrived_Joint_Yaw = comJointYaw_.jointYawInfo.isPositArrived;
  gantryInfo.isPositArrived_Joint_Roll = comJointRoll_.jointRollInfo.isPositArrived;
  gantryInfo.isPositArrived_End_Pitch = comEnd_.endInfo.isPositArrived_Pitch;
  gantryInfo.isPositArrived_End_Roll = comEnd_.endInfo.isPositArrived_Roll;

  /* Send Motor Output */
  CMtrDji::FillCanTxBuffer(comLift_.motor[CComLift::L], gantryMtrCanTxNode_.dataBuffer,
                           comLift_.mtrOutputBuffer[CComLift::L]);
  CMtrDji::FillCanTxBuffer(comLift_.motor[CComLift::R], gantryMtrCanTxNode_.dataBuffer,
                           comLift_.mtrOutputBuffer[CComLift::R]);
  CMtrDji::FillCanTxBuffer(comStretch_.motor[CComStretch::L], gantryMtrCanTxNode_.dataBuffer,
                           comStretch_.mtrOutputBuffer[CComStretch::L]);
  CMtrDji::FillCanTxBuffer(comStretch_.motor[CComStretch::R], gantryMtrCanTxNode_.dataBuffer,
                           comStretch_.mtrOutputBuffer[CComStretch::R]);
  gantryMtrCanTxNode_.Transmit();

  CMtrDji::FillCanTxBuffer(comTraverse_.motor[0], gantryTravMtrCanTxNode_.dataBuffer,
                           comTraverse_.mtrOutputBuffer[0]);
  gantryTravMtrCanTxNode_.Transmit();

  CMtrDji::FillCanTxBuffer(comJointYaw_.motor[0], gantryManipMtrCanTxNode_.dataBuffer,
                           comJointYaw_.mtrOutputBuffer[0]);
  CMtrDji::FillCanTxBuffer(comJointRoll_.motor[0], gantryManipMtrCanTxNode_.dataBuffer,
                           comJointRoll_.mtrOutputBuffer[0]);
  CMtrDji::FillCanTxBuffer(comEnd_.motor[CComEnd::L], gantryManipMtrCanTxNode_.dataBuffer,
                           comEnd_.mtrOutputBuffer[CComEnd::L]);
  CMtrDji::FillCanTxBuffer(comEnd_.motor[CComEnd::R], gantryManipMtrCanTxNode_.dataBuffer,
                           comEnd_.mtrOutputBuffer[CComEnd::R]);
  gantryManipMtrCanTxNode_.Transmit();

  CMtrDji::FillCanTxBuffer(comFlip_.motor[CComFlip::F], gantryFlipMtrCanTxNode_.dataBuffer,
                           comFlip_.mtrOutputBuffer[CComFlip::F]);
  CMtrDji::FillCanTxBuffer(comFlip_.motor[CComFlip::B], gantryFlipMtrCanTxNode_.dataBuffer,
                            comFlip_.mtrOutputBuffer[CComFlip::B]);
  gantryFlipMtrCanTxNode_.Transmit();

}


/**
 * @brief
 */
void CModGantry::HeartbeatHandler_() {

  if (moduleState == RP_RESET) return;

}


/**
 * @brief
 * @return
 */
ERpStatus CModGantry::CreateModuleTask_() {

  if (moduleTaskHandle != nullptr) vTaskDelete(moduleTaskHandle);
  xTaskCreate(StartGantryModuleTask, "Gantry Module Task",
              1024, this, proc_ModuleTaskPriority,
              &moduleTaskHandle);

  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModGantry::RestrictGantryCommand_() {

  if (moduleState == RP_RESET) {
    gantryCmd = SGantryCmd();
    return RP_ERROR;
  }

  if (gantryCmd.isAutoCtrl) return RP_BUSY;

  /* Mechanical Limit */
  gantryCmd.setPosit_Lift =
    std::clamp(gantryCmd.setPosit_Lift, 0.0f, 560.0f);
  gantryCmd.setPosit_Stretch =
    std::clamp(gantryCmd.setPosit_Stretch, 0.0f, 420.0f);
  gantryCmd.setPosit_Traverse =
    std::clamp(gantryCmd.setPosit_Traverse, -90.0f, 90.0f);
  gantryCmd.setAngle_Joint_Yaw =
    std::clamp(gantryCmd.setAngle_Joint_Yaw, -90.0f, 180.0f);
  gantryCmd.setAngle_Joint_Roll =
    std::clamp(gantryCmd.setAngle_Joint_Roll, -90.0f, 200.0f);
  gantryCmd.setAngle_End_Pitch =
    std::clamp(gantryCmd.setAngle_End_Pitch, -80.0f, 125.0f);
  gantryCmd.setSpeed_Flip_Y =
    std::clamp(gantryCmd.setSpeed_Flip_Y, -100.0f, 100.0f);
  gantryCmd.setSpeed_Flip_W =
    std::clamp(gantryCmd.setSpeed_Flip_W, -100.0f, 100.0f);

  /* Dynamic Protect */
  if (gantryCmd.setAngle_Joint_Yaw > 90.0f)
    gantryCmd.setAngle_Joint_Roll = 180.0f;

  if (gantryCmd.setAngle_Joint_Yaw > 90.0f
      && abs(gantryInfo.angle_Joint_Roll - 180.0f) > 5.0f)
    gantryCmd.setAngle_Joint_Yaw = 90.0f;

  if (gantryCmd.setPosit_Stretch < 100.0f)
    gantryCmd.setPosit_Traverse = 0.0f;

  return RP_OK;
}

} // namespace robotpilots
