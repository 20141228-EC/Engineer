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

  gantryLiftMtrCanTxNode_.InitTxNode(gantryParam.gantryLiftMtrCanInfID,
                                     gantryParam.gantryLiftMtrCanStdID,
                                     CCanInterface::ECanFrameType::DATA,
                                     CCanInterface::ECanFrameDlc::DLC_8);

  gantryJointMtrCanTxNode_.InitTxNode(gantryParam.gantryJointMtrCanInfID,
                                      gantryParam.gantryJointMtrCanStdID,
                                      CCanInterface::ECanFrameType::DATA,
                                      CCanInterface::ECanFrameDlc::DLC_8);

  /* Initialize Components */
  comPump_.InitComponent(param);
  comLift_.InitComponent(param);
  comStretch_.InitComponent(param);
  comTraverse_.InitComponent(param);
  comJoint_.InitComponent(param);
  comEnd_.InitComponent(param);

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

  static int16_t halfTickRate = 0;
  halfTickRate = 1 - halfTickRate;

  /* Update Gantry Component */
  comPump_.UpdateComponent();
  comLift_.UpdateComponent();
  comJoint_.UpdateComponent();

  if (halfTickRate) {   // 500Hz
    comStretch_.UpdateComponent();
    comTraverse_.UpdateComponent();
    comEnd_.UpdateComponent();
  }

  /* Update Gantry Info */
  gantryInfo.isPumpOn_L = comPump_.pumpInfo.isPumpOn_L;
  gantryInfo.isPumpOn_C = comPump_.pumpInfo.isPumpOn_C;
  gantryInfo.isPumpOn_R = comPump_.pumpInfo.isPumpOn_R;
  gantryInfo.posit_Lift = CComLift::MtrPositToPhyPosit(comLift_.liftInfo.posit);
  gantryInfo.posit_Stretch = CComStretch::MtrPositToPhyPosit(comStretch_.stretchInfo.posit);
  gantryInfo.posit_Traverse = CComTraverse::MtrPositToPhyPosit(comTraverse_.traverseInfo.posit);
  gantryInfo.angle_Joint_Yaw = CComJoint::MtrPositToPhyPosit_Yaw(comJoint_.jointInfo.posit_Yaw);
  gantryInfo.angle_Joint_Roll = CComJoint::MtrPositToPhyPosit_Roll(comJoint_.jointInfo.posit_Roll);
  gantryInfo.angle_End_Pitch = CComEnd::MtrPositToPhyPosit_Pitch(comEnd_.endInfo.posit_Pitch);
  gantryInfo.angle_End_Roll = CComEnd::MtrPositToPhyPosit_Roll(comEnd_.endInfo.posit_Roll);
  gantryInfo.isPositArrived_Lift = comLift_.liftInfo.isPositArrived;
  gantryInfo.isPositArrived_Stretch = comStretch_.stretchInfo.isPositArrived;
  gantryInfo.isPositArrived_Traverse = comTraverse_.traverseInfo.isPositArrived;
  gantryInfo.isPositArrived_Joint_Yaw = comJoint_.jointInfo.isPositArrived_Yaw;
  gantryInfo.isPositArrived_Joint_Roll = comJoint_.jointInfo.isPositArrived_Roll;
  gantryInfo.isPositArrived_End_Pitch = comEnd_.endInfo.isPositArrived_Pitch;
  gantryInfo.isPositArrived_End_Roll = comEnd_.endInfo.isPositArrived_Roll;

  /* Send Motor Output */
  CMtrDji::FillCanTxBuffer(comStretch_.motor[0],
                           gantryMtrCanTxNode_.dataBuffer,
                           comStretch_.mtrOutputBuffer[0]);
  CMtrDji::FillCanTxBuffer(comTraverse_.motor[0],
                           gantryMtrCanTxNode_.dataBuffer,
                           comTraverse_.mtrOutputBuffer[0]);
  CMtrDji::FillCanTxBuffer(comEnd_.motor[CComEnd::L],
                           gantryMtrCanTxNode_.dataBuffer,
                           comEnd_.mtrOutputBuffer[CComEnd::L]);
  CMtrDji::FillCanTxBuffer(comEnd_.motor[CComEnd::R],
                           gantryMtrCanTxNode_.dataBuffer,
                           comEnd_.mtrOutputBuffer[CComEnd::R]);
  if (halfTickRate) gantryMtrCanTxNode_.Transmit();   // 500Hz

  CMtrDji::FillCanTxBuffer(comLift_.motor[CComLift::L],
                           gantryLiftMtrCanTxNode_.dataBuffer,
                           comLift_.mtrOutputBuffer[CComLift::L]);
  CMtrDji::FillCanTxBuffer(comLift_.motor[CComLift::R],
                           gantryLiftMtrCanTxNode_.dataBuffer,
                           comLift_.mtrOutputBuffer[CComLift::R]);
  gantryLiftMtrCanTxNode_.Transmit();

  CMtrDm::FillCanTxBuffer(comJoint_.motor[CComJoint::YAW],
                          gantryJointMtrCanTxNode_.dataBuffer,
                          comJoint_.mtrOutputBuffer[CComJoint::YAW]);
  CMtrDm::FillCanTxBuffer(comJoint_.motor[CComJoint::ROLL],
                          gantryJointMtrCanTxNode_.dataBuffer,
                          comJoint_.mtrOutputBuffer[CComJoint::ROLL]);
  gantryJointMtrCanTxNode_.Transmit();
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

  /* Mechanical Limit */
  gantryCmd.setPosit_Lift =
    std::clamp(gantryCmd.setPosit_Lift, 0.0f, 660.0f);
  gantryCmd.setPosit_Stretch =
    std::clamp(gantryCmd.setPosit_Stretch, 0.0f, 410.0f);
  gantryCmd.setPosit_Traverse =
    std::clamp(gantryCmd.setPosit_Traverse, 0.0f, 190.0f);
  gantryCmd.setAngle_Joint_Yaw =
    std::clamp(gantryCmd.setAngle_Joint_Yaw, -90.0f, 90.0f);
  gantryCmd.setAngle_Joint_Roll =
    std::clamp(gantryCmd.setAngle_Joint_Roll, -180.0f, 150.0f);
  gantryCmd.setAngle_End_Pitch =
    std::clamp(gantryCmd.setAngle_End_Pitch, -135.0f, 80.0f);

  /* Dynamic Limit */
  if (gantryCmd.isAutoCtrl) return RP_OK;

  if (gantryInfo.posit_Stretch < 150.0f) {
    auto max_JointYaw =  90.0f;
    auto min_JointYaw = -90.0f;
    gantryCmd.setAngle_Joint_Yaw =
      std::clamp(gantryCmd.setAngle_Joint_Yaw, min_JointYaw, max_JointYaw);
  }

  return RP_OK;
}

} // namespace robotpilots
