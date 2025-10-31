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

#include "mod_subgantry.hpp"

namespace robotpilots {

/**
 * @brief
 * @param param
 * @return
 */
ERpStatus CModSubGantry::InitModule(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Module */
  auto &subGantryParam = reinterpret_cast<SModSubGantryInitParam &>(param);
  moduleId = subGantryParam.moduleId;

  subGantryMtrCanTxNode_.InitTxNode(subGantryParam.subGantryMtrCanInfID,
                                    subGantryParam.subGantryMtrCanStdID,
                                    CCanInterface::ECanFrameType::DATA,
                                    CCanInterface::ECanFrameDlc::DLC_8);

  /* Initialize Components */
  comLift_.InitComponent(param);
  comStretch_.InitComponent(param);

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
void CModSubGantry::UpdateHandler_() {

  if (moduleState == RP_RESET) return;

  /* Update Sub-Gantry Component */
  comLift_.UpdateComponent();
  comStretch_.UpdateComponent();

  /* Update Sub-Gantry Info */
  subGantryInfo.posit_Lift_L =
    CComLift::MtrPositToPhyPosit(comLift_.liftInfo.posit_L);
  subGantryInfo.posit_Lift_R =
    CComLift::MtrPositToPhyPosit(comLift_.liftInfo.posit_R);
  subGantryInfo.posit_Stretch_L =
    CComStretch::MtrPositToPhyPosit(comStretch_.stretchInfo.posit_L);
  subGantryInfo.posit_Stretch_R =
    CComStretch::MtrPositToPhyPosit(comStretch_.stretchInfo.posit_R);
  subGantryInfo.isPositArrived_Lift_L = comLift_.liftInfo.isPositArrived_L;
  subGantryInfo.isPositArrived_Lift_R = comLift_.liftInfo.isPositArrived_R;
  subGantryInfo.isPositArrived_Stretch_L = comStretch_.stretchInfo.isPositArrived_L;
  subGantryInfo.isPositArrived_Stretch_R = comStretch_.stretchInfo.isPositArrived_R;

  /* Send Motor Output */
  CMtrDji::FillCanTxBuffer(comLift_.motor[CComLift::L],
                           subGantryMtrCanTxNode_.dataBuffer,
                           comLift_.mtrOutputBuffer[CComLift::L]);
  CMtrDji::FillCanTxBuffer(comLift_.motor[CComLift::R],
                           subGantryMtrCanTxNode_.dataBuffer,
                           comLift_.mtrOutputBuffer[CComLift::R]);
  CMtrDji::FillCanTxBuffer(comStretch_.motor[CComStretch::L],
                           subGantryMtrCanTxNode_.dataBuffer,
                           comStretch_.mtrOutputBuffer[CComStretch::L]);
  CMtrDji::FillCanTxBuffer(comStretch_.motor[CComStretch::R],
                           subGantryMtrCanTxNode_.dataBuffer,
                           comStretch_.mtrOutputBuffer[CComStretch::R]);
  subGantryMtrCanTxNode_.Transmit();
}


/**
 * @brief
 */
void CModSubGantry::HeartbeatHandler_() {

}


/**
 * @brief
 * @return
 */
ERpStatus CModSubGantry::CreateModuleTask_() {

  if (moduleTaskHandle != nullptr) vTaskDelete(moduleTaskHandle);
  xTaskCreate(StartSubGantryModuleTask, "SubGantry Module Task",
              512, this, proc_ModuleTaskPriority,
              &moduleTaskHandle);

  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModSubGantry::RestrictGantryCommand_() {

    if (moduleState == RP_RESET) {
      subGantryCmd = SSubGantryCmd();
      return RP_ERROR;
    }

    if (subGantryCmd.isAutoCtrl) return RP_BUSY;

    /* Mechanical Limit */
    subGantryCmd.setPosit_Lift_L =
      std::clamp(subGantryCmd.setPosit_Lift_L, 0.0f, 295.0f);
    subGantryCmd.setPosit_Lift_R =
      std::clamp(subGantryCmd.setPosit_Lift_R, 0.0f, 295.0f);
    subGantryCmd.setPosit_Stretch_L =
      std::clamp(subGantryCmd.setPosit_Stretch_L, 0.0f, 360.0f);
    subGantryCmd.setPosit_Stretch_R =
      std::clamp(subGantryCmd.setPosit_Stretch_R, 0.0f, 360.0f);

    return RP_OK;
}

} // namespace robotpilots
