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

#include "mod_gimbal.hpp"

namespace robotpilots {

/**
 * @brief
 * @param param
 * @return
 */
ERpStatus CModGimbal::InitModule(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Module */
  auto &gimbalParam = static_cast<SModGimbalInitParam &>(param);
  moduleId = gimbalParam.moduleId;

  gimbalMtrCanTxNode_.InitTxNode(gimbalParam.gimbalMtrCanInfID,
                                 gimbalParam.gimbalMtrCanStdID,
                                 CCanInterface::ECanFrameType::DATA,
                                 CCanInterface::ECanFrameDlc::DLC_8);

  /* Initialize Components */
  comLift_.InitComponent(param);
  comPitch_.InitComponent(param);

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
void CModGimbal::UpdateHandler_() {

  if (moduleState == RP_RESET) return;

  /* Update Gimbal Components */
  comLift_.UpdateComponent();
  comPitch_.UpdateComponent();

  /* Update Gimbal Info */
  gimbalInfo.posit_Lift = CComLift::MtrPositToPhyPosit(comLift_.liftInfo.posit);
//  gimbalInfo.step_pitch = comPitch_.pitchCmd.setStep;
  gimbalInfo.isPositArrived_Lift = comLift_.liftInfo.isPositArrived;

  /* Send Motor Output */
  CMtrDji::FillCanTxBuffer(comLift_.motor[CComLift::L],
                           gimbalMtrCanTxNode_.dataBuffer,
                           comLift_.mtrOutputBuffer[CComLift::L]);
  CMtrDji::FillCanTxBuffer(comLift_.motor[CComLift::R],
                           gimbalMtrCanTxNode_.dataBuffer,
                           comLift_.mtrOutputBuffer[CComLift::R]);
  CMtrDji::FillCanTxBuffer(comPitch_.motor[0],
                           gimbalMtrCanTxNode_.dataBuffer,
                           comPitch_.mtrOutputBuffer[0]);
  gimbalMtrCanTxNode_.Transmit();
}


/**
 * @brief
 */
void CModGimbal::HeartbeatHandler_() {

  if (moduleState == RP_RESET) return;

}


/**
 * @brief
 * @return
 */
ERpStatus CModGimbal::CreateModuleTask_() {

  if (moduleTaskHandle != nullptr) vTaskDelete(moduleTaskHandle);
  xTaskCreate(StartGimbalModuleTask, "Gimbal Module Task",
              512, this, 5,
              &moduleTaskHandle);

  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModGimbal::RestrictGimbalCommand_() {

  if (moduleState == RP_RESET) {
    gimbalCmd = SGimbalCmd();
    return RP_ERROR;
  }

  if (gimbalCmd.isAutoCtrl)
    return RP_BUSY;

  gimbalCmd.setPosit_Lift =
    std::clamp(gimbalCmd.setPosit_Lift, 0.0f, 240.0f);
  gimbalCmd.setStep_Pitch =
    std::clamp(gimbalCmd.setStep_Pitch, 0ul, 2ul);

  return RP_OK;
}

} // namespace robotpilots
