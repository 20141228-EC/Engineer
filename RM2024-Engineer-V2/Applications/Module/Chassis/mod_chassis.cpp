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

#include "mod_chassis.hpp"

namespace robotpilots {

/**
 * @brief
 * @param param
 * @return
 */
ERpStatus CModChassis::InitModule(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Module */
  auto chassisParam = static_cast<SModChassisInitParam &>(param);
  moduleId = chassisParam.moduleId;

  chassisMtrCanTxNode_.InitTxNode(chassisParam.chassisMtrCanInfID,
                                  chassisParam.chassisMtrCanStdID,
                                  CCanInterface::ECanFrameType::DATA,
                                  CCanInterface::ECanFrameDlc::DLC_8);

  /* Initialize Components */
  comWheelset_.InitComponent(param);

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
void CModChassis::UpdateHandler_() {

  if (moduleState == RP_RESET) return;

  /* Update Component */
  comWheelset_.UpdateComponent();

  /* Send Motor Output */
  CMtrDji::FillCanTxBuffer(comWheelset_.motor[CComWheelset::LF],
                           chassisMtrCanTxNode_.dataBuffer,
                           comWheelset_.mtrOutputBuffer[CComWheelset::LF]);
  CMtrDji::FillCanTxBuffer(comWheelset_.motor[CComWheelset::RF],
                           chassisMtrCanTxNode_.dataBuffer,
                           comWheelset_.mtrOutputBuffer[CComWheelset::RF]);
  CMtrDji::FillCanTxBuffer(comWheelset_.motor[CComWheelset::LB],
                           chassisMtrCanTxNode_.dataBuffer,
                           comWheelset_.mtrOutputBuffer[CComWheelset::LB]);
  CMtrDji::FillCanTxBuffer(comWheelset_.motor[CComWheelset::RB],
                           chassisMtrCanTxNode_.dataBuffer,
                           comWheelset_.mtrOutputBuffer[CComWheelset::RB]);
  chassisMtrCanTxNode_.Transmit();
}


/**
 * @brief
 */
void CModChassis::HeartbeatHandler_() {

}


/**
 * @brief
 * @return
 */
ERpStatus CModChassis::CreateModuleTask_() {

  if (moduleTaskHandle != nullptr) vTaskDelete(moduleTaskHandle);
  xTaskCreate(StartChassisModuleTask, "Chassis Module Task",
              512, this, proc_ModuleTaskPriority,
              &moduleTaskHandle);

  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModChassis::RestrictChassisCommand_() {

  if (moduleState == RP_RESET) {
    chassisCmd = SChassisCmd();
    return RP_ERROR;
  }

  chassisCmd.speed_X = std::clamp(chassisCmd.speed_X, -100.0f, 100.0f);
  chassisCmd.speed_Y = std::clamp(chassisCmd.speed_Y, -100.0f, 100.0f);
  chassisCmd.speed_W = std::clamp(chassisCmd.speed_W, -100.0f, 100.0f);

  return RP_OK;
}

} // namespace robotpilots
