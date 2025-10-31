/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-03-22
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-22   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "System.hpp"

#include "FreeRTOS.h"
#include "task.h"

namespace robotpilots {

/**
 * @brief
 */
CSysRemote SysRemote;


/**
 * @brief
 * @param pStruct
 * @return
 */
ERpStatus CSysRemote::InitSystem(SSystemInitParam *pStruct) {

  if (pStruct == nullptr) return RP_ERROR;
  if (pStruct->systemId == ESystemID::SYS_NULL) return RP_ERROR;

  auto &param = *reinterpret_cast<SSysRemoteInitParam *>(pStruct);
  systemID = param.systemId;
  remote_ = reinterpret_cast<CRcInstance *>(DeviceMap.at(param.remoteDevID));
  // referee_ = static_cast<CRefereeInstance *>(DeviceMap.at(param.refereeDevID));

  RegisterSystem_();

  systemState = RP_ERROR;
  return RP_OK;
}


/**
 * @brief
 */
void CSysRemote::UpdateHandler_() {

  if (systemState != RP_OK) return;

  UpdateRemote_();
  UpdateKeyboard_();
}


/**
 * @brief
 */
void CSysRemote::HeartbeatHandler_() {

  if (systemState == RP_RESET) return;

  if (remote_->remoteState == ERcStatus::ONLINE)
    systemState = RP_OK;
  else
    systemState = RP_ERROR;
}


/**
 * @brief
 * @return
 */
ERpStatus CSysRemote::UpdateRemote_() {

  /* Soft Reset */
  if (remoteInfo.remote.switch_R == 3
      && remote_->remoteData[CRc_DR16::CH_SW1].chValue == 2
      && remote_->remoteData[CRc_DR16::CH_SW2].chValue == 2) {
    __set_FAULTMASK(1);
    NVIC_SystemReset();
  }

  remoteInfo.remote.joystick_RX = remote_->remoteData[CRc_DR16::CH_0].chValue / 6.6f;
  remoteInfo.remote.joystick_RY = remote_->remoteData[CRc_DR16::CH_1].chValue / 6.6f;
  remoteInfo.remote.joystick_LX = remote_->remoteData[CRc_DR16::CH_2].chValue / 6.6f;
  remoteInfo.remote.joystick_LY = remote_->remoteData[CRc_DR16::CH_3].chValue / 6.6f;
  remoteInfo.remote.thumbWheel  = remote_->remoteData[CRc_DR16::CH_TW].chValue / 6.6f;
  remoteInfo.remote.switch_L    = remote_->remoteData[CRc_DR16::CH_SW1].chValue;
  remoteInfo.remote.switch_R    = remote_->remoteData[CRc_DR16::CH_SW2].chValue;

  return RP_OK;
}

/**
 * @brief
 * @return
 */
ERpStatus CSysRemote::UpdateKeyboard_() {

  /* Soft Reset */
  if (remote_->remoteData[CRc_DR16::CH_KEY_CTRL] == 1
      && remote_->remoteData[CRc_DR16::CH_KEY_SHIFT].chValue == 1
      && remote_->remoteData[CRc_DR16::CH_KEY_B].chValue == 1) {
    __set_FAULTMASK(1);
    NVIC_SystemReset();
  }

  remoteInfo.keyboard.mouse_X     =  remote_->remoteData[CRc_DR16::CH_MOUSE_VX].chValue;
  remoteInfo.keyboard.mouse_Y     =  remote_->remoteData[CRc_DR16::CH_MOUSE_VY].chValue;
  remoteInfo.keyboard.mouse_Thumb =  remote_->remoteData[CRc_DR16::CH_MOUSE_VZ].chValue;
  remoteInfo.keyboard.mouse_L     = (remote_->remoteData[CRc_DR16::CH_MOUSE_L].chValue == 1);
  remoteInfo.keyboard.mouse_R     = (remote_->remoteData[CRc_DR16::CH_MOUSE_R].chValue == 1);
  remoteInfo.keyboard.key_W       = (remote_->remoteData[CRc_DR16::CH_KEY_W].chValue == 1);
  remoteInfo.keyboard.key_A       = (remote_->remoteData[CRc_DR16::CH_KEY_A].chValue == 1);
  remoteInfo.keyboard.key_S       = (remote_->remoteData[CRc_DR16::CH_KEY_S].chValue == 1);
  remoteInfo.keyboard.key_D       = (remote_->remoteData[CRc_DR16::CH_KEY_D].chValue == 1);
  remoteInfo.keyboard.key_Q       = (remote_->remoteData[CRc_DR16::CH_KEY_Q].chValue == 1);
  remoteInfo.keyboard.key_E       = (remote_->remoteData[CRc_DR16::CH_KEY_E].chValue == 1);
  remoteInfo.keyboard.key_R       = (remote_->remoteData[CRc_DR16::CH_KEY_R].chValue == 1);
  remoteInfo.keyboard.key_F       = (remote_->remoteData[CRc_DR16::CH_KEY_F].chValue == 1);
  remoteInfo.keyboard.key_G       = (remote_->remoteData[CRc_DR16::CH_KEY_G].chValue == 1);
  remoteInfo.keyboard.key_Z       = (remote_->remoteData[CRc_DR16::CH_KEY_Z].chValue == 1);
  remoteInfo.keyboard.key_X       = (remote_->remoteData[CRc_DR16::CH_KEY_X].chValue == 1);
  remoteInfo.keyboard.key_C       = (remote_->remoteData[CRc_DR16::CH_KEY_C].chValue == 1);
  remoteInfo.keyboard.key_V       = (remote_->remoteData[CRc_DR16::CH_KEY_V].chValue == 1);
  remoteInfo.keyboard.key_B       = (remote_->remoteData[CRc_DR16::CH_KEY_B].chValue == 1);
  remoteInfo.keyboard.key_Ctrl    = (remote_->remoteData[CRc_DR16::CH_KEY_CTRL].chValue == 1);
  remoteInfo.keyboard.key_Shift   = (remote_->remoteData[CRc_DR16::CH_KEY_SHIFT].chValue == 1);

  return RP_OK;
}

} // namespace robotpilots
