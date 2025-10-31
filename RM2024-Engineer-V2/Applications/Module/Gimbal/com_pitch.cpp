/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-15
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-15   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "mod_gimbal.hpp"

namespace robotpilots {

/**
 * @brief
 * @param pStruct
 * @return
 */
ERpStatus CModGimbal::CComPitch::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto &gimbalParam = reinterpret_cast<SModGimbalInitParam &>(param);
  halTimHandle      = gimbalParam.gimbalPitchHalTimHandle;
  timChannel        = gimbalParam.gimbalPitchTimChannel;

  /* Set Component Flags */
  processFlag_   = 0;
  componentState = RP_OK;

  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModGimbal::CComPitch::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* FSM Status Enum */
  enum {
    PITCH_RESET = 0,
    PITCH_INIT  = 1,
    PITCH_CTRL,
  };

  /* Gimbal Pitch Component Control FSM */
  switch (processFlag_) {

    case PITCH_RESET: {
      HAL_TIM_PWM_Stop(halTimHandle, timChannel);
      return RP_OK;
    }

    case PITCH_INIT: {
      HAL_TIM_PWM_Start(halTimHandle, timChannel);
      componentState = RP_OK;
      processFlag_ = PITCH_CTRL;
      return RP_OK;
    }

    case PITCH_CTRL: {
      if (pitchCmd.setStep == 0)        // Middle
        __HAL_TIM_SET_COMPARE(halTimHandle, timChannel, 80);
      else if (pitchCmd.setStep == 1)   // Up
        __HAL_TIM_SET_COMPARE(halTimHandle, timChannel, 55);
      else if (pitchCmd.setStep == 2)   // Down
        __HAL_TIM_SET_COMPARE(halTimHandle, timChannel, 105);
      else if (pitchCmd.setStep == 3)   // Vision
        __HAL_TIM_SET_COMPARE(halTimHandle, timChannel, 70);
      return RP_OK;
    }

    default: {
      StopComponent();
      componentState = RP_ERROR;
      return RP_ERROR;
    }
  }
}

} // namespace robotpilots
