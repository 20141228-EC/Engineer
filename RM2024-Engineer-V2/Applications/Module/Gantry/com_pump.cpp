/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-11
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-11   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "mod_gantry.hpp"

namespace robotpilots {

/**
 * @brief
 * @param param
 * @return
 */
ERpStatus CModGantry::CComPump::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto gantryParam = static_cast<SModGantryInitParam &>(param);
  pumpGpioPort[L] = gantryParam.pumpGpioPort_L;
  pumpGpioPort[C] = gantryParam.pumpGpioPort_C;
  pumpGpioPort[R] = gantryParam.pumpGpioPort_R;
  pumpGpioPin[L] = gantryParam.pumpGpioPin_L;
  pumpGpioPin[C] = gantryParam.pumpGpioPin_C;
  pumpGpioPin[R] = gantryParam.pumpGpioPin_R;

  /* Set Component Flags */
  processFlag_ = 0;
  componentState = RP_OK;

  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModGantry::CComPump::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* Update Info */
  pumpInfo.isPumpOn_L =
    HAL_GPIO_ReadPin(pumpGpioPort[L], pumpGpioPin[L]);
  pumpInfo.isPumpOn_C =
    HAL_GPIO_ReadPin(pumpGpioPort[C], pumpGpioPin[C]);
  pumpInfo.isPumpOn_R =
    HAL_GPIO_ReadPin(pumpGpioPort[R], pumpGpioPin[R]);

  /* FSM Status Enum */
  enum {
    PUMP_RESET = 0,
    PUMP_INIT  = 1,
    PUMP_CTRL,
  };

  /* Gantry Pump Component Control FSM */
  switch (processFlag_) {

    case PUMP_RESET: {

      HAL_GPIO_WritePin(pumpGpioPort[L], pumpGpioPin[L], GPIO_PIN_RESET);
      HAL_GPIO_WritePin(pumpGpioPort[C], pumpGpioPin[C], GPIO_PIN_RESET);
      HAL_GPIO_WritePin(pumpGpioPort[R], pumpGpioPin[R], GPIO_PIN_RESET);

      return RP_OK;
    }

    case PUMP_INIT: {

      pumpCmd = SPumpCmd();
      componentState = RP_OK;
      processFlag_ = PUMP_CTRL;
      return RP_OK;
    }

    case PUMP_CTRL: {

      if (pumpCmd.setPumpOn_L != pumpInfo.isPumpOn_L)
        HAL_GPIO_WritePin(pumpGpioPort[L], pumpGpioPin[L], (pumpCmd.setPumpOn_L) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      if (pumpCmd.setPumpOn_C != pumpInfo.isPumpOn_C)
        HAL_GPIO_WritePin(pumpGpioPort[C], pumpGpioPin[C], (pumpCmd.setPumpOn_C) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      if (pumpCmd.setPumpOn_R != pumpInfo.isPumpOn_R)
        HAL_GPIO_WritePin(pumpGpioPort[R], pumpGpioPin[R], (pumpCmd.setPumpOn_R) ? GPIO_PIN_SET : GPIO_PIN_RESET);

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
