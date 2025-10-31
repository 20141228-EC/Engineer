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
  pumpGpioPort[0] = gantryParam.pumpGpioPort_1;
  pumpGpioPin[0] = gantryParam.pumpGpioPin_1;
  pumpGpioPort[1] = gantryParam.pumpGpioPort_2;
  pumpGpioPin[1] = gantryParam.pumpGpioPin_2;

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
  pumpInfo.isPumpOn =
    HAL_GPIO_ReadPin(pumpGpioPort[0], pumpGpioPin[0]);

  /* FSM Status Enum */
  enum {
    PUMP_RESET = 0,
    PUMP_INIT  = 1,
    PUMP_CTRL,
  };

  /* Gantry Pump Component Control FSM */
  switch (processFlag_) {

    case PUMP_RESET: {

      HAL_GPIO_WritePin(pumpGpioPort[0], pumpGpioPin[0], GPIO_PIN_RESET);
      HAL_GPIO_WritePin(pumpGpioPort[1], pumpGpioPin[1], GPIO_PIN_RESET);

      return RP_OK;
    }

    case PUMP_INIT: {

      pumpCmd = SPumpCmd();
      componentState = RP_OK;
      processFlag_ = PUMP_CTRL;
      return RP_OK;
    }

    case PUMP_CTRL: {

      if (pumpCmd.setPumpOn != pumpInfo.isPumpOn) {
        HAL_GPIO_WritePin(pumpGpioPort[0], pumpGpioPin[0], (pumpCmd.setPumpOn) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pumpGpioPort[1], pumpGpioPin[1], (pumpCmd.setPumpOn) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }

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
