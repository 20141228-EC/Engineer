/**
 * @file        com_traverse.cpp
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
 * @brief
 * @param param
 * @return
 */
ERpStatus CModGantry::CComTraverse::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto gantryParam = static_cast<SModGantryInitParam &>(param);
  motor[0] = MotorMap.at(gantryParam.traverseMotorID);

  /* Initialize PID Controllers */
  gantryParam.traversePosPidParam.threadNum = 1;
  pidPosCtrl.InitAlgorithm(&gantryParam.traversePosPidParam);

  gantryParam.traverseSpdPidParam.threadNum = 1;
  pidSpdCtrl.InitAlgorithm(&gantryParam.traverseSpdPidParam);

  /* Clear Motor Output Buffer */
  traversePos.resize(1);
  traversePosMeasure.resize(1);
  traverseSpd.resize(1);
  traverseSpdMeasure.resize(1);
  output.resize(1);
  mtrOutputBuffer.fill(0);

  /* Set Component Flags */
  processFlag_ = 0;
  componentState = RP_OK;

  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModGantry::CComTraverse::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* Update Info */
  traverseInfo.posit = -motor[0]->motorData[CMtrInstance::DATA_POSIT];
  traverseInfo.isPositArrived = (abs(traverseCmd.setPosit - traverseInfo.posit) < 8192 * 2);

  /* FSM Status Enum */
  enum {
    TRAVERSE_RESET    = 0,
    TRAVERSE_PRE_INIT = 1,
    TRAVERSE_INIT,
    TRAVERSE_CTRL,
  };

  /* Gantry Traverse Component Control FSM */
  switch (processFlag_) {

    case TRAVERSE_RESET: {
      mtrOutputBuffer.fill(0);
      return RP_OK;
    }

    case TRAVERSE_PRE_INIT: {
      traverseCmd.setPosit = static_cast<int32_t>(rangeLimit * 1.2);
      motor[0]->motorData[CMtrInstance::DATA_POSIT] = -traverseCmd.setPosit;
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      processFlag_ = TRAVERSE_INIT;
      return RP_OK;
    }

    case TRAVERSE_INIT: {
      if (motor[0]->motorState == CMtrInstance::EMotorStatus::STALL) {
        traverseCmd.setPosit = 0;
        motor[0]->motorData[CMtrInstance::DATA_POSIT] = 8192 * 1.0;
        pidPosCtrl.ResetAlgorithm();
        pidSpdCtrl.ResetAlgorithm();
        componentState = RP_OK;
        processFlag_ = TRAVERSE_CTRL;
        return RP_OK;
      }
      traverseCmd.setPosit -= 400;   // Reset Speed
      return _UpdateOutput(static_cast<float_t>(traverseCmd.setPosit));
    }

    case TRAVERSE_CTRL: {
      traverseCmd.setPosit = std::clamp(traverseCmd.setPosit, 0l, rangeLimit);
      return _UpdateOutput(static_cast<float_t>(traverseCmd.setPosit));
    }

    default: {
      StopComponent();
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      componentState = RP_ERROR;
      return RP_ERROR;
    }
  }
}


/**
 * @brief Convert physical position to motor position
 * @param phyPosit[in] Physical position
 * @return Motor position
 */
int32_t CModGantry::CComTraverse::PhyPositToMtrPosit(float_t phyPosit) {

  const int32_t zeroOffset = 330000;
  const float_t scale = 3666.67f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief Convert motor position to physical position
 * @param mtrPosit[in] Motor position
 * @return Physical position
 */
float_t CModGantry::CComTraverse::MtrPositToPhyPosit(int32_t mtrPosit) {

  const int32_t zeroOffset = 330000;
  const float_t scale = 3666.67f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param posit
 * @return
 */
ERpStatus CModGantry::CComTraverse::_UpdateOutput(float_t posit) {

  traversePos[0] = -posit;
  traversePosMeasure[0] = static_cast<float_t>(motor[0]->motorData[CMtrInstance::DATA_POSIT]);

  pidPosCtrl.UpdatePidController(traversePos, traversePosMeasure, traverseSpd);

  traverseSpdMeasure[0] = static_cast<float_t>(motor[0]->motorData[CMtrInstance::DATA_SPEED]);

  pidSpdCtrl.UpdatePidController(traverseSpd, traverseSpdMeasure, output);

  mtrOutputBuffer = {
    static_cast<int16_t>(output[0]),
  };

  return RP_OK;
}

} // namespace robotpilots
