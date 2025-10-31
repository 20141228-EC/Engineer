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
ERpStatus CModGantry::CComEnd::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto gantryParam = static_cast<SModGantryInitParam &>(param);
  motor[L] = MotorMap.at(gantryParam.endMotorID_L);
  motor[R] = MotorMap.at(gantryParam.endMotorID_R);

  /* Initialize PID Controllers */
  gantryParam.endPosPidParam.threadNum = 2;
  pidPosCtrl.InitAlgorithm(&gantryParam.endPosPidParam);

  gantryParam.endSpdPidParam.threadNum = 2;
  pidSpdCtrl.InitAlgorithm(&gantryParam.endSpdPidParam);

  /* Clear Motor Output Buffer */
  endPos.resize(2);
  endPosMeasure.resize(2);
  endSpd.resize(2);
  endSpdMeasure.resize(2);
  output.resize(2);
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
ERpStatus CModGantry::CComEnd::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* Update Info */
  endInfo.posit_Roll =
      (motor[L]->motorData[CMtrInstance::DATA_POSIT] + motor[R]->motorData[CMtrInstance::DATA_POSIT]) / 2;
  endInfo.posit_Pitch =
      ((motor[R]->motorData[CMtrInstance::DATA_POSIT] - endInfo.posit_Roll) - (motor[L]->motorData[CMtrInstance::DATA_POSIT] - endInfo.posit_Roll)) / 2;
  endInfo.isPositArrived_Pitch = (abs(endCmd.setPosit_Pitch - endInfo.posit_Pitch) < 8192 * 2);
  endInfo.isPositArrived_Roll = (abs(endCmd.setPosit_Roll - endInfo.posit_Roll) < 8192 * 2);

  /* FSM Status Enum */
  enum {
    END_RESET    = 0,
    END_PRE_INIT = 1,
    END_INIT,
    END_CTRL,
  };

  /* Gantry End Component Control FSM */
  switch (processFlag_) {

    case END_RESET: {
      mtrOutputBuffer.fill(0);
      return RP_OK;
    }

    case END_PRE_INIT: {
      endCmd.setPosit_Pitch = rangeLimit * 1.2f;
      motor[L]->motorData[CMtrInstance::DATA_POSIT] =  rangeLimit;
      motor[R]->motorData[CMtrInstance::DATA_POSIT] = -rangeLimit;
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      processFlag_ = END_INIT;
      return RP_OK;
    }

    case END_INIT: {
      if (motor[L]->motorState == CMtrInstance::EMotorStatus::STALL
          && motor[R]->motorState == CMtrInstance::EMotorStatus::STALL) {
        endCmd.setPosit_Pitch = 0;
        endCmd.setPosit_Roll = 0;
        motor[L]->motorData[CMtrInstance::DATA_POSIT] = -8192 * 1.5;
        motor[R]->motorData[CMtrInstance::DATA_POSIT] =  8192 * 1.5;
        pidPosCtrl.ResetAlgorithm();
        pidSpdCtrl.ResetAlgorithm();
        componentState = RP_OK;
        processFlag_ = END_CTRL;
        return RP_OK;
      }
      endCmd.setPosit_Pitch -= 200;    // Reset Speed
      return _UpdateOutput(static_cast<float_t>(endCmd.setPosit_Pitch), 0);
    }

    case END_CTRL: {
      endCmd.setPosit_Pitch = std::clamp(endCmd.setPosit_Pitch, 0l, rangeLimit);
      return _UpdateOutput(static_cast<float_t>(endCmd.setPosit_Pitch),
                           static_cast<float_t>(endCmd.setPosit_Roll));
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
 * @brief
 * @param phyPosit
 * @return
 */
int32_t CModGantry::CComEnd::PhyPositToMtrPosit_Pitch(float_t phyPosit) {

  const int32_t zeroOffset = 144450;
  const float_t scale = -1141.09f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief
 * @param mtrPosit
 * @return
 */
float_t CModGantry::CComEnd::MtrPositToPhyPosit_Pitch(int32_t mtrPosit) {

  const int32_t zeroOffset = 144450;
  const float_t scale = -1141.09f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param phyPosit
 * @return
 */
int32_t CModGantry::CComEnd::PhyPositToMtrPosit_Roll(float_t phyPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 2200.0f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief
 * @param mtrPosit
 * @return
 */
float_t CModGantry::CComEnd::MtrPositToPhyPosit_Roll(int32_t mtrPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 2200.0f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param posit
 * @return
 */
ERpStatus CModGantry::CComEnd::_UpdateOutput(float_t posit_Pitch,
                                             float_t posit_Roll) {

  endPos[L] = posit_Roll + posit_Pitch;
  endPos[R] = posit_Roll - posit_Pitch;
  endPosMeasure[L] = static_cast<float_t>(motor[L]->motorData[CMtrInstance::DATA_POSIT]);
  endPosMeasure[R] = static_cast<float_t>(motor[R]->motorData[CMtrInstance::DATA_POSIT]);

  pidPosCtrl.UpdatePidController(endPos, endPosMeasure, endSpd);

  endSpdMeasure[L] = static_cast<float_t>(motor[L]->motorData[CMtrInstance::DATA_SPEED]);
  endSpdMeasure[R] = static_cast<float_t>(motor[R]->motorData[CMtrInstance::DATA_SPEED]);

  pidSpdCtrl.UpdatePidController(endSpd, endSpdMeasure, output);

  mtrOutputBuffer = {
    static_cast<int16_t>(output[L]),
    static_cast<int16_t>(output[R]),
  };

  return RP_OK;
}

} // namespace robotpilots
