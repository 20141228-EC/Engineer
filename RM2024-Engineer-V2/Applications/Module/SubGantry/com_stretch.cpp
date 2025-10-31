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

#include "mod_subgantry.hpp"

namespace robotpilots {

/**
 * @brief
 * @param param
 * @return
 */
ERpStatus CModSubGantry::CComStretch::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto &subGantryParam = reinterpret_cast<SModSubGantryInitParam &>(param);
  motor[L]             = MotorMap.at(subGantryParam.stretchMotorID_L);
  motor[R]             = MotorMap.at(subGantryParam.stretchMotorID_R);

  /* Initialize PID Controllers */
  subGantryParam.stretchPosPidParam.threadNum = 2;
  pidPosCtrl.InitAlgorithm(&subGantryParam.stretchPosPidParam);

  subGantryParam.stretchSpdPidParam.threadNum = 2;
  pidSpdCtrl.InitAlgorithm(&subGantryParam.stretchSpdPidParam);

  /* Clear Motor Output Buffer */
  mtrOutputBuffer.fill(0);

  /* Set Component Flags */
  processFlag_   = 0;
  componentState = RP_OK;

  return RP_OK;
}


/**
 * @brief
 */
ERpStatus CModSubGantry::CComStretch::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* Update Info */
  stretchInfo.posit_L =  motor[L]->motorData[CMtrInstance::DATA_POSIT];
  stretchInfo.posit_R = -motor[R]->motorData[CMtrInstance::DATA_POSIT];
  stretchInfo.isPositArrived_L = (abs(stretchCmd.setPosit_L - stretchInfo.posit_L) < 8192);
  stretchInfo.isPositArrived_R = (abs(stretchCmd.setPosit_R - stretchInfo.posit_R) < 8192);

  /* FSM Status Enum */
  enum {
    STRETCH_RESET = 0,
    STRETCH_PRE_INIT = 1,
    STRETCH_INIT,
    STRETCH_CTRL,
  };

  /* Gantry Lift Component FSM */
  switch (processFlag_) {

    case STRETCH_RESET: {
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      return RP_OK;
    }

    case STRETCH_PRE_INIT: {
      stretchCmd.setPosit_L                            = static_cast<int32_t>(rangeLimit * 1.2);
      stretchCmd.setPosit_R                            = static_cast<int32_t>(rangeLimit * 1.2);
      motor[L]->motorData[CMtrInstance::DATA_POSIT] =  stretchCmd.setPosit_L;
      motor[R]->motorData[CMtrInstance::DATA_POSIT] = -stretchCmd.setPosit_R;
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      processFlag_ = STRETCH_INIT;
      return RP_OK;
    }

    case STRETCH_INIT: {
      if (motor[L]->motorState == CMtrInstance::EMotorStatus::STALL
          && motor[R]->motorState == CMtrInstance::EMotorStatus::STALL) {
        stretchCmd = SStretchCmd();
        motor[L]->motorData[CMtrInstance::DATA_POSIT] = -static_cast<int32_t>(8192 * 0.5);
        motor[R]->motorData[CMtrInstance::DATA_POSIT] =  static_cast<int32_t>(8192 * 0.5);
        pidPosCtrl.ResetAlgorithm();
        pidSpdCtrl.ResetAlgorithm();
        componentState = RP_OK;
        processFlag_ = STRETCH_CTRL;
        return RP_OK;
      }
      stretchCmd.setPosit_L -= 200;// Reset Speed
      stretchCmd.setPosit_R -= 200;// Reset Speed
      return _UpdateOutput(static_cast<float_t>(stretchCmd.setPosit_L),
                           static_cast<float_t>(stretchCmd.setPosit_R));
    }

    case STRETCH_CTRL: {
      stretchCmd.setPosit_L = std::clamp(stretchCmd.setPosit_L, 0l, rangeLimit);
      stretchCmd.setPosit_R = std::clamp(stretchCmd.setPosit_R, 0l, rangeLimit);
      return _UpdateOutput(static_cast<float_t>(stretchCmd.setPosit_L),
                           static_cast<float_t>(stretchCmd.setPosit_R));
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
 * @param phyPosit
 * @return
 */
int32_t CModSubGantry::CComStretch::PhyPositToMtrPosit(float_t phyPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale      = 3650.00f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief Convert motor position to physical position
 * @param mtrPosit
 * @return
 */
float_t CModSubGantry::CComStretch::MtrPositToPhyPosit(int32_t mtrPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale      = 3650.00f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param posit_L
 * @param posit_R
 * @return
 */
ERpStatus CModSubGantry::CComStretch::_UpdateOutput(float_t posit_L, float_t posit_R) {

  DataBuffer<float_t> liftPos = {
    static_cast<float_t>( posit_L),
    static_cast<float_t>(-posit_R),
  };

  DataBuffer<float_t> liftPosMeasure = {
    static_cast<float_t>(motor[L]->motorData[CMtrInstance::DATA_POSIT]),
    static_cast<float_t>(motor[R]->motorData[CMtrInstance::DATA_POSIT]),
  };

  auto liftSpd =
    pidPosCtrl.UpdatePidController(liftPos, liftPosMeasure);

  DataBuffer<float_t> liftSpdMeasure = {
    static_cast<float_t>(motor[L]->motorData[CMtrInstance::DATA_SPEED]),
    static_cast<float_t>(motor[R]->motorData[CMtrInstance::DATA_SPEED]),
  };

  auto output =
    pidSpdCtrl.UpdatePidController(liftSpd, liftSpdMeasure);

  mtrOutputBuffer = {
    static_cast<int16_t>(output[L]),
    static_cast<int16_t>(output[R]),
  };

  return RP_OK;
}

} // namespace robotpilots
