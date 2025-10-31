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

#include "mod_gantry.hpp"

namespace robotpilots {

/**
 * @brief
 * @param param
 * @return
 */
ERpStatus CModGantry::CComStretch::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto gantryParam = static_cast<SModGantryInitParam &>(param);
  motor[L] = MotorMap.at(gantryParam.stretchMotorID_L);
  motor[R] = MotorMap.at(gantryParam.stretchMotorID_R);

  /* Initialize PID Controllers */
  gantryParam.stretchPosPidParam.threadNum = 2;
  pidPosCtrl.InitAlgorithm(&gantryParam.stretchPosPidParam);

  gantryParam.stretchSpdPidParam.threadNum = 2;
  pidSpdCtrl.InitAlgorithm(&gantryParam.stretchSpdPidParam);

  /* Clear Motor Output Buffer */
  stretchPos.resize(2);
  stretchPosMeasure.resize(2);
  stretchSpd.resize(2);
  stretchSpdMeasure.resize(2);
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
ERpStatus CModGantry::CComStretch::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* Update Info */
  stretchInfo.posit =
    (motor[R]->motorData[CMtrInstance::DATA_POSIT] - motor[L]->motorData[CMtrInstance::DATA_POSIT]) / 2;
  stretchInfo.isPositArrived = (abs(stretchCmd.setPosit - stretchInfo.posit) < 8192 * 2);

  /* Control Process */
  switch (processFlag_) {

    case 0: {   // Lift Reset
      mtrOutputBuffer.fill(0);
      return RP_OK;
    }

    case 1: {   // Lift Pre-Init
      stretchCmd.setPosit = static_cast<int32_t>(rangeLimit * 1.2);
      motor[L]->motorData[CMtrInstance::DATA_POSIT] = -stretchCmd.setPosit;
      motor[R]->motorData[CMtrInstance::DATA_POSIT] =  stretchCmd.setPosit;
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      processFlag_++;
      return RP_OK;
    }

    case 2: {   // Lift Init
      if (motor[L]->motorState == CMtrInstance::EMotorStatus::STALL
          && motor[R]->motorState == CMtrInstance::EMotorStatus::STALL) {
        stretchCmd.setPosit = 0;
        motor[L]->motorData[CMtrInstance::DATA_POSIT] =  8192 * 1.0;
        motor[R]->motorData[CMtrInstance::DATA_POSIT] = -8192 * 1.0;
        pidPosCtrl.ResetAlgorithm();
        pidSpdCtrl.ResetAlgorithm();
        componentState = RP_OK;
        processFlag_++;
        return RP_OK;
      }
      stretchCmd.setPosit -= 200;
      return _UpdateOutput(static_cast<float_t>(stretchCmd.setPosit));
    }

    case 3: {   // Lift Control
      stretchCmd.setPosit = std::clamp(stretchCmd.setPosit, 0l, rangeLimit);
      return _UpdateOutput(static_cast<float_t>(stretchCmd.setPosit));
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
int32_t CModGantry::CComStretch::PhyPositToMtrPosit(float_t phyPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 1738.10f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief Convert motor position to physical position
 * @param mtrPosit[in] Motor position
 * @return Physical position
 */
float_t CModGantry::CComStretch::MtrPositToPhyPosit(int32_t mtrPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 1738.10f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param posit
 * @return
 */
ERpStatus CModGantry::CComStretch::_UpdateOutput(float_t posit) {

  stretchPos[L] = static_cast<float_t>(-posit);
  stretchPos[R] = static_cast<float_t>( posit);
  stretchPosMeasure[L] = static_cast<float_t>(motor[L]->motorData[CMtrInstance::DATA_POSIT]);
  stretchPosMeasure[R] = static_cast<float_t>(motor[R]->motorData[CMtrInstance::DATA_POSIT]);

  pidPosCtrl.UpdatePidController(stretchPos, stretchPosMeasure, stretchSpd);

  stretchSpdMeasure[L] = static_cast<float_t>(motor[L]->motorData[CMtrInstance::DATA_SPEED]);
  stretchSpdMeasure[R] = static_cast<float_t>(motor[R]->motorData[CMtrInstance::DATA_SPEED]);

  pidSpdCtrl.UpdatePidController(stretchSpd, stretchSpdMeasure, output);

  mtrOutputBuffer = {
    static_cast<int16_t>(output[L]),
    static_cast<int16_t>(output[R]),
  };

  return RP_OK;
}

} // namespace robotpilots
