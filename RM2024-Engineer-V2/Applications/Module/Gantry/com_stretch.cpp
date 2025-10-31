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
  motor[0] = MotorMap.at(gantryParam.stretchMotorID);

  /* Initialize PID Controllers */
  gantryParam.stretchPosPidParam.threadNum = 1;
  gantryParam.stretchPosPidParam.tickRate = 500;
  pidPosCtrl.InitAlgorithm(&gantryParam.stretchPosPidParam);

  gantryParam.stretchSpdPidParam.threadNum = 1;
  gantryParam.stretchSpdPidParam.tickRate = 500;
  pidSpdCtrl.InitAlgorithm(&gantryParam.stretchSpdPidParam);

  /* Clear Motor Output Buffer */
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
  stretchInfo.posit = -motor[0]->motorData[CMtrInstance::DATA_POSIT];
  stretchInfo.isPositArrived = (abs(stretchCmd.setPosit - stretchInfo.posit) < 8192);

  /* Control Process */
  switch (processFlag_) {

    case 0: {   // Lift Reset
      mtrOutputBuffer.fill(0);
      return RP_OK;
    }

    case 1: {   // Lift Pre-Init
      stretchCmd.setPosit = static_cast<int32_t>(rangeLimit * 1.2);
      motor[0]->motorData[CMtrInstance::DATA_POSIT] = -stretchCmd.setPosit;
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      processFlag_++;
      return RP_OK;
    }

    case 2: {   // Lift Init
      if (motor[0]->motorState == CMtrInstance::EMotorStatus::STALL) {
        stretchCmd.setPosit = 0;
        motor[0]->motorData[CMtrInstance::DATA_POSIT] = 8192 * 1.0;
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
  const float_t scale = 1060.97f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief Convert motor position to physical position
 * @param mtrPosit[in] Motor position
 * @return Physical position
 */
float_t CModGantry::CComStretch::MtrPositToPhyPosit(int32_t mtrPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 1060.97f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param posit
 * @return
 */
ERpStatus CModGantry::CComStretch::_UpdateOutput(float_t posit) {

  DataBuffer<float_t> liftPos = {
      static_cast<float_t>(-posit),
  };

  DataBuffer<float_t> liftPosMeasure = {
      static_cast<float_t>(motor[0]->motorData[CMtrInstance::DATA_POSIT]),
  };

  auto liftSpd =
      pidPosCtrl.UpdatePidController(liftPos, liftPosMeasure);

  DataBuffer<float_t> liftSpdMeasure = {
      static_cast<float_t>(motor[0]->motorData[CMtrInstance::DATA_SPEED]),
  };

  auto output =
      pidSpdCtrl.UpdatePidController(liftSpd, liftSpdMeasure);

  mtrOutputBuffer = {
      static_cast<int16_t>(output[0]),
  };

  return RP_OK;
}

} // namespace robotpilots
