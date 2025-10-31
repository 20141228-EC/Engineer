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
ERpStatus CModGantry::CComLift::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto gantryParam = reinterpret_cast<SModGantryInitParam &>(param);
  motor[L] = MotorMap.at(gantryParam.liftMotorID_L);
  motor[R] = MotorMap.at(gantryParam.liftMotorID_R);

  /* Initialize PID Controllers */
  gantryParam.liftPosPidParam.threadNum = 2;
  pidPosCtrl.InitAlgorithm(&gantryParam.liftPosPidParam);

  gantryParam.liftSpdPidParam.threadNum = 2;
  pidSpdCtrl.InitAlgorithm(&gantryParam.liftSpdPidParam);

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
ERpStatus CModGantry::CComLift::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* Update Info */
  liftInfo.posit =
      (motor[R]->motorData[CMtrInstance::DATA_POSIT] - motor[L]->motorData[CMtrInstance::DATA_POSIT]) / 2;
  liftInfo.isPositArrived = (abs(liftCmd.setPosit - liftInfo.posit) < 8192 * 3);

  /* Gantry Lift Component FSM */
  switch (processFlag_) {

    case 0: {   // Lift Reset
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      return RP_OK;
    }

    case 1: {   // Lift Pre-Init
      liftCmd.setPosit = static_cast<int32_t>(rangeLimit * 1.2);
      motor[L]->motorData[CMtrInstance::DATA_POSIT] = -liftCmd.setPosit;
      motor[R]->motorData[CMtrInstance::DATA_POSIT] =  liftCmd.setPosit;
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      processFlag_++;
      return RP_OK;
    }

    case 2: {   // Lift Init
      if (motor[L]->motorState == CMtrInstance::EMotorStatus::STALL
          && motor[R]->motorState == CMtrInstance::EMotorStatus::STALL) {
        liftCmd.setPosit = 0;
        motor[L]->motorData[CMtrInstance::DATA_POSIT] = static_cast<int32_t>( 8192 * 0.1);
        motor[R]->motorData[CMtrInstance::DATA_POSIT] = static_cast<int32_t>(-8192 * 0.1);
        pidPosCtrl.ResetAlgorithm();
        pidSpdCtrl.ResetAlgorithm();
        componentState = RP_OK;
        processFlag_++;
        return RP_OK;
      }
      liftCmd.setPosit -= 50;   // Reset Speed
      return _UpdateOutput(static_cast<float_t>(liftCmd.setPosit));
    }

    case 3: {   // Lift Control
      liftCmd.setPosit = std::clamp(liftCmd.setPosit, 0l, rangeLimit);
      return _UpdateOutput(static_cast<float_t>(liftCmd.setPosit));
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
int32_t CModGantry::CComLift::PhyPositToMtrPosit(float_t phyPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 924.242f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief Convert motor position to physical position
 * @param mtrPosit[in] Motor position
 * @return Physical position
 */
float_t CModGantry::CComLift::MtrPositToPhyPosit(int32_t mtrPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 924.242f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param posit
 * @return
 */
ERpStatus CModGantry::CComLift::_UpdateOutput(float_t posit) {

  DataBuffer<float_t> liftPos = {
    static_cast<float_t>(-posit),
    static_cast<float_t>( posit),
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
