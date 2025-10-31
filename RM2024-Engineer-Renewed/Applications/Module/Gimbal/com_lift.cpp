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

#include "mod_gimbal.hpp"

namespace robotpilots {

/**
 * @brief
 * @param param
 * @return
 */
ERpStatus CModGimbal::CComLift::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto gimbalParam = static_cast<SModGimbalInitParam &>(param);
  motor[L] = MotorMap.at(gimbalParam.liftMotorID_L);
  motor[R] = MotorMap.at(gimbalParam.liftMotorID_R);

  /* Initialize PID Controllers */
  gimbalParam.liftPosPidParam.threadNum = 2;
  pidPosCtrl.InitAlgorithm(&gimbalParam.liftPosPidParam);

  gimbalParam.liftSpdPidParam.threadNum = 2;
  pidSpdCtrl.InitAlgorithm(&gimbalParam.liftSpdPidParam);

  /* Clear Motor Output Buffer */
  liftPos.resize(2);
  liftPosMeasure.resize(2);
  liftSpd.resize(2);
  liftSpdMeasure.resize(2);
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
ERpStatus CModGimbal::CComLift::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* Update Info */
  liftInfo.posit =
    (motor[L]->motorData[CMtrInstance::DATA_POSIT] - motor[R]->motorData[CMtrInstance::DATA_POSIT]) / 2;
  liftInfo.isPositArrived = (abs(liftCmd.setPosit - liftInfo.posit) < 8192);

  /* FSM Status Enum */
  enum {
    LIFT_RESET       = 0,
    LIFT_PRE_INIT = 1,
    LIFT_INIT,
    LIFT_CTRL,
  };

  /* FSM */
  switch (processFlag_) {

    case LIFT_RESET: {

      mtrOutputBuffer.fill(0);
      return RP_OK;
    }

    case LIFT_PRE_INIT: {

      liftCmd.setPosit = static_cast<int32_t>(rangeLimit * 1.2);
      motor[L]->motorData[CMtrInstance::DATA_POSIT] =  liftCmd.setPosit;
      motor[R]->motorData[CMtrInstance::DATA_POSIT] = -liftCmd.setPosit;
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();

      processFlag_ = LIFT_INIT;   // Enter LIFT_INIT
      return RP_OK;
    }

    case LIFT_INIT: {

      if (motor[L]->motorState == CMtrInstance::EMotorStatus::STALL
          && motor[R]->motorState == CMtrInstance::EMotorStatus::STALL) {
        motor[L]->motorData[CMtrInstance::DATA_POSIT] = -8192 * 1.0;
        motor[R]->motorData[CMtrInstance::DATA_POSIT] =  8192 * 1.0;
        pidPosCtrl.ResetAlgorithm();
        pidSpdCtrl.ResetAlgorithm();

        liftCmd.setPosit = 0;
        componentState = RP_OK;
        processFlag_ = LIFT_CTRL;   // Enter LIFT_CTRL
        return RP_OK;
      }

      liftCmd.setPosit -= 200;   // Reset Speed
      return _UpdateOutput(static_cast<float_t>(liftCmd.setPosit));
    }

    case LIFT_CTRL: {

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
int32_t CModGimbal::CComLift::PhyPositToMtrPosit(float_t phyPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 3125.0f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief Convert motor position to physical position
 * @param mtrPosit[in] Motor position
 * @return Physical position
 */
float_t CModGimbal::CComLift::MtrPositToPhyPosit(int32_t mtrPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 3125.0f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param posit
 * @return
 */
ERpStatus CModGimbal::CComLift::_UpdateOutput(float_t posit) {

  liftPos[L] = static_cast<float_t>( posit);
  liftPos[R] = static_cast<float_t>(-posit);
  liftPosMeasure[L] = static_cast<float_t>(motor[L]->motorData[CMtrInstance::DATA_POSIT]);
  liftPosMeasure[R] = static_cast<float_t>(motor[R]->motorData[CMtrInstance::DATA_POSIT]);

  pidPosCtrl.UpdatePidController(liftPos, liftPosMeasure, liftSpd);

  liftSpdMeasure[L] = static_cast<float_t>(motor[L]->motorData[CMtrInstance::DATA_SPEED]);
  liftSpdMeasure[R] = static_cast<float_t>(motor[R]->motorData[CMtrInstance::DATA_SPEED]);

  pidSpdCtrl.UpdatePidController(liftSpd, liftSpdMeasure, output);

  mtrOutputBuffer = {
    static_cast<int16_t>(output[L]),
    static_cast<int16_t>(output[R]),
  };

  return RP_OK;
}

} // namespace robotpilots