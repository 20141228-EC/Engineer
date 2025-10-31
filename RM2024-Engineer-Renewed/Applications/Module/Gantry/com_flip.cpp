/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-22
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-22   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "mod_gantry.hpp"

namespace robotpilots {

/**
 * @brief
 * @param param
 * @return
 */
ERpStatus CModGantry::CComFlip::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto gantryParam = static_cast<SModGantryInitParam &>(param);
  motor[F] = MotorMap.at(gantryParam.flipMotorID_F);
  motor[B] = MotorMap.at(gantryParam.flipMotorID_B);

  /* Initialize PID Controllers */
  gantryParam.flipSpdPidParam.threadNum = 2;
  pidSpdCtrl.InitAlgorithm(&gantryParam.flipSpdPidParam);

  /* Clear Motor Output Buffer */
  flipSpd.resize(2);
  flipSpdMeasure.resize(2);
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
ERpStatus CModGantry::CComFlip::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* Update Info */

  /* FSM Status Enum */
  enum {
    FLIP_RESET = 0,
    FLIP_INIT  = 1,
    FLIP_CTRL,
  };

  /* Gantry End Component Control FSM */
  switch (processFlag_) {

    case FLIP_RESET: {

      mtrOutputBuffer.fill(0);
      return RP_OK;
    }

    case FLIP_INIT: {

      mtrOutputBuffer.fill(0);
      pidSpdCtrl.ResetAlgorithm();

      processFlag_ = FLIP_CTRL;
      componentState = RP_OK;
      return RP_OK;
    }

    case FLIP_CTRL: {

      flipCmd.setSpeed_Y =
        std::clamp(flipCmd.setSpeed_Y, -100l, 100l);
      flipCmd.setSpeed_W =
        std::clamp(flipCmd.setSpeed_W, -100l, 100l);
      return _UpdateOutput(static_cast<float_t>(flipCmd.setSpeed_Y * 20),
                           static_cast<float_t>(flipCmd.setSpeed_W * 20));
    }

    default: {

      StopComponent();
      mtrOutputBuffer.fill(0);
      pidSpdCtrl.ResetAlgorithm();
      componentState = RP_ERROR;
      return RP_ERROR;
    }
  }
}


/**
 * @brief
 * @param speed_Y
 * @param speed_W
 * @return
 */
ERpStatus CModGantry::CComFlip::_UpdateOutput(float_t speed_Y,
                                              float_t speed_W) {

  speed_Y += abs(speed_W) * 0.4f;

  flipSpd[F] = speed_W - speed_Y;
  flipSpd[B] = speed_W + speed_Y;
  flipSpdMeasure[F] = static_cast<float_t>(motor[F]->motorData[CMtrInstance::DATA_SPEED]);
  flipSpdMeasure[B] = static_cast<float_t>(motor[B]->motorData[CMtrInstance::DATA_SPEED]);

  pidSpdCtrl.UpdatePidController(flipSpd, flipSpdMeasure, output);

  mtrOutputBuffer = {
    static_cast<int16_t>(output[F]),
    static_cast<int16_t>(output[B]),
  };

  return RP_OK;
}

} // namespace robotpilots