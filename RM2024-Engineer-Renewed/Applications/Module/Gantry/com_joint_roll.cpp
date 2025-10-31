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
ERpStatus CModGantry::CComJointRoll::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto gantryParam = static_cast<SModGantryInitParam &>(param);
  motor[0] = MotorMap.at(gantryParam.jointMotorID_Roll);

  /* Initialize PID Controllers */
  gantryParam.jointPosPidParam.threadNum = 1;
  pidPosCtrl.InitAlgorithm(&gantryParam.jointPosPidParam);

  gantryParam.jointSpdPidParam.threadNum = 1;
  pidSpdCtrl.InitAlgorithm(&gantryParam.jointSpdPidParam);

  /* Clear Motor Output Buffer */
  jointRollPos.resize(1);
  jointRollPosMeasure.resize(1);
  jointRollSpd.resize(1);
  jointRollSpdMeasure.resize(1);
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
ERpStatus CModGantry::CComJointRoll::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* Update Info */
  jointRollInfo.posit = -motor[0]->motorData[CMtrInstance::DATA_POSIT];
  jointRollInfo.isPositArrived = (abs(jointRollCmd.setPosit - jointRollInfo.posit) < 8192);

  /* Control Process */
  switch (processFlag_) {

    case 0: {   // Lift Reset
      mtrOutputBuffer.fill(0);
      return RP_OK;
    }

    case 1: {   // Lift Pre-Init
      jointRollCmd.setPosit = static_cast<int32_t>(rangeLimit * 1.2);
      motor[0]->motorData[CMtrInstance::DATA_POSIT] = -jointRollCmd.setPosit;
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      processFlag_++;
      return RP_OK;
    }

    case 2: {   // Lift Init
      if (motor[0]->motorState == CMtrInstance::EMotorStatus::STALL) {
        jointRollCmd.setPosit = 0;
        motor[0]->motorData[CMtrInstance::DATA_POSIT] = 8192 * 1.0;
        pidPosCtrl.ResetAlgorithm();
        pidSpdCtrl.ResetAlgorithm();
        componentState = RP_OK;
        processFlag_++;
        return RP_OK;
      }
      jointRollCmd.setPosit -= 200;
      return _UpdateOutput(static_cast<float_t>(jointRollCmd.setPosit));
    }

    case 3: {   // Lift Control
      jointRollCmd.setPosit = std::clamp(jointRollCmd.setPosit, 0l, rangeLimit);
      return _UpdateOutput(static_cast<float_t>(jointRollCmd.setPosit));
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
int32_t CModGantry::CComJointRoll::PhyPositToMtrPosit(float_t phyPosit) {

  const int32_t zeroOffset = 467733;
  const float_t scale = -2061.03f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief
 * @param mtrPosit
 * @return
 */
float_t CModGantry::CComJointRoll::MtrPositToPhyPosit(int32_t mtrPosit) {

  const int32_t zeroOffset = 467733;
  const float_t scale = -2061.03f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param posit
 * @return
 */
ERpStatus CModGantry::CComJointRoll::_UpdateOutput(float_t posit) {

  jointRollPos[0] = static_cast<float_t>(-posit);
  jointRollPosMeasure[0] = static_cast<float_t>(motor[0]->motorData[CMtrInstance::DATA_POSIT]);

  pidPosCtrl.UpdatePidController(jointRollPos, jointRollPosMeasure, jointRollSpd);

  jointRollSpdMeasure[0] = static_cast<float_t>(motor[0]->motorData[CMtrInstance::DATA_SPEED]);

  pidSpdCtrl.UpdatePidController(jointRollSpd, jointRollSpdMeasure, output);

  mtrOutputBuffer = {
    static_cast<int16_t>(output[0]),
  };

  return RP_OK;
}

} // namespace robotpilots