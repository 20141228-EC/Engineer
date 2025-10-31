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
ERpStatus CModGantry::CComJointYaw::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto gantryParam = static_cast<SModGantryInitParam &>(param);
  motor[0] = MotorMap.at(gantryParam.jointMotorID_Yaw);

  /* Initialize PID Controllers */
  gantryParam.jointPosPidParam.threadNum = 1;
  pidPosCtrl.InitAlgorithm(&gantryParam.jointPosPidParam);

  gantryParam.jointSpdPidParam.threadNum = 1;
  pidSpdCtrl.InitAlgorithm(&gantryParam.jointSpdPidParam);

  /* Clear Motor Output Buffer */
  jointYawPos.resize(1);
  jointYawPosMeasure.resize(1);
  jointYawSpd.resize(1);
  jointYawSpdMeasure.resize(1);
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
ERpStatus CModGantry::CComJointYaw::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* Update Info */
  jointYawInfo.posit = motor[0]->motorData[CMtrInstance::DATA_POSIT];
  jointYawInfo.isPositArrived = (abs(jointYawCmd.setPosit - jointYawInfo.posit) < 8192);

  /* Control Process */
  switch (processFlag_) {

    case 0: {   // Lift Reset
      mtrOutputBuffer.fill(0);
      return RP_OK;
    }

    case 1: {   // Lift Pre-Init
      jointYawCmd.setPosit = static_cast<int32_t>(rangeLimit * 1.2);
      motor[0]->motorData[CMtrInstance::DATA_POSIT] = jointYawCmd.setPosit;
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      processFlag_++;
      return RP_OK;
    }

    case 2: {   // Lift Init
      if (motor[0]->motorState == CMtrInstance::EMotorStatus::STALL) {
        jointYawCmd.setPosit = 0;
        motor[0]->motorData[CMtrInstance::DATA_POSIT] = -8192 * 1.0;
        pidPosCtrl.ResetAlgorithm();
        pidSpdCtrl.ResetAlgorithm();
        componentState = RP_OK;
        processFlag_++;
        return RP_OK;
      }
      jointYawCmd.setPosit -= 200;
      return _UpdateOutput(static_cast<float_t>(jointYawCmd.setPosit));
    }

    case 3: {   // Lift Control
      jointYawCmd.setPosit = std::clamp(jointYawCmd.setPosit, 0l, rangeLimit);
      return _UpdateOutput(static_cast<float_t>(jointYawCmd.setPosit));
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
int32_t CModGantry::CComJointYaw::PhyPositToMtrPosit(float_t phyPosit) {

  const int32_t zeroOffset = 150835;
  const float_t scale = 1635.44f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief
 * @param mtrPosit
 * @return
 */
float_t CModGantry::CComJointYaw::MtrPositToPhyPosit(int32_t mtrPosit) {

  const int32_t zeroOffset = 150835;
  const float_t scale = 1635.44f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param posit
 * @return
 */
ERpStatus CModGantry::CComJointYaw::_UpdateOutput(float_t posit) {

  jointYawPos[0] = static_cast<float_t>(posit);
  jointYawPosMeasure[0] = static_cast<float_t>(motor[0]->motorData[CMtrInstance::DATA_POSIT]);

  pidPosCtrl.UpdatePidController(jointYawPos, jointYawPosMeasure, jointYawSpd);

  jointYawSpdMeasure[0] = static_cast<float_t>(motor[0]->motorData[CMtrInstance::DATA_SPEED]);

  pidSpdCtrl.UpdatePidController(jointYawSpd, jointYawSpdMeasure, output);

  mtrOutputBuffer = {
    static_cast<int16_t>(output[0]),
  };

  return RP_OK;
}

} // namespace robotpilots
