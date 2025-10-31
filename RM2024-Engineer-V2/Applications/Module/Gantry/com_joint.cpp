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
ERpStatus CModGantry::CComJoint::InitComponent(SModInitParam &param) {

  /* Initialize Component */
  auto &gantryParam = static_cast<SModGantryInitParam &>(param);
  motor[YAW]  = MotorMap.at(gantryParam.jointMotorID_Yaw);
  motor[ROLL] = MotorMap.at(gantryParam.jointMotorID_Roll);

  /* Initialize PID Controller */
  gantryParam.jointPosPidParam.threadNum = 2;
  pidPosCtrl.InitAlgorithm(&gantryParam.jointPosPidParam);

  gantryParam.jointSpdPidParam.threadNum = 2;
  pidSpdCtrl.InitAlgorithm(&gantryParam.jointSpdPidParam);

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
ERpStatus CModGantry::CComJoint::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* Update Component Info */
  jointInfo.posit_Yaw = motor[YAW]->motorData[CMtrInstance::DATA_POSIT];
  jointInfo.posit_Roll = motor[ROLL]->motorData[CMtrInstance::DATA_POSIT];
  jointInfo.isPositArrived_Yaw = abs(jointCmd.setPosit_Yaw - jointInfo.posit_Yaw) < 16;
  jointInfo.isPositArrived_Roll = abs(jointCmd.setPosit_Roll - jointInfo.posit_Roll) < 16;

  /* FSM Status Enum */
  enum {
    JOINT_RESET   = 0,
    JOINT_PREINIT = 1,
    JOINT_INIT,
    JOINT_CTRL,
  };

  /* Gantry Joint Component Control FSM */
  switch (processFlag_) {

    case JOINT_RESET: {
      mtrOutputBuffer.fill(0);
      return RP_OK;
    }

    case JOINT_PREINIT: {
      motor[YAW]->motorData[CMtrInstance::DATA_POSIT] = motor[YAW]->motorData[CMtrInstance::DATA_ANGLE] - 2020;
      if (motor[YAW]->motorData[CMtrInstance::DATA_POSIT] > 4096)
        motor[YAW]->motorData[CMtrInstance::DATA_POSIT] -= 8192;
      motor[ROLL]->motorData[CMtrInstance::DATA_POSIT] = motor[ROLL]->motorData[CMtrInstance::DATA_ANGLE] - 4730;
      processFlag_ = JOINT_INIT;
      return RP_OK;
    }

    case JOINT_INIT: {
      if (abs(jointInfo.posit_Yaw + 2048) < 128 &&
          abs(jointInfo.posit_Roll - 0) < 128) {
        jointCmd.setPosit_Yaw = -2048;
        jointCmd.setPosit_Roll = 0;
        pidPosCtrl.ResetAlgorithm();
        pidSpdCtrl.ResetAlgorithm();
        componentState = RP_OK;
        processFlag_ = JOINT_CTRL;
      }
      return _UpdateOutput(-2048.0f, 0.0f);
    }

    case JOINT_CTRL: {
      jointCmd.setPosit_Yaw = std::clamp(jointCmd.setPosit_Yaw, -2048l, rangeLimit_YAW-2048l);
      jointCmd.setPosit_Roll = std::clamp(jointCmd.setPosit_Roll, -4096l, rangeLimit_ROLL-4096l);
      return _UpdateOutput(static_cast<float_t>(jointCmd.setPosit_Yaw),
                           static_cast<float_t>(jointCmd.setPosit_Roll));
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
int32_t CModGantry::CComJoint::PhyPositToMtrPosit_Yaw(float_t phyPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 22.7555f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief
 * @param mtrPosit
 * @return
 */
float_t CModGantry::CComJoint::MtrPositToPhyPosit_Yaw(int32_t mtrPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 22.7555f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param phyPosit
 * @return
 */
int32_t CModGantry::CComJoint::PhyPositToMtrPosit_Roll(float_t phyPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 22.7555f;

  return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}


/**
 * @brief
 * @param mtrPosit
 * @return
 */
float_t CModGantry::CComJoint::MtrPositToPhyPosit_Roll(int32_t mtrPosit) {

  const int32_t zeroOffset = 0;
  const float_t scale = 22.7555f;

  return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}


/**
 * @brief
 * @param posit_Yaw
 * @param posit_Roll
 * @return
 */
ERpStatus CModGantry::CComJoint::_UpdateOutput(float_t posit_Yaw, float_t posit_Roll) {

  DataBuffer<float_t> jointPos = {
    posit_Yaw,
    posit_Roll,
  };

  DataBuffer<float_t> jointPosMeasure = {
    static_cast<float_t>(motor[YAW]->motorData[CMtrInstance::DATA_POSIT]),
    static_cast<float_t>(motor[ROLL]->motorData[CMtrInstance::DATA_POSIT]),
  };

  auto jointSpd =
    pidPosCtrl.UpdatePidController(jointPos, jointPosMeasure);

  DataBuffer<float_t> jointSpdMeasure = {
    static_cast<float_t>(motor[YAW]->motorData[CMtrInstance::DATA_SPEED]) / 100.0f,
    static_cast<float_t>(motor[ROLL]->motorData[CMtrInstance::DATA_SPEED]) / 100.0f,
  };

  auto output =
    pidSpdCtrl.UpdatePidController(jointSpd, jointSpdMeasure);

  mtrOutputBuffer = {
    static_cast<int16_t>(output[YAW]),
    static_cast<int16_t>(output[ROLL]),
  };

  return RP_OK;
}

} // namespace robotpilots
