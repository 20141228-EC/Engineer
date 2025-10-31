/**
 * @file        com_wheelset.cpp
 * @version     1.1
 * @date        2024-04-05
 * @author      Morthine Xiang
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-04-05   <td>1.0         <td>Morthine Xiang  <td>First Create.
 * <tr><td>2024-05-10   <td>1.1         <td>Morthine Xiang  <td>Rewrite Component Structure.
 * </table>
 */

#include "mod_chassis.hpp"

namespace robotpilots {

/**
 * @brief
 * @param param
 * @return
 */
ERpStatus CModChassis::CComWheelset::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto chassisParam = static_cast<SModChassisInitParam &>(param);
  mems = MemsMap.at(chassisParam.memsDevID);
  motor[LF] = MotorMap.at(chassisParam.wheelsetMotorID_LF);
  motor[RF] = MotorMap.at(chassisParam.wheelsetMotorID_RF);
  motor[LB] = MotorMap.at(chassisParam.wheelsetMotorID_LB);
  motor[RB] = MotorMap.at(chassisParam.wheelsetMotorID_RB);

  /* Initialize PID Controllers */
  chassisParam.wheelsetSpdPidParam.threadNum = 4;
  pidSpdCtrl.InitAlgorithm(&chassisParam.wheelsetSpdPidParam);

  chassisParam.yawCorrectionPidParam.threadNum = 1;
  pidYawCtrl.InitAlgorithm(&chassisParam.yawCorrectionPidParam);

  /* Clear Motor Output Buffer */
  yawSpd.resize(1);
  yawSpdMeasure.resize(1);
  wheelSpd.resize(4);
  wheelSpdMeasure.resize(4);
  output.resize(4);
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
ERpStatus CModChassis::CComWheelset::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* FSM */
  switch (processFlag_) {

    case 0: {   // WheelSet Reset
      mtrOutputBuffer.fill(0);
      return RP_OK;
    }

    case 1: {   // WheelSet Pre-Init
      motor[LF]->motorData[CMtrInstance::DATA_POSIT] = 0;
      motor[RF]->motorData[CMtrInstance::DATA_POSIT] = 0;
      motor[LB]->motorData[CMtrInstance::DATA_POSIT] = 0;
      motor[RB]->motorData[CMtrInstance::DATA_POSIT] = 0;
      mtrOutputBuffer.fill(0);
      pidYawCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      processFlag_ = 2;
      return RP_OK;
    }

    case 2: {   // WheelSet Init
      if (mems->memsState == CMemsInstance::EMemsStatus::NORMAL) {
        wheelsetCmd = SWheelsetCommand();
        processFlag_ = 3;
        componentState = RP_OK;
        return RP_OK;
      }

//      return _UpdateOutput(0.0f, 0.0f, 0.0f);
      return _UpdateOutput(wheelsetCmd.speed_X, wheelsetCmd.speed_Y, wheelsetCmd.speed_W);
    }

    case 3: {   // WheelSet Control

//      DataBuffer<float_t> yawSpd = {wheelsetCmd.speed_W / 10.0f};
//      DataBuffer<float_t> yawSpdMeasure = {mems->memsData[CMemsInstance::DATA_GYRO_Z]};
      yawSpd[0] = wheelsetCmd.speed_W / 10.0f;
      yawSpdMeasure[0] = mems->memsData[CMemsInstance::DATA_GYRO_Z];

      auto output = pidYawCtrl.UpdatePidController(yawSpd, yawSpdMeasure);

      return _UpdateOutput(wheelsetCmd.speed_X, wheelsetCmd.speed_Y, output[0] * 10);
    }

    default: {
      processFlag_ = 0;
      return RP_ERROR;
    }
  }
}


/**
 * @brief
 * @param speed_X
 * @param speed_Y
 * @param speed_W
 * @return
 */
ERpStatus CModChassis::CComWheelset::_UpdateOutput(float_t speed_X,
                                                   float_t speed_Y,
                                                   float_t speed_W) {

  wheelSpd[LF] = speed_Y + speed_X + speed_W;
  wheelSpd[RF] = - speed_Y + speed_X + speed_W;
  wheelSpd[LB] = speed_Y - speed_X + speed_W;
  wheelSpd[RB] = - speed_Y - speed_X + speed_W;
  wheelSpdMeasure[LF] = static_cast<float_t>(motor[LF]->motorData[CMtrInstance::DATA_SPEED]);
  wheelSpdMeasure[RF] = static_cast<float_t>(motor[RF]->motorData[CMtrInstance::DATA_SPEED]);
  wheelSpdMeasure[LB] = static_cast<float_t>(motor[LB]->motorData[CMtrInstance::DATA_SPEED]);
  wheelSpdMeasure[RB] = static_cast<float_t>(motor[RB]->motorData[CMtrInstance::DATA_SPEED]);

  pidSpdCtrl.UpdatePidController(wheelSpd, wheelSpdMeasure, output);

  mtrOutputBuffer = {
    static_cast<int16_t>(output[LF]),
    static_cast<int16_t>(output[RF]),
    static_cast<int16_t>(output[LB]),
    static_cast<int16_t>(output[RB]),
  };

  return RP_OK;
}

} // namespace robotpilots
