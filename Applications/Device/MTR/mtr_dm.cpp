/**
 * @file mtr_dm.cpp
 * @author Sassinak
 * @brief DM电机设备类实现
 * @version 1.0
 * @date 2025-05-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "mtr/mtr_dm.hpp"
#include "conf_CanTxNode.hpp"

namespace my_engineer {

/**
 * @brief In MIT mode, conversion is required.
 * @note 增加范围钳制，防止越界导致数值回绕
 */
int CDevMtrDM::float_to_uint(float_t x_float, float_t x_min, float_t x_max, int bits){
    /* Clamp input to valid range to prevent overflow/underflow */
    if (x_float < x_min) x_float = x_min;
    if (x_float > x_max) x_float = x_max;

    /* Converts a float to an unsigned int, given range and number of bits */
    float_t span = x_max - x_min;
    float_t offset = x_min;
    return (int) ((x_float-offset)*((float_t)((1<<bits)-1))/span);
  }

/**
 * @brief In MIT mode, conversion is required.
 * @param x_int
 * @param x_min
 * @param x_max
 * @param bits
 * @return
 */ 
float_t CDevMtrDM::uint_to_float(int x_int, float_t x_min, float_t x_max, int bits){
    /* converts unsigned int to float, given range and number of bits */
    float_t span = x_max - x_min;
    float_t offset = x_min;
    return ((float_t)x_int)*span/((float_t)((1<<bits)-1)) + offset;
}

/**
 * @brief Enable the motor in MIT mode (uses internal TxNode).
 */
void CDevMtrDM::EnableMotor() {
  if (deviceStatus == APP_RESET) return;
  if (dmMtrMode != EMotorControlMode::MODE_MIT) return;

  motorEnabled_ = true;
  canTxNode_mit_.Transmit(DM_EnableBuffer);
}

/**
 * @brief Disable the motor in MIT mode (uses internal TxNode).
 */
void CDevMtrDM::DisableMotor() {
  if (deviceStatus == APP_RESET) return;
  if (dmMtrMode != EMotorControlMode::MODE_MIT) return;

  motorEnabled_ = false;
  canTxNode_mit_.Transmit(DM_DisableBuffer);
}

/**
 * @brief Save current position as zero point (MIT mode).
 */
void CDevMtrDM::SetZero() {
  if (deviceStatus == APP_RESET) return;
  if (dmMtrMode != EMotorControlMode::MODE_MIT) return;

  static const uint8_t zeroBuffer[8] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE
  };
  canTxNode_mit_.Transmit(zeroBuffer);
}

/**
 * @brief Clear motor error (MIT mode).
 */
void CDevMtrDM::ClearError() {
  if (deviceStatus == APP_RESET) return;
  if (dmMtrMode != EMotorControlMode::MODE_MIT) return;

  static const uint8_t clearBuffer[8] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB
  };
  canTxNode_mit_.Transmit(clearBuffer);
}

/**
 * @brief MIT mode control (fill + transmit in one call).
 * @param kp Position stiffness (0-500)
 * @param kd Damping coefficient (0-5)
 * @param pos Target position (rad)
 * @param spd Speed setpoint (rad/s)
 * @param torq Torque feedforward (N·m)
 */
void CDevMtrDM::Control_MIT(float_t kp, float_t kd, float_t pos, float_t spd, float_t torq) {
  if (deviceStatus == APP_RESET) return;
  if (dmMtrMode != EMotorControlMode::MODE_MIT) return;

  uint16_t raw_angle  = float_to_uint(pos, -mitLimit_.Q_MAX, mitLimit_.Q_MAX, 16);
  uint16_t raw_speed  = float_to_uint(spd, -mitLimit_.DQ_MAX, mitLimit_.DQ_MAX, 12);
  uint16_t raw_Kp     = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  uint16_t raw_Kd     = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  uint16_t raw_torque = float_to_uint(torq, -mitLimit_.TAU_MAX, mitLimit_.TAU_MAX, 12);

  uint8_t buffer[8];
  buffer[0] = static_cast<uint8_t>(raw_angle >> 8);
  buffer[1] = static_cast<uint8_t>(raw_angle);
  buffer[2] = static_cast<uint8_t>(raw_speed >> 4);
  buffer[3] = static_cast<uint8_t>(((raw_speed & 0x0F) << 4) | (raw_Kp >> 8));
  buffer[4] = static_cast<uint8_t>(raw_Kp);
  buffer[5] = static_cast<uint8_t>(raw_Kd >> 4);
  buffer[6] = static_cast<uint8_t>(((raw_Kd & 0x0F) << 4) | (raw_torque >> 8));
  buffer[7] = static_cast<uint8_t>(raw_torque);

  canTxNode_mit_.Transmit(buffer);
}


/**
 * @brief
 * @param pStructInitParam
 * @return
 */
EAppStatus CDevMtrDM::InitDevice(const SDevInitParam_Base *pStructInitParam) {

  if (pStructInitParam == nullptr) return APP_ERROR;
  if (pStructInitParam->deviceID == EDeviceID::DEV_NULL) return APP_ERROR;

  uint32_t canRxID = 0x00;

  auto &param = *reinterpret_cast<const SMtrInitParam_DM *>(pStructInitParam);
  if (param.dmMtrID == EDmMtrID::ID_NULL) return APP_ERROR;
  deviceID         = param.deviceID;
  dmMotorID        = param.dmMtrID;
  dmMtrMode        = param.dmMtrMode;
  Kp               = param.Kp;  // Proportional gain for MIT mode
  Kd               = param.Kd;  // Derivative gain for MIT mode
  pInfCAN_         = reinterpret_cast<CInfCAN *>(InterfaceIDMap.at(param.interfaceID));
  useAngleToPosit_ = param.useAngleToPosit;
  // 保存 MIT 参数范围（不同型号电机可配置不同值）
  mitLimit_.Q_MAX   = param.Q_MAX;
  mitLimit_.DQ_MAX  = param.DQ_MAX;
  mitLimit_.TAU_MAX = param.TAU_MAX;
  
  /*You must confirm which mode you want to use.！！！*/
  switch (dmMtrMode)
  {
    case EMotorControlMode::MODE_CURRENT:
      canRxID = 0x300 + static_cast<uint32_t>(dmMotorID);
      canRxNode_.InitRxNode(param.interfaceID, canRxID,
                        CInfCAN::ECanFrameType::DATA,
                        CInfCAN::ECanFrameDlc::DLC_8);
      RegisterDevice_();
      RegisterMotor_();
      break;
    case EMotorControlMode::MODE_MIT:
      canRxID = param.MIT_RxCANID;
      canRxNode_.InitRxNode(param.interfaceID, canRxID,
                        CInfCAN::ECanFrameType::DATA,
                        CInfCAN::ECanFrameDlc::DLC_8);
      /* 初始化 MIT 模式专用发送节点 */
      canTxNode_mit_.InitTxNode(param.interfaceID, param.MIT_TxCANID,
                        CInfCAN::ECanFrameType::DATA,
                        CInfCAN::ECanFrameDlc::DLC_8);
      RegisterDevice_();
      RegisterMotor_();
      break;
    default:
      return APP_ERROR;  ///< 未知模式，不注册，直接返回错误
  }

  deviceStatus = APP_OK;
  motorStatus = EMotorStatus::OFFLINE;
  if (dmMtrMode == EMotorControlMode::MODE_MIT) {
    EnableMotor();  // 使用内部 TxNode 发送使能帧
  }
  return APP_OK;
}


/**
 * @brief
 * @param buffer
 * @param current
 * @return
 */
EAppStatus CDevMtrDM::FillCanTxBuffer(DataBuffer<uint8_t> &buffer, const int16_t current) {

  if (deviceStatus == APP_RESET) return APP_ERROR;
  if (buffer.size() != 8) return APP_ERROR;

  if(dmMtrMode != EMotorControlMode::MODE_CURRENT) return APP_ERROR; 

  switch (dmMotorID) {

    case EDmMtrID::ID_1:
    case EDmMtrID::ID_5:
      buffer[0] = static_cast<uint8_t>(current >> 8);
      buffer[1] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDmMtrID::ID_2:
    case EDmMtrID::ID_6:
      buffer[2] = static_cast<uint8_t>(current >> 8);
      buffer[3] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDmMtrID::ID_3:
    case EDmMtrID::ID_7:
      buffer[4] = static_cast<uint8_t>(current >> 8);
      buffer[5] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDmMtrID::ID_4:
    case EDmMtrID::ID_8:
      buffer[6] = static_cast<uint8_t>(current >> 8);
      buffer[7] = static_cast<uint8_t>(current & 0xFF);
      break;

    default:
      return APP_ERROR;
}

  return APP_OK;
}


/**
 * @brief
 * @param buffer
 * @param current
 * @return
 */
EAppStatus CDevMtrDM::FillCanTxBuffer(uint8_t *buffer, const int16_t current) {

  if (deviceStatus == APP_RESET) return APP_ERROR;
  if (buffer == nullptr) return APP_ERROR;

  switch (dmMotorID) {

    case EDmMtrID::ID_1:
    case EDmMtrID::ID_5:
      buffer[0] = static_cast<uint8_t>(current >> 8);
      buffer[1] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDmMtrID::ID_2:
    case EDmMtrID::ID_6:
      buffer[2] = static_cast<uint8_t>(current >> 8);
      buffer[3] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDmMtrID::ID_3:
    case EDmMtrID::ID_7:
      buffer[4] = static_cast<uint8_t>(current >> 8);
      buffer[5] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDmMtrID::ID_4:
    case EDmMtrID::ID_8:
      buffer[6] = static_cast<uint8_t>(current >> 8);
      buffer[7] = static_cast<uint8_t>(current & 0xFF);
      break;

    default:
      return APP_ERROR;
  }

  return APP_OK;
}


/**
 * @brief
 * @param mtr
 * @param buffer
 * @param current
 * @return
 */
EAppStatus CDevMtrDM::FillCanTxBuffer(CDevMtr *mtr, DataBuffer<uint8_t> &buffer, const int16_t current) {

  if (mtr == nullptr) return APP_ERROR;

  auto dmMtr = static_cast<CDevMtrDM *>(mtr);

  return dmMtr->FillCanTxBuffer(buffer, current);
}


/**
 * @brief
 * @param mtr
 * @param buffer
 * @param current
 * @return
 */
EAppStatus CDevMtrDM::FillCanTxBuffer(CDevMtr *mtr, uint8_t *buffer, const int16_t current) {

  if (mtr == nullptr) return APP_ERROR;

  auto dmMtr = static_cast<CDevMtrDM *>(mtr);

  return dmMtr->FillCanTxBuffer(buffer, current);
}



/*------------------------------------------------------MIT MODE START----------------------------------------------------------------------------------------------*/

/**
 * @brief
 * @param buffer
 * @param current
 * @return
 */
EAppStatus CDevMtrDM::FillCanTxBuffer_MIT(DataBuffer<uint8_t> &buffer,const float_t pos,const float_t speed, const float_t torq,const float_t Kp,const float_t Kd) {

  if (deviceStatus == APP_RESET) return APP_ERROR;
  if (buffer.size() != 8) return APP_ERROR;

  if(dmMtrMode != EMotorControlMode::MODE_MIT) return APP_ERROR; 

  uint16_t  raw_angle   = float_to_uint(pos, -mitLimit_.Q_MAX, mitLimit_.Q_MAX, 16);
	uint16_t	raw_speed   = float_to_uint(speed, -mitLimit_.DQ_MAX, mitLimit_.DQ_MAX, 12);
	uint16_t	raw_Kp_MIT  = float_to_uint(Kp, KP_MIN, KP_MAX, 12);
	uint16_t	raw_Kd_MIT  = float_to_uint(Kd, KD_MIN, KD_MAX, 12);
  uint16_t	raw_torque  = float_to_uint(torq, -mitLimit_.TAU_MAX, mitLimit_.TAU_MAX, 12);

  buffer[0] = static_cast<uint8_t>(raw_angle >> 8);
  buffer[1] = static_cast<uint8_t>(raw_angle);
  buffer[2] = static_cast<uint8_t>(raw_speed >> 4);
  buffer[3] = static_cast<uint8_t>(((raw_speed & 0x0F) << 4) | (raw_Kp_MIT >> 8));
  buffer[4] = static_cast<uint8_t>(raw_Kp_MIT);
  buffer[5] = static_cast<uint8_t>(raw_Kd_MIT >> 4);
  buffer[6] = static_cast<uint8_t>(((raw_Kd_MIT & 0x0F) << 4) | (raw_torque >> 8));
  buffer[7] = static_cast<uint8_t>(raw_torque);

  return APP_OK;
}

/**
 * @brief
 * @param buffer
 * @return
 */
EAppStatus CDevMtrDM::FillCanTxBuffer_MIT(uint8_t *buffer,const float_t pos,const float_t speed, const float_t torq,const float_t Kp,const float_t Kd) {

  if (deviceStatus == APP_RESET) return APP_ERROR;
  if (buffer == nullptr) return APP_ERROR;

  if(dmMtrMode != EMotorControlMode::MODE_MIT) return APP_ERROR;

  uint16_t  raw_angle   = float_to_uint(pos, -mitLimit_.Q_MAX, mitLimit_.Q_MAX, 16);
	uint16_t	raw_speed   = float_to_uint(speed, -mitLimit_.DQ_MAX, mitLimit_.DQ_MAX, 12);
	uint16_t	raw_Kp_MIT  = float_to_uint(Kp, KP_MIN, KP_MAX, 12);
	uint16_t	raw_Kd_MIT  = float_to_uint(Kd, KD_MIN, KD_MAX, 12);
  uint16_t	raw_torque  = float_to_uint(torq, -mitLimit_.TAU_MAX, mitLimit_.TAU_MAX, 12);

  buffer[0] = static_cast<uint8_t>(raw_angle >> 8);
  buffer[1] = static_cast<uint8_t>(raw_angle);
  buffer[2] = static_cast<uint8_t>(raw_speed >> 4);
  buffer[3] = static_cast<uint8_t>(((raw_speed & 0x0F) << 4) | (raw_Kp_MIT >> 8));
  buffer[4] = static_cast<uint8_t>(raw_Kp_MIT);
  buffer[5] = static_cast<uint8_t>(raw_Kd_MIT >> 4);
  buffer[6] = static_cast<uint8_t>(((raw_Kd_MIT & 0x0F) << 4) | (raw_torque >> 8));
  buffer[7] = static_cast<uint8_t>(raw_torque);

 return APP_OK;
}

/**
 * @brief
 * @param mtr
 * @param buffer
 * @return
 */
EAppStatus CDevMtrDM::FillCanTxBuffer_MIT(CDevMtr *mtr, DataBuffer<uint8_t> &buffer,const float_t pos,const float_t speed, const float_t torq,const float_t Kp,const float_t Kd) {

  if (mtr == nullptr) return APP_ERROR;

  auto dmMtr = static_cast<CDevMtrDM *>(mtr);

  return dmMtr->FillCanTxBuffer_MIT(buffer, pos, speed, torq, Kp, Kd);
}

/**
 * @brief
 * @param mtr
 * @param buffer
 * @return
 */
EAppStatus CDevMtrDM::FillCanTxBuffer_MIT(CDevMtr *mtr, uint8_t *buffer,const float_t pos,const float_t speed, const float_t torq,const float_t Kp,const float_t Kd) {
  if (mtr == nullptr) return APP_ERROR;

  auto dmMtr = static_cast<CDevMtrDM *>(mtr);

  return dmMtr->FillCanTxBuffer_MIT(buffer, pos, speed, torq, Kp, Kd);
}




/*-----------------------------------------------------------MIT MODE END-----------------------------------------------------------------------------------------*/

/**
 * @brief
 */
void CDevMtrDM::UpdateHandler_() {

  if (deviceStatus == APP_RESET) return;

  /* Update Motor Data */
  switch (dmMtrMode) {
    case EMotorControlMode::MODE_CURRENT:
      if (canRxNode_.timestamp >= lastHeartbeatTime_) {
      motorData[DATA_ANGLE]   = (int16_t)(canRxNode_.dataBuffer[0] << 8 | canRxNode_.dataBuffer[1]);
      motorData[DATA_SPEED]   = (int16_t)(canRxNode_.dataBuffer[2] << 8 | canRxNode_.dataBuffer[3]);
      motorData[DATA_CURRENT] = (int16_t)(canRxNode_.dataBuffer[4] << 8 | canRxNode_.dataBuffer[5]);
      motorData[DATA_TEMP]    = (int8_t)(canRxNode_.dataBuffer[6]);
      motorData[DATA_POSIT]   = motorData[DATA_POSIT]  = (useAngleToPosit_) ? getPosition_() : 0;
      lastHeartbeatTime_  = canRxNode_.timestamp;
      }
      break;
    /*Here only get the raw data that from motor. If you need to know the true data,you need to use Function 'uint_to_float'*/  
    case EMotorControlMode::MODE_MIT:
      if (canRxNode_.timestamp >= lastHeartbeatTime_) {
        motorData[DATA_ID]          = static_cast<int32_t>(canRxNode_.dataBuffer[0] & 0x0F); // MIT motor ID
        motorData[DATA_STATE]       = static_cast<int32_t>(canRxNode_.dataBuffer[0] >> 4); // MIT motor state
        motorData[DATA_ANGLE]       = (uint16_t)((canRxNode_.dataBuffer[1] << 8) | canRxNode_.dataBuffer[2]);
        motorData[DATA_SPEED]       = (uint16_t)((canRxNode_.dataBuffer[3] << 4) | (canRxNode_.dataBuffer[4] >> 4));
        motorData[DATA_TORQUE]      = (uint16_t)(((canRxNode_.dataBuffer[4] & 0x0F) << 8) | canRxNode_.dataBuffer[5]);
        motorData[DATA_MOS_TEMP]    = (int8_t)(canRxNode_.dataBuffer[6]);
        motorData[DATA_ROTOR_TEMP]  = (int8_t)(canRxNode_.dataBuffer[7]);
        motorData[DATA_POSIT]       = (useAngleToPosit_) ? getPosition_() : 0;
        lastHeartbeatTime_ = canRxNode_.timestamp;
      }
      break;
    default:
      return;
  }

}


/**
 * @brief
 */
void CDevMtrDM::HeartbeatHandler_() {

  const auto tickRate          = 10;     // Unit: Hz
  const auto offlineDelay      = 500;    // Unit: ms
  const auto stallSpdThreshold = 50;     // Unit: rpm

  if (motorStatus == EMotorStatus::RESET) return;

  if(dmMtrMode == EMotorControlMode::MODE_MIT && motorEnabled_) {
    /* 节流: 每100ms发送一次使能帧，仅在 motorEnabled_ 时发送 */
    if (HAL_GetTick() - lastEnableTime_ > 100) {
      canTxNode_mit_.Transmit(DM_EnableBuffer);  // 直接发送，不修改 motorEnabled_
      lastEnableTime_ = HAL_GetTick();
    }
  }

  if (HAL_GetTick() - lastHeartbeatTime_ > offlineDelay) {  // 直接用 offlineDelay (500ms)
    motorStatus = EMotorStatus::OFFLINE;
    return;
  }

  if (dmMtrMode == EMotorControlMode::MODE_MIT) {
    /* MIT模式: 12bit原始值中点≈2048对应零速，需转为物理速度再判断 */
    float_t physicalSpeed = uint_to_float(
        motorData[DATA_SPEED], -mitLimit_.DQ_MAX, mitLimit_.DQ_MAX, 12);
    const float_t stallSpdThresholdRad = 1.0f;  // Unit: rad/s
    motorStatus = (fabsf(physicalSpeed) < stallSpdThresholdRad) ?
        EMotorStatus::STOP : EMotorStatus::RUNNING;
  } else {
    motorStatus = (abs(motorData[DATA_SPEED]) < stallSpdThreshold) ?
        EMotorStatus::STOP : EMotorStatus::RUNNING;
  }
}


/**
 * @brief
 * @return
 */
int32_t CDevMtrDM::getPosition_() {

  if (motorStatus == EMotorStatus::OFFLINE) return 0;
  if (!useAngleToPosit_) return 0;

  if (motorData[DATA_POSIT] == 0 && lastAngle_ == 0) {
    lastAngle_ = motorData[DATA_ANGLE];
    return 0;
  }

  int32_t err = motorData[DATA_ANGLE] - lastAngle_;
  if (abs(err) > static_cast<int32_t>(encoderResolution_) / 2)
    err -= static_cast<int32_t>(encoderResolution_) * (err / abs(err));

  lastAngle_ = motorData[DATA_ANGLE];

  return (motorData[DATA_POSIT] + err);
}

} // namespace my_engineer
