/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-04-17
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-04-17   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "mtr/mtr_dm.hpp"

namespace robotpilots {

/**
 * @brief
 * @param pStruct
 * @return
 */
ERpStatus CMtrDm::InitDevice(const SDevInitParam *pStruct) {

  if (pStruct == nullptr) return RP_ERROR;
  if (pStruct->deviceID == EDeviceID::DEV_NULL) return RP_ERROR;

  auto &param = *reinterpret_cast<const SMtrDmInitParam *>(pStruct);
  if (param.dmMtrID == EDmMtrID::ID_NULL) return RP_ERROR;
  deviceId         = param.deviceID;
  dmMotorID        = param.dmMtrID;
  canInterface_    = reinterpret_cast<CCanInterface *>(InterfaceMap.at(param.interfaceId));
  useAngleToPosit_ = param.useAngleToPosit;

  auto canRxID = 0x300 + static_cast<uint32_t>(dmMotorID);
  canRxNode_.InitRxNode(param.interfaceId, canRxID,
                        CCanInterface::ECanFrameType::DATA,
                        CCanInterface::ECanFrameDlc::DLC_8);

  RegisterDevice_();
  RegisterMotor_();

  deviceState = RP_OK;
  motorState = EMotorStatus::OFFLINE;

  return RP_OK;
}


/**
 * @brief
 * @param buffer
 * @param current
 * @return
 */
ERpStatus CMtrDm::FillCanTxBuffer(DataBuffer<uint8_t> &buffer, const int16_t current) {

  if (deviceState == RP_RESET) return RP_ERROR;
  if (buffer.size() != 8) return RP_ERROR;

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
      return RP_ERROR;
  }

  return RP_OK;
}


/**
 * @brief
 * @param buffer
 * @param current
 * @return
 */
ERpStatus CMtrDm::FillCanTxBuffer(uint8_t *buffer, const int16_t current) {

  if (deviceState == RP_RESET) return RP_ERROR;
  if (buffer == nullptr) return RP_ERROR;

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
      return RP_ERROR;
  }

  return RP_OK;
}


/**
 * @brief
 * @param mtr
 * @param buffer
 * @param current
 * @return
 */
ERpStatus CMtrDm::FillCanTxBuffer(CMtrInstance *mtr, DataBuffer<uint8_t> &buffer, const int16_t current) {

  if (mtr == nullptr) return RP_ERROR;

  auto dmMtr = static_cast<CMtrDm *>(mtr);

  return dmMtr->FillCanTxBuffer(buffer, current);
}


/**
 * @brief
 * @param mtr
 * @param buffer
 * @param current
 * @return
 */
ERpStatus CMtrDm::FillCanTxBuffer(CMtrInstance *mtr, uint8_t *buffer, const int16_t current) {

  if (mtr == nullptr) return RP_ERROR;

  auto dmMtr = static_cast<CMtrDm *>(mtr);

  return dmMtr->FillCanTxBuffer(buffer, current);
}


/**
 * @brief
 */
void CMtrDm::UpdateHandler_() {

  if (deviceState == RP_RESET) return;

  /* Update Motor Data */
  if (canRxNode_.timestamp >= lastHeartbeatTime_) {
    motorData[DATA_ANGLE]   = (int16_t)(canRxNode_.dataBuffer[0] << 8 | canRxNode_.dataBuffer[1]);
    motorData[DATA_SPEED]   = (int16_t)(canRxNode_.dataBuffer[2] << 8 | canRxNode_.dataBuffer[3]);
    motorData[DATA_CURRENT] = (int16_t)(canRxNode_.dataBuffer[4] << 8 | canRxNode_.dataBuffer[5]);
    motorData[DATA_TEMP]    = (int8_t)(canRxNode_.dataBuffer[6]);
    motorData[DATA_POSIT]   = motorData[DATA_POSIT]  = (useAngleToPosit_) ? getPosition_() : 0;
    lastHeartbeatTime_  = canRxNode_.timestamp;
  }
}


/**
 * @brief
 */
void CMtrDm::HeartbeatHandler_() {

  const auto tickRate          = 10;     // Unit: Hz
  const auto offlineDelay      = 500;    // Unit: ms
  const auto stallSpdThreshold = 50;     // Unit: rpm

  if (motorState == EMotorStatus::RESET) return;

  if (HAL_GetTick() - lastHeartbeatTime_ > (offlineDelay / tickRate)) {
    motorState = EMotorStatus::OFFLINE;
    return;
  }

  motorState = (abs(motorData[DATA_SPEED]) < stallSpdThreshold) ?
    EMotorStatus::STOP : EMotorStatus::RUNNING;
}


/**
 * @brief
 * @return
 */
int32_t CMtrDm::getPosition_() {

  if (motorState == EMotorStatus::OFFLINE) return 0;
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

} // namespace robotpilots
