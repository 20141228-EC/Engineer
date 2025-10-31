/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-03-19
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-19   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "mtr/mtr_dji.hpp"

namespace robotpilots {

/**
   * @brief
   * @param pStruct
   * @return
   */
ERpStatus CMtrDji::InitDevice(const SDevInitParam *pStruct) {

  if (pStruct == nullptr) return RP_ERROR;
  if (pStruct->deviceID == EDeviceID::DEV_NULL) return RP_ERROR;

  auto &param = *static_cast<const SDjiMtrInitParam *>(pStruct);
  if (param.djiMtrID == EDjiMtrID::ID_NULL) return RP_ERROR;
  deviceId         = param.deviceID;
  djiMtrID         = param.djiMtrID;
  useStallMonit_   = param.useStallMonit;
  useAngleToPosit_ = param.useAngleToPosit;
  canInterface_    = static_cast<CCanInterface *>(InterfaceMap.at(param.interfaceId));

  if (useStallMonit_) {
    stallMonitDataSrc_ = param.stallMonitDataSrc;
    stallThreshold_ = param.stallThreshold;
    stallTime_ = param.stallTime;
  }

  if (useAngleToPosit_) {
    encoderResolution_ = 8192;
  }

  auto canRxID = 0x200 + static_cast<uint32_t>(djiMtrID);
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
ERpStatus CMtrDji::FillCanTxBuffer(DataBuffer<uint8_t> &buffer, const int16_t current) {

  if (deviceState == RP_RESET) return RP_ERROR;
  if (buffer.size() != 8) return RP_ERROR;

  switch (djiMtrID) {

    case EDjiMtrID::ID_1:
    case EDjiMtrID::ID_5:
      buffer[0] = static_cast<uint8_t>(current >> 8);
      buffer[1] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDjiMtrID::ID_2:
    case EDjiMtrID::ID_6:
      buffer[2] = static_cast<uint8_t>(current >> 8);
      buffer[3] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDjiMtrID::ID_3:
    case EDjiMtrID::ID_7:
      buffer[4] = static_cast<uint8_t>(current >> 8);
      buffer[5] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDjiMtrID::ID_4:
    case EDjiMtrID::ID_8:
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
ERpStatus CMtrDji::FillCanTxBuffer(uint8_t *buffer, const int16_t current) {

  if (deviceState == RP_RESET) return RP_ERROR;
  if (buffer == nullptr) return RP_ERROR;

  switch (djiMtrID) {

    case EDjiMtrID::ID_1:
    case EDjiMtrID::ID_5:
      buffer[0] = static_cast<uint8_t>(current >> 8);
      buffer[1] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDjiMtrID::ID_2:
    case EDjiMtrID::ID_6:
      buffer[2] = static_cast<uint8_t>(current >> 8);
      buffer[3] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDjiMtrID::ID_3:
    case EDjiMtrID::ID_7:
      buffer[4] = static_cast<uint8_t>(current >> 8);
      buffer[5] = static_cast<uint8_t>(current & 0xFF);
      break;

    case EDjiMtrID::ID_4:
    case EDjiMtrID::ID_8:
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
ERpStatus CMtrDji::FillCanTxBuffer(CMtrInstance *mtr, DataBuffer<uint8_t> &buffer, const int16_t current) {

  if (mtr == nullptr) return RP_ERROR;

  auto djiMtr = static_cast<CMtrDji *>(mtr);

  return djiMtr->FillCanTxBuffer(buffer, current);
}


/**
 * @brief
 * @param mtr
 * @param buffer
 * @param current
 * @return
 */
ERpStatus CMtrDji::FillCanTxBuffer(CMtrInstance *mtr, uint8_t *buffer, const int16_t current) {

  if (mtr == nullptr) return RP_ERROR;

  auto djiMtr = static_cast<CMtrDji *>(mtr);

  return djiMtr->FillCanTxBuffer(buffer, current);
}


/**
 * @brief
 */
void CMtrDji::HeartbeatHandler_() {

//  const auto tickRate          = 10;     // Unit: Hz
//  const auto offlineDelay      = 500;    // Unit: ms
//  const auto stallSpdThreshold = 50;     // Unit: rpm
//
//  if (motorState == EMotorStatus::RESET) return;
//
//  if (HAL_GetTick() - lastHeartbeatTime_ > (offlineDelay / tickRate)) {
//    motorState = EMotorStatus::OFFLINE;
//    return;
//  }
//
//  motorState = (abs(motorData[DATA_SPEED]) < stallSpdThreshold) ?
//    EMotorStatus::STOP : EMotorStatus::RUNNING;
//
//  if (useStallMonit_) {
//    if (motorState == EMotorStatus::STOP
//        && abs(motorData[stallMonitDataSrc_]) >= static_cast<int32_t>(stallThreshold_))
//      stallCount_++;
//    else
//      stallCount_ = 0;
//
//    if (stallCount_ >= (stallTime_ / tickRate)) {
//      stallCount_ = stallTime_ / tickRate;
//      motorState = EMotorStatus::STALL;
//    }
//  }

  if (motorState != EMotorStatus::RESET) {
    if (HAL_GetTick() - lastHeartbeatTime_ > 500 / 10)
      motorState = EMotorStatus::OFFLINE;
    else {
      motorState = (abs(motorData[DATA_SPEED]) < 50) ?
        EMotorStatus::STOP : EMotorStatus::RUNNING;
      if (useStallMonit_) {
        if (abs(motorData[stallMonitDataSrc_]) > static_cast<int32_t>(stallThreshold_)
            && motorState == EMotorStatus::STOP) {    // Stall Threshold
          stallCount_++;
          if (stallCount_ >= stallTime_ / 10) {     // Stall Time
            stallCount_ = stallTime_ / 10;
            motorState = EMotorStatus::STALL;
          }
        } else {
          stallCount_ = 0;
          motorState = EMotorStatus::RUNNING;
        }
      }
    }
  }
}

/**
 * @brief
 * @return
 */
int32_t CMtrDji::getPosition_() {

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
