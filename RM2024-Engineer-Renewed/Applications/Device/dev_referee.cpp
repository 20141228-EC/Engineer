/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-04-30
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-04-30   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "dev_referee.hpp"

namespace robotpilots {

/**
 * @brief
 * @param pStruct
 *
 * @return
 */
ERpStatus CDevReferee::InitDevice(const SDevInitParam *pStruct) {

  if (pStruct == nullptr) return RP_ERROR;
  if (pStruct->deviceID == EDeviceID::DEV_NULL) return RP_ERROR;

  auto &param = *static_cast<const SDevRefereeInitParam *>(pStruct);
  deviceId = param.deviceID;
  uartInterface_ = static_cast<CUartInterface *>(InterfaceMap.at(param.interfaceId));

  auto callback = [this](auto &buffer, auto len) {
    if (len > 256) return;
    std::copy(buffer.data(), buffer.data() + len, rxBuffer_.data());
    rxTimestamp_ = HAL_GetTick();
//    ResolveRxPackage_();
  };

  RegisterDevice_();
  uartInterface_->RegisterCallback(callback);

  deviceState = RP_OK;
  refereeState = ERefereeStatus::OFFLINE;

  return RP_OK;
}


/**
 * @brief
 * @param commandID
 * @param packageHeader
 * @return
 */
ERpStatus CDevReferee::SendPackage(ECommandID commandID, SPkgHeader &packageHeader) {

  if (deviceState == RP_RESET) return RP_ERROR;

  return RP_OK;
}


/**
 * @brief
 */
void CDevReferee::UpdateHandler_() {

  if (deviceState == RP_RESET) return;

  if (rxTimestamp_ > lastHeartbeatTime_)
    ResolveRxPackage_();
}


/**
 * @brief
 */
void CDevReferee::HeartbeatHandler_() {

  if (deviceState == RP_RESET) return;

  if (HAL_GetTick() - lastHeartbeatTime_ > 1000) {
    deviceState  = RP_ERROR;
    refereeState = ERefereeStatus::OFFLINE;
  } else {
    deviceState  = RP_OK;
    refereeState = ERefereeStatus::ONLINE;
  }
}


/**
 * @brief
 * @return
 */
ERpStatus CDevReferee::ResolveRxPackage_() {

  if (refereeState == ERefereeStatus::RESET) return RP_ERROR;
  
  /* Get Package Header */
  for (const auto &item: rxBuffer_) {

    if (item != 0xA5) continue;

    auto header = reinterpret_cast<const SPkgHeader *>(&item);
    if (CCrcValidator::Crc8Verify(&item, header->CRC8, 4) != RP_OK)
      continue;

    switch (header->cmdId) {

      case ECommandID::ID_RACE_STATUS: {
        auto pkg = reinterpret_cast<const SRaceStatusPkg *>(&item);
        if (CCrcValidator::Crc16Verify(&item, pkg->CRC16, sizeof(SRaceStatusPkg) - 2) != RP_OK)
          continue;
        raceStatusPkg = *pkg;
        break;
      }

      case ECommandID::ID_RACE_RESULT: {
        auto pkg = reinterpret_cast<const SRaceResultPkg *>(&item);
        if (CCrcValidator::Crc16Verify(&item, pkg->CRC16, sizeof(SRaceResultPkg) - 2) != RP_OK)
          return RP_ERROR;
        raceResultPkg = *pkg;
        break;
      }

      case ECommandID::ID_ROBOT_HP: {
        auto pkg = reinterpret_cast<const SRobotHpPkg *>(&item);
        if (CCrcValidator::Crc16Verify(&item, pkg->CRC16, sizeof(SRobotHpPkg) - 2) != RP_OK)
          return RP_ERROR;
        robotHpPkg = *pkg;
        break;
      }

      case ECommandID::ID_ROBOT_STATUS: {
        auto pkg = reinterpret_cast<const SRobotStatusPkg *>(&item);
        if (CCrcValidator::Crc16Verify(&item, pkg->CRC16, sizeof(SRobotStatusPkg) - 2) != RP_OK)
          return RP_ERROR;
        robotStatusPkg = *pkg;
        break;
      }

      case ECommandID::ID_ROBOT_PERF: {
        auto pkg = reinterpret_cast<const SRobotPerfPkg *>(&item);
        if (CCrcValidator::Crc16Verify(&item, pkg->CRC16, sizeof(SRobotPerfPkg) - 2) != RP_OK)
          return RP_ERROR;
        robotPerfPkg = *pkg;
        break;
      }

      default: {
        return RP_ERROR;
      }
    }
  }

  rxBuffer_.fill(0);
  lastHeartbeatTime_ = HAL_GetTick();
  return RP_OK;
}


/**
 * @brief
 * @param cmdId
 * @return
 */
size_t CDevReferee::GetDataSegmentLength(ECommandID cmdId) {

  switch (cmdId) {
    case ECommandID::ID_RACE_STATUS: return 11;
    case ECommandID::ID_RACE_RESULT: return 1;
    case ECommandID::ID_ROBOT_HP: return 32;
    case ECommandID::ID_VENUE_EVENT: return 4;
    case ECommandID::ID_WARNING: return 3;
    case ECommandID::ID_ROBOT_STATUS: return 13;
    case ECommandID::ID_ROBOT_PERF: return 16;
    case ECommandID::ID_ROBOT_POSIT: return 24;
    case ECommandID::ID_ROBOT_BUFF: return 6;
    case ECommandID::ID_DMG_INFO: return 1;
    case ECommandID::ID_RFID_STATUS: return 4;
    case ECommandID::ID_ROBOT_MSG: return 113;
    default: return 0;
  }
}

} // namespace robotpilots
