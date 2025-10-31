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
    if (len > 512) return;
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
//  for (const auto &item: rxBuffer_) {
//
//    if (item != 0xA5) continue;
//
//    auto header = reinterpret_cast<const SPkgHeader *>(&item);
//    if (CCrcValidator::Crc8Verify(&item, header->CRC8, 4) != RP_OK)
//      continue;
//
//    switch (header->cmdId) {
//
//      case ECommandID::ID_RACE_STATUS: {
//        auto pkg = reinterpret_cast<const SRaceStatusPkg *>(&item);
//        if (CCrcValidator::Crc16Verify(&item, pkg->CRC16, sizeof(SRaceStatusPkg) - 2) != RP_OK)
//          continue;
//        raceStatusPkg = *pkg;
//        break;
//      }
//
//      case ECommandID::ID_RACE_RESULT: {
//        auto pkg = reinterpret_cast<const SRaceResultPkg *>(&item);
//        if (CCrcValidator::Crc16Verify(&item, pkg->CRC16, sizeof(SRaceResultPkg) - 2) != RP_OK)
//          return RP_ERROR;
//        raceResultPkg = *pkg;
//        break;
//      }
//
//      case ECommandID::ID_ROBOT_HP: {
//        auto pkg = reinterpret_cast<const SRobotHpPkg *>(&item);
//        if (CCrcValidator::Crc16Verify(&item, pkg->CRC16, sizeof(SRobotHpPkg) - 2) != RP_OK)
//          return RP_ERROR;
//        robotHpPkg = *pkg;
//        break;
//      }
//
//      case ECommandID::ID_ROBOT_STATUS: {
//        auto pkg = reinterpret_cast<const SRobotStatusPkg *>(&item);
//        if (CCrcValidator::Crc16Verify(&item, pkg->CRC16, sizeof(SRobotStatusPkg) - 2) != RP_OK)
//          return RP_ERROR;
//        robotStatusPkg = *pkg;
//        break;
//      }
//
//      case ECommandID::ID_ROBOT_PERF: {
//        auto pkg = reinterpret_cast<const SRobotPerfPkg *>(&item);
//        if (CCrcValidator::Crc16Verify(&item, pkg->CRC16, sizeof(SRobotPerfPkg) - 2) != RP_OK)
//          return RP_ERROR;
//        robotPerfPkg = *pkg;
//        break;
//      }
//
//      default: {
//        return RP_ERROR;
//      }
//    }
//  }

  for (size_t i = 0; i < rxBuffer_.size(); i++) {

    if (rxBuffer_[i] != 0xA5)
      continue;

    auto header = reinterpret_cast<SPkgHeader *>(&rxBuffer_[i]);
    if (CCrcValidator::Crc8Verify(reinterpret_cast<uint8_t *>(header), header->CRC8, 4) != RP_OK)
      continue;

    switch (header->cmdId) {

      case ECommandID::ID_RACE_STATUS: {
        if (i + sizeof(SRaceStatusPkg) > rxBuffer_.size())
          break;
        auto pkg = reinterpret_cast<SRaceStatusPkg *>(header);
        if (CCrcValidator::Crc16Verify(reinterpret_cast<uint8_t *>(pkg), pkg->CRC16, sizeof(SRaceStatusPkg) - 2) != RP_OK)
          break;
        raceStatusPkg = *pkg;
        i += sizeof(SRaceStatusPkg) - 1;
        break;
      }

      case ECommandID::ID_RACE_RESULT: {
        if (i + sizeof(SRaceResultPkg) > rxBuffer_.size())
          break;
        auto pkg = reinterpret_cast<SRaceResultPkg *>(header);
        if (CCrcValidator::Crc16Verify(reinterpret_cast<uint8_t *>(pkg), pkg->CRC16, sizeof(SRaceResultPkg) - 2) != RP_OK)
          break;
        raceResultPkg = *pkg;
        i += sizeof(SRaceResultPkg) - 1;
        break;
      }

      case ECommandID::ID_ROBOT_HP: {
        if (i + sizeof(SRobotHpPkg) > rxBuffer_.size())
          break;
        auto pkg = reinterpret_cast<SRobotHpPkg *>(header);
        if (CCrcValidator::Crc16Verify(reinterpret_cast<uint8_t *>(pkg), pkg->CRC16, sizeof(SRobotHpPkg) - 2) != RP_OK)
          break;
        robotHpPkg = *pkg;
        i += sizeof(SRobotHpPkg) - 1;
        break;
      }

      case ECommandID::ID_ROBOT_STATUS: {
        if (i + sizeof(SRobotStatusPkg) > rxBuffer_.size())
          break;
        auto pkg = reinterpret_cast<SRobotStatusPkg *>(header);
        if (CCrcValidator::Crc16Verify(reinterpret_cast<uint8_t *>(pkg), pkg->CRC16, sizeof(SRobotStatusPkg) - 2) != RP_OK)
          return RP_ERROR;
        robotStatusPkg = *pkg;
        i += sizeof(SRobotStatusPkg) - 1;
        break;
      }

      case ECommandID::ID_ROBOT_PERF: {
        if (i + sizeof(SRobotPerfPkg) > rxBuffer_.size())
          break;
        auto pkg = reinterpret_cast<SRobotPerfPkg *>(header);
        if (CCrcValidator::Crc16Verify(reinterpret_cast<uint8_t *>(pkg), pkg->CRC16, sizeof(SRobotPerfPkg) - 2) != RP_OK)
          break;
        robotPerfPkg = *pkg;
        i += sizeof(SRobotPerfPkg) - 1;
        break;
      }

      default: {
        break;
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
