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

#include "dev_vision.hpp"

namespace robotpilots {

/**
 * @brief
 * @param pStruct
 *
 * @return
 */
ERpStatus CDevVision::InitDevice(const SDevInitParam *pStruct) {

  if (pStruct == nullptr) return RP_ERROR;
  if (pStruct->deviceID == EDeviceID::DEV_NULL) return RP_ERROR;

  auto &param = *reinterpret_cast<const SDevVisionInitParam *>(pStruct);
  deviceId = param.deviceID;
  uartInterface_ = reinterpret_cast<CUartInterface *>(InterfaceMap.at(param.interfaceId));

  auto callback = [this](auto &buffer, auto len) {
    if (len > 256 + sizeof(SPkgHeader)) return;
    std::copy(buffer.data(), buffer.data() + len, rxBuffer_);
    rxTimestamp_ = HAL_GetTick();
  };

  RegisterDevice_();
  uartInterface_->RegisterCallback(callback);

  deviceState = RP_OK;
  visionState = EVisionStatus::OFFLINE;

  return RP_OK;
}


/**
 * @brief
 * @param packageID
 * @param packageHeader
 * @return
 */
ERpStatus CDevVision::SendPackage(EPackageID packageID, SPkgHeader &packageHeader) {

  if (deviceState == RP_RESET) return RP_ERROR;

  switch (packageID) {

    case ID_ORETANK_INFO: {
      auto pkg = reinterpret_cast<SOretankinfoPkg *>(&packageHeader);
      pkg->header.SOF = 0xA5;
      pkg->header.id = ID_ORETANK_INFO;
      pkg->header.len = sizeof(SOretankinfoPkg) - sizeof(SPkgHeader) - 2;
      pkg->header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&pkg->header), sizeof(SPkgHeader) - 1);
      pkg->CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(pkg), sizeof(SOretankinfoPkg) - 2);

      return uartInterface_->Transmit(reinterpret_cast<uint8_t *>(pkg), sizeof(SOretankinfoPkg));
    }

    case ID_RACE_INFO: {
      auto pkg = reinterpret_cast<SRaceinfoPkg *>(&packageHeader);
      pkg->header.SOF = 0xA5;
      pkg->header.id = ID_RACE_INFO;
      pkg->header.len = sizeof(SRaceinfoPkg) - sizeof(SPkgHeader) - 2;
      pkg->header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&pkg->header), sizeof(SPkgHeader) - 1);
      pkg->CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(pkg), sizeof(SRaceinfoPkg) - 2);

      return uartInterface_->Transmit(reinterpret_cast<uint8_t *>(pkg), sizeof(SRaceinfoPkg));
    }

    default: { return RP_ERROR; }
  }
}


/**
 * @brief
 */
void CDevVision::UpdateHandler_() {

  if (deviceState == RP_RESET) return;

  if (rxTimestamp_ > lastHeartbeatTime_)
    ResolveRxPackage_();
}


/**
 * @brief
 */
void CDevVision::HeartbeatHandler_() {

  if (deviceState == RP_RESET) return;

  if (HAL_GetTick() - lastHeartbeatTime_ > 1000) {
    deviceState = RP_ERROR;
    visionState = EVisionStatus::OFFLINE;
  } else {
    deviceState = RP_OK;
    visionState = EVisionStatus::ONLINE;
  }
}


/**
 * @brief
 */
ERpStatus CDevVision::ResolveRxPackage_() {

  if (deviceState == RP_RESET) return RP_ERROR;

  /* Get Package Header */
  auto header = reinterpret_cast<SPkgHeader *>(rxBuffer_);

  if (header->SOF != 0xA5) return RP_ERROR;
  if (CCrcValidator::Crc8Verify(rxBuffer_, header->CRC8, sizeof(SPkgHeader) - 1) != RP_OK)
    return RP_ERROR;

  /* Resolve Package */
  switch (header->id) {

    case ID_ORETANK_INFO: {   // Ore Tank Position Information Package
      auto pkg = reinterpret_cast<SOretankinfoPkg *>(rxBuffer_);
      if (CCrcValidator::Crc16Verify(rxBuffer_, pkg->CRC16, sizeof(SOretankinfoPkg) - 2) != RP_OK)
        return RP_ERROR;
      oretankinfoPkg = *pkg;
      break;
    }

    default: { return RP_ERROR; }
  }

  lastHeartbeatTime_ = HAL_GetTick();
  return RP_OK;
}

} // namespace robotpilots
