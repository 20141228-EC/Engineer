/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-03-18
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-18   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "rc/remote.hpp"

namespace robotpilots {

/**
 * @brief
 */
void CRcInstance::HeartbeatHandler_() {

  if (deviceState == RP_RESET) return;

  remoteState = ((HAL_GetTick() - lastHeartbeatTime_) > 500) ? ERcStatus::OFFLINE : ERcStatus::ONLINE;

  if (remoteState == ERcStatus::ONLINE)
    UpdateChannel_();
}

/**
 * @brief
 * @param num
 * @param type
 * @return
 */
ERpStatus CRcInstance::InitChannel_(size_t num, EChannelType type) {

  if (num > remoteData.size()) return RP_ERROR;

  remoteData[num].chType = type;
  remoteData[num].chState = EChannelStatus::RESET;
  remoteData[num].chValue = 0;

  return RP_OK;
}

/**
 * @brief
 * @return
 */
ERpStatus CRcInstance::UpdateChannel_() {

  if (deviceState == RP_RESET) return RP_ERROR;

  for (auto &item: remoteData) {
    item.chState = EChannelStatus::RESET;

    switch (item.chType) {

      case EChannelType::LEVER:
        if (item > 50) item.chState = EChannelStatus::HIGH;
        if (item < -50) item.chState = EChannelStatus::DOWN;
        break;

      case EChannelType::BUTTON:
        if (item == 1) item.chState = EChannelStatus::PRESS;
        break;

      default:
        break;
    }
  }

  return RP_OK;
}

} // namespace robotpilots
