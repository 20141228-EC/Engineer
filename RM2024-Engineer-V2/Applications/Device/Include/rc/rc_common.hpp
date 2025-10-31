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
#ifndef RP_RC_COMMON_HPP
#define RP_RC_COMMON_HPP

#include "dev_common.hpp"
#include "rc_def.hpp"

namespace robotpilots {

class CRcInstance : public CDevInstance {
protected:
  /**
   * @brief
   */
  struct SRcInitParam : public SDevInitParam {
    EInterfaceID interfaceId = EInterfaceID::INF_NULL;
  };

  /**
   * @brief
   */
  void HeartbeatHandler_() override;

  /**
   * @brief
   * @param num
   * @param type
   * @return
   */
  ERpStatus InitChannel_(size_t num, EChannelType type);

  /**
   * @brief
   * @return
   */
  ERpStatus UpdateChannel_();


public:

  /**
   * @brief
   */
  class CRcChannel {
  public:

    EChannelType chType = EChannelType::UNDEF;

    EChannelStatus chState = EChannelStatus::RESET;

    int16_t chValue = 0;

    explicit operator int16_t() const { return chValue; }

    bool operator== (const EChannelStatus &state) const { return chState == state; }

    bool operator!= (const EChannelStatus &state) const { return chState != state; }

    bool operator> (const int16_t &value) const { return chValue > value; }

    bool operator< (const int16_t &value) const { return chValue < value; }

    bool operator== (const int16_t &value) const { return chValue == value; }

    bool operator!= (const int16_t &value) const { return chValue != value; }

  };

  /**
   * @brief
   */
  ERcType remoteType = ERcType::UNDEF;

  /**
   * @brief
   */
  ERcStatus remoteState = ERcStatus::RESET;

  /**
   * @brief
   */
  std::vector<CRcChannel> remoteData;

  /**
   * @brief
   */
  CRcInstance() { deviceType = EDeviceType::DEV_RC; }

  /**
   * @brief
   */
  ~CRcInstance() override { UnregisteredDevice_(); }

};

} // namespace robotpilots

#endif // RP_RC_COMMON_HPP
