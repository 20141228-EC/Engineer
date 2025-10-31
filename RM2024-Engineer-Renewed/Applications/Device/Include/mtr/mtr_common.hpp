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

#ifndef RP_MTR_COMMON_HPP
#define RP_MTR_COMMON_HPP

#include "dev_common.hpp"

namespace robotpilots {

class CMtrInstance : public CDevInstance {
protected:

  /**
   * @brief
   */
  struct SMtrInitParam : public SDevInitParam {
    EInterfaceID interfaceId = EInterfaceID::INF_NULL;
  };

//  void HeartbeatHandler_() override;

  ERpStatus RegisterMotor_();

  ERpStatus UnregisteredMotor_();

public:

  /**
   * @brief
   */
  enum class EMotorType {
    MTR_UNDEF = -1,
    MTR_SERVO,
    MTR_M2006,
    MTR_M3508,
    MTR_RM6020,
    MTR_DM4310,
  } motorType = EMotorType::MTR_UNDEF;

  /**
   * @brief
   */
  enum class EMotorStatus {
    RESET = -1,
    OFFLINE,
    STOP,
    RUNNING,
    STALL,
    ERROR,
  } motorState = EMotorStatus::RESET;

  /**
   * @brief
   */
  enum EMotorDataType {
    DATA_UNDEF = -1,
    DATA_SPEED = 0,
    DATA_ANGLE,
    DATA_POSIT,
    DATA_TORQUE,
    DATA_VOLTAGE,
    DATA_CURRENT,
    DATA_TEMP,
    DATA_COUNT_,
  };

  /**
   * @brief
   */
  std::array<int32_t, DATA_COUNT_> motorData = {0};

  /**
   * @brief
   */
  CMtrInstance() { deviceType = EDeviceType::DEV_MTR; }

  /**
   * @brief
   */
  ~CMtrInstance() override { UnregisteredMotor_(); }

  /**
   * @brief Set Motor Output Value
   * @param output[in] Motor Output Value
   *
   * @return
   */
  virtual ERpStatus SetMotorOutput(float_t output) { return RP_ERROR; }

};

extern std::map<EDeviceID, CMtrInstance *> MotorMap;

} // namespace robotpilots

#endif // RP_MTR_COMMON_HPP
