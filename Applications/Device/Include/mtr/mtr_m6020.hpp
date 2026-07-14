/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-12-16
 * @author      Sassinak(1752500338@qq.com)
 * @brief 
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-12-16   <td>1.0         <td>Sassinak  <td>First Create.
 * </table>
 */

#ifndef MTR_M6020_HPP
#define MTR_M6020_HPP

#include "mtr_dji.hpp"

namespace my_engineer {

class CDevMtrM6020 : public CDevMtrDJI {
private:
  /**
   * @brief
   */
  void UpdateHandler_() override {

    if (deviceStatus == APP_RESET) return;

    /* Update Motor Data */
    if (canRxNode_.timestamp >= lastHeartbeatTime_) {
      motorData[DATA_ANGLE]   = (uint16_t)(canRxNode_.dataBuffer[0] << 8 | canRxNode_.dataBuffer[1]);
      motorData[DATA_SPEED]   = (int16_t)(canRxNode_.dataBuffer[2] << 8 | canRxNode_.dataBuffer[3]);
      motorData[DATA_CURRENT] = (int16_t)(canRxNode_.dataBuffer[4] << 8 | canRxNode_.dataBuffer[5]);
      motorData[DATA_POSIT]   = (useAngleToPosit_) ? getPosition_() : 0;
      motorData[DATA_TEMP]    = (int8_t)(canRxNode_.dataBuffer[6]);
      lastHeartbeatTime_  = canRxNode_.timestamp;
    }
  }  
public:

  using SMtrInitParam_M6020 = SMtrInitParam_DJI; 
  CDevMtrM6020(){ motorType = EMotorType::MTR_M6020;};
};


}

#endif