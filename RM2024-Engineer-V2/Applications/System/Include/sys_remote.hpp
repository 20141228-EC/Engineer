/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-03-22
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-22   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#ifndef RP_SYS_REMOTE_HPP
#define RP_SYS_REMOTE_HPP

#include "sys_common.hpp"
#include "Device.hpp"

namespace robotpilots {

/**
 * @brief
 */
class CSysRemote final : public CSysInstance {
public:

  /**
   * @brief
   */
  struct SSysRemoteInitParam : public SSystemInitParam {
    EDeviceID remoteDevID = EDeviceID::DEV_NULL;
    EDeviceID refereeDevID = EDeviceID::DEV_NULL;
  };

  /**
   * @brief
   */
  struct SRemoteInfo {
    float_t joystick_LX = 0;
    float_t joystick_LY = 0;
    float_t joystick_RX = 0;
    float_t joystick_RY = 0;
    float_t thumbWheel = 0;
    uint8_t switch_L = 0;
    uint8_t switch_R = 0;
  };

  /**
   * @brief
   */
  struct SKeyboardInfo {
    int16_t mouse_X = 0;
    int16_t mouse_Y = 0;
    int16_t mouse_Thumb = 0;
    bool mouse_L = false;
    bool mouse_R = false;
    bool key_W = false;
    bool key_A = false;
    bool key_S = false;
    bool key_D = false;
    bool key_Q = false;
    bool key_E = false;
    bool key_R = false;
    bool key_F = false;
    bool key_G = false;
    bool key_Z = false;
    bool key_X = false;
    bool key_C = false;
    bool key_V = false;
    bool key_B = false;
    bool key_Ctrl = false;
    bool key_Shift = false;
  };

  /**
   * @brief
   */
  struct SRemoteInfoPackage {
    SRemoteInfo remote;
    SKeyboardInfo keyboard;
  } remoteInfo;

  /**
   * @brief
   * @param pStruct
   * @return
   */
  ERpStatus InitSystem(SSystemInitParam *pStruct) final;

private:

  /**
   * @brief
   */
  CRcInstance *remote_;

  CDevReferee *referee_;

  /**
   * @brief
   */
  void UpdateHandler_() final;

  /**
   * @brief
   */
  void HeartbeatHandler_() final;

  /**
   * @brief
   * @return
   */
  ERpStatus UpdateRemote_();

  /**
   * @brief
   * @return
   */
  ERpStatus UpdateKeyboard_();

  /**
   * @brief
   * @return
   */
  ERpStatus UpdateReferee_();

};

extern CSysRemote SysRemote;

} // namespace robotpilots

#endif // RP_SYS_REMOTE_HPP
