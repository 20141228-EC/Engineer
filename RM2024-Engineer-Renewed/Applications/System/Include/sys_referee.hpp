/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-06
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-06   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#ifndef RP_SYS_REFEREE_HPP
#define RP_SYS_REFEREE_HPP

#include "sys_common.hpp"

namespace robotpilots {

class CSysReferee final : public CSysInstance {
public:

  struct SSysRefereeInitParam : public SSystemInitParam {
    EDeviceID refereeDevID = EDeviceID::DEV_NULL;
  };

  struct SRaceInfo {
    int16_t raceType;     ///< Race Type (1 - RMUC, 2 - Deleted, 3 - ICRA, 4 - RMUL 3v3, 5 - RMUL 1v1)
    int16_t raceStage;    ///< Race Stage (1 - Prepare, 2 - 15s Self-check, 3 - 5s Countdown, 4 - Racing, 5 - Settle)
    time_t timeStamp;     ///< Unix Timestamp
  };

  struct SRobotInfo {
    int16_t robotCamp;   ///< Robot Clamp (0 - Unknown, 1 - Red, 2 - Blue)
    int16_t robotID;     ///< Robot ID (0 - Unknown, 1 - 6)
  };

  struct SSysRefereeInfo {
    SRaceInfo race;
    SRobotInfo robot;
  } refereeInfo;

  ERpStatus InitSystem(SSystemInitParam *pStruct) final;

private:

  class CUiFigure {

  };

  CDevReferee *referee_ = nullptr;

  void UpdateHandler_() final;

  void HeartbeatHandler_() final;

  ERpStatus UpdateRaceInfo_();

  ERpStatus UpdateRobotInfo_();

  static void StartSysRefereeTask(void *arg);

};

extern CSysReferee SysReferee;

} // namespace robotpilots

#endif // RP_SYS_REFEREE_HPP
