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

/**
 * @brief Referee System Class
 */
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
    time_t unixTimestamp;
    SRaceInfo race;
    SRobotInfo robot;
  } refereeInfo;

  ERpStatus InitSystem(SSystemInitParam *pStruct) final;

private:

  enum EUiConfigID {
    TEXT_PUMP = 0,
    TEXT_MODE,
    TEXT_CURRENT_MODE,
    CIRCLE_PUMP_C,
    CIRCLE_PUMP_L,
    CIRCLE_PUMP_R,
    LINE_L,
    LINE_R,
  };

  std::array<CDevReferee::SUiFigureConfig, 8> uiConfig;

  CDevReferee::SRobotMsgPkg<CDevReferee::SUiDrawTextMsg> pumpTextMsg, modeTextMsg, curModeTextMsg;

  CDevReferee::SRobotMsgPkg<CDevReferee::SUiDrawPentaMsg> visionFigureMsg;

  CDevReferee::SRobotMsgPkg<CDevReferee::SUiDrawHeptaMsg> stateFigureMsg;

  CDevReferee *referee_ = nullptr;

  CUartInterface * interface_ = nullptr;

  void UpdateHandler_() final;

  void HeartbeatHandler_() final;

  ERpStatus UpdateRaceInfo_();

  ERpStatus UpdateRobotInfo_();

  void UI_InitDrawing();

  void UI_StartStaticTextDrawing_();

  void UI_StartCurModeTextDrawing_();

  void UI_StartStateFigureDrawing_();

  void UI_StartVisionFigureDrawing_();

  void UI_UpdateCurModeTextDrawing();

  void UI_UpdateStateFigureDrawing_();

  void UI_UpdateVisionFigureDrawing_();

  static void StartSysRefereeUiTask(void *arg);

};

extern CSysReferee SysReferee;

} // namespace robotpilots

#endif // RP_SYS_REFEREE_HPP
