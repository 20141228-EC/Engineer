/******************************************************************************
 * @brief        
 * 
 * @file         sllllr.hpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-03-16
 * @note		 后缀为TextMsg的为动态ui，后缀为FigureMsg的为静态ui
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/

#ifndef SYS_REFEREE_HPP
#define SYS_REFEREE_HPP

#include "sys_common.hpp"
#include "Module.hpp"
#include "Device.hpp"

namespace my_engineer {

/**
 * @brief 裁判系统类
 * 
 */
class CSystemReferee final: public CSystemBase{
public:
	// 定义裁判系统初始化参数结构体
	struct SSystemInitParam_Referee: public SSystemInitParam_Base{
		EDeviceID refereeDevID = EDeviceID::DEV_NULL; ///< 裁判系统设备ID
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

	struct SRadarInfo {
		bool if_dart_comming = false; ///< 是否有飞镖来袭
	};

	struct SSysRefereeInfo {
		time_t unixTimestamp;
		SRaceInfo race;
		SRobotInfo robot;
		SRadarInfo radar;
	} refereeInfo;

	// 初始化系统
	EAppStatus InitSystem(SSystemInitParam_Base *pStruct) final;

	CModChassis::SChassisInfo chassis;

private:
	enum EUiConfigID {
		TEXT_PUMP = 0,
		TEXT_MODE,
		TEXT_CURRENT_MODE,
		CRAWLER_STATUS,
	};

	std::array<CDevReferee::SUiFigureConfig, 13> uiConfig;

	CDevReferee::SRobotMsgPkg<CDevReferee::SUiDrawTextMsg> hipTextMsg, crawlerTextMsg, modeTextMsg, curModeTextMsg, hipInfoTextMsg, p3LockTextMsg;

	CDevReferee::SRobotMsgPkg<CDevReferee::SUiDrawPentaMsg> visionFigureMsg;

	CDevReferee::SRobotMsgPkg<CDevReferee::SUiDrawTextMsg> RadarTextMsg;

	CDevReferee::SRobotMsgPkg<CDevReferee::SUiDrawHeptaMsg> stateFigureMsg;

	CDevReferee::SRobotMsgPkg<CDevReferee::SUiDrawTextMsg> pitchTextMsg, pitchStaticTextMsg;

	CDevReferee::SRobotMsgPkg<CDevReferee::SUiDrawDoubleMsg> positionFigureMsg, parallelFigureMsg;
	
	CDevReferee::SRobotMsgPkg<CDevReferee::SUiDrawPentaMsg> armAngleFigureMsg, armYawFigureMsg;

	CDevReferee::SRobotMsgPkg<CDevReferee::SUiDrawSingleMsg> p3LockMsg;

    CDevReferee *pRefereeDev_ = nullptr;

    CInfUART * pInterface_ = nullptr;

	void UpdateHandler_() final;

    void HeartbeatHandler_() final;

	EAppStatus UpdateRaceInfo_();

	EAppStatus UpdateRobotInfo_();

	EAppStatus UpdateRadarInfo_();

	EAppStatus UpdateControllerInfo_();

	void UI_InitDrawing();

	void UI_StartStaticTextDrawing_();

	void UI_StartCurModeTextDrawing_();

	void UI_StartRadarTextDrawing_();

	void UI_StartStateFigureDrawing_();

	void UI_StartVisionFigureDrawing_();

	void UI_StartHipTextDrawing_();

	void UI_StartPitchTextDrawing_();

	void UI_UpdateCurModeTextDrawing_();

	void UI_UpdateStateFigureDrawing_();

	void UI_UpdateVisionFigureDrawing_();

	void UI_UpdateHipTextDrawing_();

	void UI_UpdatePitchTextDrawing_();

	void UI_StartPositionFigureDrawing_();

	void UI_StartArmAngleFigureDrawing_();
	void UI_StartArmYawFigureDrawing_();

	void UI_UpdateArmAngleFigureDrawing_();
	void UI_UpdateArmYawFigureDrawing_();

	void UI_UpdatePositionFigureDrawing_();

	void UI_StartParallelFigureDrawing_();

	void UI_RADAR_WARNING_TextDrawing_();
	void UI_RADAR_WARNING_TextClearing_();

	static void StartSysRefereeUiTask(void *arg);

};

extern CSystemReferee SysReferee;

}   // namespace my_engineer

#endif // SYS_REFEREE_HPP
