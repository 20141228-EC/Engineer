/******************************************************************************
 * @brief
 *
 * @file         sys_controller_link.hpp
 * @author       ciallo (1002046597@qq.com)
 * @version      V2.0
 * @date         2026-06-03
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-06-03
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#ifndef SYS_CONTROLLER_LINK_HPP
#define SYS_CONTROLLER_LINK_HPP

#include "sys_common.hpp"
#include "Device.hpp"
#include <algorithm>

namespace my_engineer {

/**
 * @brief 控制器通信系统类
 *
 */
class CSystemControllerLink final: public CSystemBase{
public:

	// 定义控制器通信系统初始化参数结构体
	struct SSystemInitParam_ControllerLink: public SSystemInitParam_Base{
		EDeviceID controllerLinkDevID = EDeviceID::DEV_NULL; ///< 控制器通信设备ID
	};

	// 拨杆档位枚举
	enum EToggleSwitch : uint8_t {
		TOGGLE_MIDDLE = 0,     ///< 中档
		TOGGLE_ARM_ROLL = 1,   ///< 臂Roll末端模式
		TOGGLE_CHASSIS = 2,    ///< 底盘模式
	};

	/**
	 * @brief 系统层使用float (5轴)
	 */
	struct SArmAngles {
		float_t yaw = 0.f;
		float_t pitch1 = 0.f;
		float_t pitch2 = 0.f;
		float_t roll = 0.f;
		float_t pitch_end = 0.f;
	};

	// ControllerLink信息结构体(Controller -> Robot)
	struct SControllerLinkInfo {
		bool controller_OK = false;          ///< 控制器状态OK
		bool return_success = false;         ///< 归位成功标志
		// EToggleSwitch toggle_switch = TOGGLE_MIDDLE;  ///< 拨杆档位
		bool gripper_close = false;          ///< 夹爪闭合
		bool gripper_regrip = false;         ///< 夹爪二次夹紧请求
		SArmAngles arm;                      ///< 单臂5轴角度
		// int8_t rocker_X = 0;                 ///< 摇杆X: roll_end / 底盘左右移动 (-100~100)
		// int8_t rocker_Y = 0;                 ///< 摇杆Y: 底盘前进 (-100~100)，仅底盘模式有效
	} controllerInfo;

	// 机器人信息结构体(Robot -> Controller)
	struct SRobotInfo {
		bool ask_reset_flag = false;           ///< 要求复位
		bool controlled_by_controller = false; ///< 被控制器控制中
		bool robot_init_ok = false;            ///< 机器人初始化完成
		bool p3_lock = false;                   ///< 保留（机器人端协议兼容）
		SArmAngles arm;                        ///< 单臂5轴角度
		SArmAngles torque;                     ///< 臂部力矩/电流反馈（原始值转float）
	} robotInfo;

	// 初始化系统
	EAppStatus InitSystem(SSystemInitParam_Base *pStruct) final;

	// 控制器通信设备指针
	CDevControllerLink *pcontrollerLink_ = nullptr;
	CDevButton *pbuttons_ = nullptr; ///< 按键设备指针

private:

	void UpdateHandler_() final;

	void HeartbeatHandler_() final;

	// 更新ControllerLink信息
	void UpdateControllerLinkInfo_();

	// 更新Robot信息
	void UpdateRobotInfo_();

	// 更新发送数据包 RobotData
	void UpdateRobotDataPkg_();

	// 更新发送数据包 ControllerData
	void UpdateControllerDataPkg_();

	/*更新按键信息*/
	void UpdateButtonInfo_();

	/*选难度状态机*/
	enum class ELevelStep : uint8_t {
		IDLE = 0,
		KEY_PRESS,
		KEY_RELEASE,
		MOVE_TO_LEVEL,
		CLICK_LEVEL,
		RELEASE_CLICK_LEVEL,
		MOVE_TO_YES,
		CLICK_YES,
		RELEASE_CLICK_YES,
	};

	ELevelStep levelStep_ = ELevelStep::IDLE;
	uint16_t levelTargetX_ = 0;
	uint16_t levelTargetY_ = 0;

	///< 难度位置查找表 [level][x,y]
	const uint16_t Level_Positions[4][2] = {
		{833, 511},  ///< LEVEL_1
		{833, 551},  ///< LEVEL_2
		{833, 592},  ///< LEVEL_3
		{833, 633},  ///< LEVEL_4
	};

	///< 确定键的位置
	const uint16_t Yes_Position[2] = {841, 765};

	void StartLevelChoose_(uint8_t level);
	void TickLevelChoose_();

};

extern CSystemControllerLink SysControllerLink;

}   // namespace my_engineer

#endif // SYS_CONTROLLER_LINK_HPP
