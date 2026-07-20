/******************************************************************************
 * @brief
 *
 * @file         sys_controller_link.hpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V2.0
 * @date         2025-04-05
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-17
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#ifndef SYS_CONTROLLER_LINK_HPP
#define SYS_CONTROLLER_LINK_HPP

#include "sys_common.hpp"
#include "Device.hpp"

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

	// 单臂角度结构体（5轴）
	struct SArmAngles {
		float_t yaw = 0.0f;
		float_t pitch1 = 0.0f;
		float_t pitch2 = 0.0f;
		float_t roll = 0.0f;
		float_t pitch_end = 0.0f;
	};

	// ControllerLink信息结构体(Controller -> Robot)
	struct SControllerLinkInfo {
		bool controller_OK = false;          ///< 控制器状态OK
		bool return_success = false;         ///< 归位成功标志
		//EToggleSwitch toggle_switch = TOGGLE_MIDDLE;  ///< 拨杆档位
		bool level_1 = false;          ///< 等级1
		bool level_2 = false;          ///< 等级2
		bool level_3 = false;          ///< 等级3
		bool end_roll_toggle = false;  ///< 末端 roll 翻转状态
		SArmAngles arm;                      ///< 单臂5轴角度
		int8_t rocker_X = 0;                 ///< 摇杆X: roll_end / 底盘左右移动 (-100~100)
		int8_t rocker_Y = 0;                 ///< 摇杆Y: 底盘前进 (-100~100)
	} controllerInfo;

	// 机器人信息结构体(Robot -> Controller)
	struct SRobotInfo {
		bool ask_reset_flag = false;         ///< 是否要求复位
		bool controlled_by_controller = false; ///< 是否被控制器控制
		bool robot_init_ok = false;          ///< 机器人初始化完成
		bool preset_active = false;          ///< preset 进行中
		SArmAngles arm;                      ///< 单臂5轴角度
		SArmAngles torque;                   ///< 臂部力矩/电流反馈（原始值转float）
	} robotInfo;

	// 控制器是否在线（设备层心跳/数据层标志位）
	bool IsControllerOnline() const {
		return pcontrollerLink_
			&& pcontrollerLink_->controllerLinkStatus == CDevControllerLink::EControllerLinkStatus::ONLINE
			&& controllerInfo.controller_OK;
	}

	// 初始化系统
	EAppStatus InitSystem(SSystemInitParam_Base *pStruct) final;

private:

	// 控制器通信设备指针
	CDevControllerLink *pcontrollerLink_ = nullptr;

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

};

extern CSystemControllerLink SysControllerLink;

}   // namespace my_engineer

#endif // SYS_CONTROLLER_LINK_HPP
