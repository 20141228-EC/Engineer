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
		EToggleSwitch toggle_switch = TOGGLE_MIDDLE;  ///< 拨杆档位
		bool gripper_left_close = false;     ///< 左夹爪闭合
		bool gripper_right_close = false;    ///< 右夹爪闭合
		SArmAngles left_arm;                 ///< 左臂5轴角度
		SArmAngles right_arm;                ///< 右臂5轴角度
		int8_t rocker_LX = 0;                ///< 左臂roll_end增量 (-100~100)
		int8_t rocker_RX = 0;                ///< 右臂roll_end增量 (-100~100)
		int8_t rocker_RY = 0;                ///< 底盘前进
	} controllerInfo;

	// 机器人信息结构体(Robot -> Controller)
	struct SRobotInfo {
		bool ask_reset_flag = false;         ///< 是否要求复位
		bool controlled_by_controller = false; ///< 是否被控制器控制
		bool ask_return_flag = false;        ///< 是否要求归位
		SArmAngles left_arm;                 ///< 左臂5轴角度
		SArmAngles right_arm;                ///< 右臂5轴角度
	} robotInfo;

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
