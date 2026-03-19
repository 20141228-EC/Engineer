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
#include <algorithm>

/*选不同难度时，x和y的位置*/
#define POSIT_LEVEL3_X 0
#define POSIT_LEVEL3_Y 0
#define POSIT_LEVEL4_X 0
#define POSIT_LEVEL4_Y 0

/*确定键的位置*/
#define POSIT_YES_X 0
#define POSIT_YES_Y 0

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
	 * @brief 系统层使用float
	 * @note  末端Roll由摇杆控制
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
		EToggleSwitch toggle_switch = TOGGLE_MIDDLE;  ///< 拨杆档位
		bool gripper_left_close = false;     ///< 左夹爪闭合
		bool gripper_right_close = false;    ///< 右夹爪闭合
		bool gripper_right_regrip = false;   ///< 右夹爪二次夹紧请求（脉冲信号）
		SArmAngles left_arm;                 ///< 左臂5轴角度
		SArmAngles right_arm;                ///< 右臂5轴角度
		int8_t rocker_LX = 0;                ///< 左臂roll_end(-100~100)
		int8_t rocker_RX = 0;                ///< 右臂roll_end(-100~100) / 底盘左右移动
		int8_t rocker_RY = 0;                ///< 底盘前进 (-100~100)，仅底盘模式有效
	} controllerInfo;

	// 机器人信息结构体(Robot -> Controller)
	struct SRobotInfo {
		bool ask_reset_flag = false;           ///< 要求复位
		bool controlled_by_controller = false; ///< 被控制器控制中
		bool robot_init_ok = false;            ///< 机器人初始化完成
		SArmAngles left_arm;                   ///< 左臂5轴角度
		SArmAngles right_arm;                  ///< 右臂5轴角度
	} robotInfo;

	// 初始化系统
	EAppStatus InitSystem(SSystemInitParam_Base *pStruct) final;

	// 控制器通信设备指针
	CDevControllerLink *pcontrollerLink_ = nullptr;
	CDevFourButton *pbuttons_ = nullptr; ///< 按键设备指针

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

	/*根据难度等级来更新键鼠信息*/
	EAppStatus Level4Move_();
	EAppStatus Level3Move_();
	EAppStatus Mouse_move_(uint16_t pos_x, uint16_t pos_y, uint8_t mouse_left, uint8_t mouse_right);
	EAppStatus KeyBoard_move_(uint8_t key_value1, uint8_t key_value2);

};

extern CSystemControllerLink SysControllerLink;

}   // namespace my_engineer

#endif // SYS_CONTROLLER_LINK_HPP
