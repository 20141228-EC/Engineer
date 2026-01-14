/******************************************************************************
 * @brief        
 * 
 * @file         sys_controller_link.hpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-04-05
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

	enum KEY_STATUS  {RELEASE = 0, PRESS = 1, LONG_PRESS = 2,};

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
		bool controller_OK = false;
		bool return_success = false;
		int8_t Rocker_X = 0;           ///< 摇杆X轴 (-100~100)
		int8_t Rocker_Y = 0;           ///< 摇杆Y轴 (-100~100)
		KEY_STATUS Rocker_Key = KEY_STATUS::RELEASE; ///< 摇杆按键状态
		uint8_t toggle_switch = 0;     ///< 拨杆状态
		uint8_t button = 0;            ///< 按钮状态
		bool isReset = false; ///< 是否复位
		bool isLevel4 = false; ///< 是否启用四级难度
		bool isLevel3 = false; ///< 是否启用三级难度
		bool isSelf = false; ///< 是否启用自定义按键
		SArmAngles left_arm;   ///< 左臂角度
		SArmAngles right_arm;  ///< 右臂角度
	} controllerInfo;

	// 机器人信息结构体(Robot -> Controller)
	struct SRobotInfo {
		bool ask_reset_flag = false; ///< 是否要求复位
		bool controlled_by_controller = false; ///< 是否被控制器控制
		bool ask_return_flag = false; ///< 是否要求归位
		SArmAngles left_arm;   ///< 左臂角度
		SArmAngles right_arm;  ///< 右臂角度
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

	// 角度压缩：float -> int16_t (精度：0.01°，范围：±327.67°)
	static inline int16_t CompressAngle_(float angle) {
		return static_cast<int16_t>(std::clamp(angle * 100.0f, -32767.0f, 32767.0f));
	}

	// 角度解压：int16_t -> float
	static inline float DecompressAngle_(int16_t compressed) {
		return static_cast<float>(compressed) * 0.01f;
	}

};

extern CSystemControllerLink SysControllerLink;

}   // namespace my_engineer

#endif // SYS_CONTROLLER_LINK_HPP
