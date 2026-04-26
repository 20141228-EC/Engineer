/******************************************************************************
 * @brief
 *
 * @file         sys_controller_link.cpp
 * @author       Ciallo (1002046597@qq.com)
 * @version      V2.0
 * @date         2025-04-05
 * @LastEditTime 2026-01-17
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#include "sys_controller_link.hpp"

namespace my_engineer {

#define I_AM_CONTROLLER 0 // 当前板子是副板，控制机器人

// 实例化一个控制器通信系统
CSystemControllerLink SysControllerLink;

/**
 * @brief 初始化控制器通信系统
 *
 * @param pStruct
 * @return EAppStatus
 */
EAppStatus CSystemControllerLink::InitSystem(SSystemInitParam_Base *pStruct) {

	// 检查参数及ID是否为空
	if (pStruct == nullptr) return APP_ERROR;
	if (pStruct->systemID == ESystemID::SYS_NULL) return APP_ERROR;

	// 类型转换
	auto &param = *reinterpret_cast<SSystemInitParam_ControllerLink *>(pStruct);

	// 初始化控制器通信设备
	systemID = param.systemID;
	auto it = DeviceIDMap.find(param.controllerLinkDevID);
	if (it != DeviceIDMap.end() && it->second != nullptr) {
		pcontrollerLink_ = static_cast<CDevControllerLink *>(it->second);
	}

	// 注册系统
	RegisterSystem_();

	systemStatus = APP_ERROR;
	return APP_OK;
}

/**
 * @brief 更新处理
 *
 */
void CSystemControllerLink::UpdateHandler_() {
	// 检查系统状态
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	#if I_AM_CONTROLLER == 0
		// 接收：全速解析控制器发来的最新数据（25Hz的数据包）
		UpdateControllerLinkInfo_();

		// 发送：独立限频 10Hz
		// 系统任务1kHz
		static uint8_t sendDelay = 100;
		//sendDelay -= 4;
		sendDelay -= 1;
		if (sendDelay == 0) {
			sendDelay = 100;
			UpdateRobotDataPkg_();
			pcontrollerLink_->SendPackage(CDevControllerLink::ID_ROBOT_DATA, pcontrollerLink_->robotData_info_pkg.header);
		}
	#else
		// 更新机器人信息
		UpdateRobotInfo_();
		// 更新发送数据包
		UpdateControllerDataPkg_();
		// 发送控制器信息
		pcontrollerLink_->SendPackage(CDevControllerLink::ID_CONTROLLER_DATA, pcontrollerLink_->controllerData_info_pkg.header);
	#endif
}

/**
 * @brief 更新控制器信息 (设备层 -> 系统层)
 * 从压缩数据包解压到系统层结构体
 */
void CSystemControllerLink::UpdateControllerLinkInfo_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->controllerData_info_pkg;

	// 解析状态标志位
	controllerInfo.controller_OK = pkg.status_flags.controller_init_ok;
	controllerInfo.return_success = pkg.status_flags.return_sucess;
	//controllerInfo.toggle_switch = static_cast<EToggleSwitch>((pkg.status_flags & STATUS_TOGGLE_MASK) >> STATUS_TOGGLE_SHIFT);
	controllerInfo.gripper_close = pkg.status_flags.grip;
	controllerInfo.gripper_regrip = pkg.status_flags.regrip;

	// 单臂角度数据 (float直传)
	controllerInfo.arm.yaw       = pkg.arm.yaw;
	controllerInfo.arm.pitch1    = pkg.arm.pitch1;
	controllerInfo.arm.pitch2    = pkg.arm.pitch2;
	controllerInfo.arm.pitch3	= pkg.arm.pitch3;
	controllerInfo.arm.roll      = pkg.arm.roll;
	controllerInfo.arm.pitch_end = pkg.arm.pitch_end;

	// 摇杆数据
	controllerInfo.rocker_X = pkg.rocker_X;
	controllerInfo.rocker_Y = pkg.rocker_Y;
}

/**
 * @brief 更新机器人信息 (设备层 -> 系统层)
 *
 */
void CSystemControllerLink::UpdateRobotInfo_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->robotData_info_pkg;

	// 解析状态标志位
	robotInfo.ask_reset_flag = pkg.status_flags.ask_reset;
	robotInfo.controlled_by_controller = pkg.status_flags.control_by_controller;
	robotInfo.robot_init_ok = pkg.status_flags.robot_init_ok;
	robotInfo.p3_lock = pkg.status_flags.p3_lock;

	// 解压角度 (int16 -> float)
	robotInfo.arm.yaw       = CDevControllerLink::DecompressAngle(pkg.arm.yaw);
	robotInfo.arm.pitch1    = CDevControllerLink::DecompressAngle(pkg.arm.pitch1);
	robotInfo.arm.pitch2    = CDevControllerLink::DecompressAngle(pkg.arm.pitch2);
	robotInfo.arm.pitch3    = CDevControllerLink::DecompressAngle(pkg.arm.pitch3);
	robotInfo.arm.roll      = CDevControllerLink::DecompressAngle(pkg.arm.roll);
	robotInfo.arm.pitch_end = CDevControllerLink::DecompressAngle(pkg.arm.pitch_end);

	// 力矩/电流反馈 (int16 -> float)
	robotInfo.torque.yaw       = static_cast<float>(pkg.torque.yaw);
	robotInfo.torque.pitch1    = static_cast<float>(pkg.torque.pitch1);
	robotInfo.torque.pitch2    = static_cast<float>(pkg.torque.pitch2);
	robotInfo.torque.pitch3    = static_cast<float>(pkg.torque.pitch3);
	robotInfo.torque.roll      = static_cast<float>(pkg.torque.roll);
	robotInfo.torque.pitch_end = static_cast<float>(pkg.torque.pitch_end);
}

/**
 * @brief 更新发送数据包 RobotData (系统层 -> 设备层)
 *
 */
void CSystemControllerLink::UpdateRobotDataPkg_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->robotData_info_pkg;

	pkg.status_flags = {}; // 清空状态标志位

	// 打包状态标志位 (使用RobotData专用定义)
	if (robotInfo.ask_reset_flag) pkg.status_flags.ask_reset = 1;
	if (robotInfo.controlled_by_controller) pkg.status_flags.control_by_controller = 1;
	if (robotInfo.robot_init_ok) pkg.status_flags.robot_init_ok = 1;
	if (robotInfo.p3_lock) pkg.status_flags.p3_lock = 1;

	// 压缩角度 (float -> int16)
	pkg.arm.yaw       = CDevControllerLink::CompressAngle(robotInfo.arm.yaw);
	pkg.arm.pitch1    = CDevControllerLink::CompressAngle(robotInfo.arm.pitch1);
	pkg.arm.pitch2    = CDevControllerLink::CompressAngle(robotInfo.arm.pitch2);
	pkg.arm.pitch3    = CDevControllerLink::CompressAngle(robotInfo.arm.pitch3);
	pkg.arm.roll      = CDevControllerLink::CompressAngle(robotInfo.arm.roll);
	pkg.arm.pitch_end = CDevControllerLink::CompressAngle(robotInfo.arm.pitch_end);

	// 力矩/电流 (float -> int16)
	pkg.torque.yaw       = static_cast<int16_t>(robotInfo.torque.yaw);
	pkg.torque.pitch1    = static_cast<int16_t>(robotInfo.torque.pitch1);
	pkg.torque.pitch2    = static_cast<int16_t>(robotInfo.torque.pitch2);
	pkg.torque.pitch3    = static_cast<int16_t>(robotInfo.torque.pitch3);
	pkg.torque.roll      = static_cast<int16_t>(robotInfo.torque.roll);
	pkg.torque.pitch_end = static_cast<int16_t>(robotInfo.torque.pitch_end);
}

/**
 * @brief 更新发送数据包 ControllerData (系统层 -> 设备层)
 *
 */
void CSystemControllerLink::UpdateControllerDataPkg_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->controllerData_info_pkg;
	
	pkg.status_flags = {};

	if (controllerInfo.controller_OK) pkg.status_flags.controller_init_ok = 1;
	if (controllerInfo.return_success) pkg.status_flags.return_sucess = 1;
	//pkg.status_flags.toggle_switch = static_cast<uint8_t>(controllerInfo.toggle_switch);
	if (controllerInfo.gripper_close) pkg.status_flags.grip = 1;

	// 单臂角度数据 (float直传)
	pkg.arm.yaw       = controllerInfo.arm.yaw;
	pkg.arm.pitch1    = controllerInfo.arm.pitch1;
	pkg.arm.pitch2    = controllerInfo.arm.pitch2;
	pkg.arm.pitch3    = controllerInfo.arm.pitch3;
	pkg.arm.roll      = controllerInfo.arm.roll;
	pkg.arm.pitch_end = controllerInfo.arm.pitch_end;

	// 摇杆数据
	pkg.rocker_X = controllerInfo.rocker_X;
	pkg.rocker_Y = controllerInfo.rocker_Y;
}

/**
 * @brief 心跳处理
 *
 */
void CSystemControllerLink::HeartbeatHandler_() {
	systemStatus = APP_OK;
}

} // namespace my_engineer