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

	// 解析状态标志位 (按位掩码)
	controllerInfo.controller_OK = (pkg.status_flags & STATUS_CONTROLLER_OK) != 0;
	controllerInfo.return_success = (pkg.status_flags & STATUS_RETURN_SUCCESS) != 0;
	// toggle switch 位由 STATUS_TOGGLE_* 掩码提取（若需要可恢复）
	controllerInfo.gripper_close = ((pkg.status_flags & STATUS_GRIPPER_LEFT) != 0) || ((pkg.status_flags & STATUS_GRIPPER_RIGHT) != 0);
	controllerInfo.gripper_regrip = false; // 未定义单独的regrip位，保留为false

	// 单臂角度数据 (解压 compressed -> float)，映射到 left_arm
	controllerInfo.arm.yaw       = CDevControllerLink::DecompressAngle(pkg.left_arm.yaw);
	controllerInfo.arm.pitch1    = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch1);
	controllerInfo.arm.pitch2    = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch2);
	controllerInfo.arm.pitch3    = CDevControllerLink::DecompressAngle(pkg.left_arm.roll);
	controllerInfo.arm.roll      = CDevControllerLink::DecompressAngle(pkg.left_arm.roll);
	controllerInfo.arm.pitch_end = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch_end);

	// 摇杆数据 映射：使用 left rocker X 和 right rocker Y 作为参考
	controllerInfo.rocker_X = pkg.rocker_LX;
	controllerInfo.rocker_Y = pkg.rocker_RY;
}

/**
 * @brief 更新机器人信息 (设备层 -> 系统层)
 *
 */
void CSystemControllerLink::UpdateRobotInfo_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->robotData_info_pkg;

	// 解析状态标志位 (按位掩码)
	robotInfo.ask_reset_flag = (pkg.status_flags & STATUS_ASK_RESET) != 0;
	robotInfo.controlled_by_controller = (pkg.status_flags & STATUS_CONTROLLED) != 0;
	robotInfo.robot_init_ok = (pkg.status_flags & STATUS_ROBOT_INIT_OK) != 0;
	robotInfo.p3_lock = false; // no explicit bit for p3_lock in current protocol

	// 解压角度 (int16 -> float)，映射到 left_arm
	robotInfo.arm.yaw       = CDevControllerLink::DecompressAngle(pkg.left_arm.yaw);
	robotInfo.arm.pitch1    = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch1);
	robotInfo.arm.pitch2    = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch2);
	robotInfo.arm.pitch3    = CDevControllerLink::DecompressAngle(pkg.left_arm.roll);
	robotInfo.arm.roll      = CDevControllerLink::DecompressAngle(pkg.left_arm.roll);
	robotInfo.arm.pitch_end = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch_end);

	// 设备包中没有 torque 字段，清零占位
	robotInfo.torque.yaw = 0.0f;
	robotInfo.torque.pitch1 = 0.0f;
	robotInfo.torque.pitch2 = 0.0f;
	robotInfo.torque.pitch3 = 0.0f;
	robotInfo.torque.roll = 0.0f;
	robotInfo.torque.pitch_end = 0.0f;
}

/**
 * @brief 更新发送数据包 RobotData (系统层 -> 设备层)
 *
 */
void CSystemControllerLink::UpdateRobotDataPkg_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->robotData_info_pkg;

	pkg.status_flags = 0; // 清空状态标志位

	// 打包状态标志位 (按位设置)
	if (robotInfo.ask_reset_flag) pkg.status_flags |= STATUS_ASK_RESET;
	if (robotInfo.controlled_by_controller) pkg.status_flags |= STATUS_CONTROLLED;
	if (robotInfo.robot_init_ok) pkg.status_flags |= STATUS_ROBOT_INIT_OK;

	// 压缩角度 (float -> int16)，放入 left_arm
	pkg.left_arm.yaw       = CDevControllerLink::CompressAngle(robotInfo.arm.yaw);
	pkg.left_arm.pitch1    = CDevControllerLink::CompressAngle(robotInfo.arm.pitch1);
	pkg.left_arm.pitch2    = CDevControllerLink::CompressAngle(robotInfo.arm.pitch2);
	pkg.left_arm.roll      = CDevControllerLink::CompressAngle(robotInfo.arm.roll);
	pkg.left_arm.pitch_end = CDevControllerLink::CompressAngle(robotInfo.arm.pitch_end);

	// 设备包没有 torque 字段，跳过
}

/**
 * @brief 更新发送数据包 ControllerData (系统层 -> 设备层)
 *
 */
void CSystemControllerLink::UpdateControllerDataPkg_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->controllerData_info_pkg;
	
	pkg.status_flags = 0;

	if (controllerInfo.controller_OK) pkg.status_flags |= STATUS_CONTROLLER_OK;
	if (controllerInfo.return_success) pkg.status_flags |= STATUS_RETURN_SUCCESS;
	if (controllerInfo.gripper_close) pkg.status_flags |= STATUS_GRIPPER_LEFT; // 使用左夹爪位表示闭合

	// 单臂角度数据 (float -> compressed)，放入 left_arm
	pkg.left_arm.yaw       = CDevControllerLink::CompressAngle(controllerInfo.arm.yaw);
	pkg.left_arm.pitch1    = CDevControllerLink::CompressAngle(controllerInfo.arm.pitch1);
	pkg.left_arm.pitch2    = CDevControllerLink::CompressAngle(controllerInfo.arm.pitch2);
	pkg.left_arm.roll      = CDevControllerLink::CompressAngle(controllerInfo.arm.roll);
	pkg.left_arm.pitch_end = CDevControllerLink::CompressAngle(controllerInfo.arm.pitch_end);

	// 摇杆数据 映射到可用字段
	pkg.rocker_LX = controllerInfo.rocker_X;
	pkg.rocker_RY = controllerInfo.rocker_Y;
}

/**
 * @brief 心跳处理
 *
 */
void CSystemControllerLink::HeartbeatHandler_() {
	systemStatus = APP_OK;
}

} // namespace my_engineer