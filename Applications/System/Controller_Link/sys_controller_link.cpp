/******************************************************************************
 * @brief        
 * 
 * @file         sys_controller_link.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V2.0
 * @date         2025-04-05
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-17
 *
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "sys_controller_link.hpp"
#include "mod_controller.hpp"  // 用于获取模块层摇杆数据

namespace my_engineer {

#define I_AM_CONTROLLER 1       // 1=控制器端, 0=机器人端
#define DEBUG_SKIP_INIT_CHECK 0 // 1=调试模式(跳过初始化检查), 0=正常模式

// 实例化控制器通信系统
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

	// 获取按键设备指针
	auto it_btn = DeviceIDMap.find(EDeviceID::DEV_MULTI_BUTTON);
	if (it_btn != DeviceIDMap.end() && it_btn->second != nullptr) {
		pbuttons_ = static_cast<CDevFourButton *>(it_btn->second);
	}

	// 注册系统
	RegisterSystem_();

	systemStatus = APP_ERROR;
	return APP_OK;
}

/**
 * @brief 更新处理
 */
void CSystemControllerLink::UpdateHandler_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

#if I_AM_CONTROLLER == 0
	// 机器人端：解析控制器数据，发送机器人数据
	UpdateControllerLinkInfo_();
	UpdateRobotDataPkg_();
	pcontrollerLink_->SendPackage(CDevControllerLink::ID_ROBOT_DATA,
	                               pcontrollerLink_->robotData_info_pkg.header);
#else
	// 控制器端：始终更新输入和接收数据
	UpdateButtonInfo_();
	UpdateRobotInfo_();

	// 只有机器人初始化完成后才发送控制数据
#if DEBUG_SKIP_INIT_CHECK == 0
	// 调试模式：跳过初始化检查，始终发送
	UpdateControllerDataPkg_();
	pcontrollerLink_->SendPackage(CDevControllerLink::ID_CONTROLLER_DATA,
	                               pcontrollerLink_->controllerData_info_pkg.header);
#else
	// 正常模式：等待机器人初始化完成
	if (robotInfo.robot_init_ok) {
		UpdateControllerDataPkg_();
		pcontrollerLink_->SendPackage(CDevControllerLink::ID_CONTROLLER_DATA,
		                               pcontrollerLink_->controllerData_info_pkg.header);
	}
#endif
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
	controllerInfo.controller_OK = (pkg.status_flags & STATUS_CONTROLLER_OK) != 0;
	controllerInfo.return_success = (pkg.status_flags & STATUS_RETURN_SUCCESS) != 0;
	controllerInfo.toggle_switch = static_cast<EToggleSwitch>((pkg.status_flags & STATUS_TOGGLE_MASK) >> STATUS_TOGGLE_SHIFT);
	controllerInfo.gripper_left_close = (pkg.status_flags & STATUS_GRIPPER_LEFT) != 0;
	controllerInfo.gripper_right_close = (pkg.status_flags & STATUS_GRIPPER_RIGHT) != 0;

	// 解压左臂角度数据 (int16 -> float)
	controllerInfo.left_arm.yaw = CDevControllerLink::DecompressAngle(pkg.left_arm.yaw);
	controllerInfo.left_arm.pitch1 = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch1);
	controllerInfo.left_arm.pitch2 = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch2);
	controllerInfo.left_arm.roll = CDevControllerLink::DecompressAngle(pkg.left_arm.roll);
	controllerInfo.left_arm.pitch_end = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch_end);

	// 解压右臂角度数据 (int16 -> float)
	controllerInfo.right_arm.yaw = CDevControllerLink::DecompressAngle(pkg.right_arm.yaw);
	controllerInfo.right_arm.pitch1 = CDevControllerLink::DecompressAngle(pkg.right_arm.pitch1);
	controllerInfo.right_arm.pitch2 = CDevControllerLink::DecompressAngle(pkg.right_arm.pitch2);
	controllerInfo.right_arm.roll = CDevControllerLink::DecompressAngle(pkg.right_arm.roll);
	controllerInfo.right_arm.pitch_end = CDevControllerLink::DecompressAngle(pkg.right_arm.pitch_end);

	// 摇杆数据
	controllerInfo.rocker_LX = pkg.rocker_LX;
	controllerInfo.rocker_RX = pkg.rocker_RX;
	controllerInfo.rocker_RY = pkg.rocker_RY;
}

/**
 * @brief 更新机器人信息 (设备层 -> 系统层)
 */
void CSystemControllerLink::UpdateRobotInfo_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->robotData_info_pkg;

	// 解析状态标志位 (使用RobotData专用定义)
	robotInfo.ask_reset_flag = (pkg.status_flags & STATUS_ASK_RESET) != 0;
	robotInfo.controlled_by_controller = (pkg.status_flags & STATUS_CONTROLLED) != 0;
	robotInfo.robot_init_ok = (pkg.status_flags & STATUS_ROBOT_INIT_OK) != 0;

	// 解压左臂角度数据
	robotInfo.left_arm.yaw = CDevControllerLink::DecompressAngle(pkg.left_arm.yaw);
	robotInfo.left_arm.pitch1 = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch1);
	robotInfo.left_arm.pitch2 = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch2);
	robotInfo.left_arm.roll = CDevControllerLink::DecompressAngle(pkg.left_arm.roll);
	robotInfo.left_arm.pitch_end = CDevControllerLink::DecompressAngle(pkg.left_arm.pitch_end);

	// 解压右臂角度数据
	robotInfo.right_arm.yaw = CDevControllerLink::DecompressAngle(pkg.right_arm.yaw);
	robotInfo.right_arm.pitch1 = CDevControllerLink::DecompressAngle(pkg.right_arm.pitch1);
	robotInfo.right_arm.pitch2 = CDevControllerLink::DecompressAngle(pkg.right_arm.pitch2);
	robotInfo.right_arm.roll = CDevControllerLink::DecompressAngle(pkg.right_arm.roll);
	robotInfo.right_arm.pitch_end = CDevControllerLink::DecompressAngle(pkg.right_arm.pitch_end);
}

/**
 * @brief 更新发送数据包 RobotData (系统层 -> 设备层)
 */
void CSystemControllerLink::UpdateRobotDataPkg_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->robotData_info_pkg;

	// 打包状态标志位 (使用RobotData专用定义)
	pkg.status_flags = 0;
	if (robotInfo.ask_reset_flag) pkg.status_flags |= STATUS_ASK_RESET;
	if (robotInfo.controlled_by_controller) pkg.status_flags |= STATUS_CONTROLLED;
	if (robotInfo.robot_init_ok) pkg.status_flags |= STATUS_ROBOT_INIT_OK;

	// 压缩左臂角度数据
	pkg.left_arm.yaw = CDevControllerLink::CompressAngle(robotInfo.left_arm.yaw);
	pkg.left_arm.pitch1 = CDevControllerLink::CompressAngle(robotInfo.left_arm.pitch1);
	pkg.left_arm.pitch2 = CDevControllerLink::CompressAngle(robotInfo.left_arm.pitch2);
	pkg.left_arm.roll = CDevControllerLink::CompressAngle(robotInfo.left_arm.roll);
	pkg.left_arm.pitch_end = CDevControllerLink::CompressAngle(robotInfo.left_arm.pitch_end);

	// 压缩右臂角度数据
	pkg.right_arm.yaw = CDevControllerLink::CompressAngle(robotInfo.right_arm.yaw);
	pkg.right_arm.pitch1 = CDevControllerLink::CompressAngle(robotInfo.right_arm.pitch1);
	pkg.right_arm.pitch2 = CDevControllerLink::CompressAngle(robotInfo.right_arm.pitch2);
	pkg.right_arm.roll = CDevControllerLink::CompressAngle(robotInfo.right_arm.roll);
	pkg.right_arm.pitch_end = CDevControllerLink::CompressAngle(robotInfo.right_arm.pitch_end);
}

/**
 * @brief 更新发送数据包 ControllerData (系统层 -> 设备层)
 */
void CSystemControllerLink::UpdateControllerDataPkg_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->controllerData_info_pkg;

	// 打包状态标志位
	pkg.status_flags = 0;
	if (controllerInfo.controller_OK) pkg.status_flags |= STATUS_CONTROLLER_OK;
	if (controllerInfo.return_success) pkg.status_flags |= STATUS_RETURN_SUCCESS;
	pkg.status_flags |= (static_cast<uint8_t>(controllerInfo.toggle_switch) << STATUS_TOGGLE_SHIFT) & STATUS_TOGGLE_MASK;
	if (controllerInfo.gripper_left_close) pkg.status_flags |= STATUS_GRIPPER_LEFT;
	if (controllerInfo.gripper_right_close) pkg.status_flags |= STATUS_GRIPPER_RIGHT;

	// 压缩左臂角度数据 (float -> int16)
	pkg.left_arm.yaw = CDevControllerLink::CompressAngle(controllerInfo.left_arm.yaw);
	pkg.left_arm.pitch1 = CDevControllerLink::CompressAngle(controllerInfo.left_arm.pitch1);
	pkg.left_arm.pitch2 = CDevControllerLink::CompressAngle(controllerInfo.left_arm.pitch2);
	pkg.left_arm.roll = CDevControllerLink::CompressAngle(controllerInfo.left_arm.roll);
	pkg.left_arm.pitch_end = CDevControllerLink::CompressAngle(controllerInfo.left_arm.pitch_end);

	// 压缩右臂角度数据 (float -> int16)
	pkg.right_arm.yaw = CDevControllerLink::CompressAngle(controllerInfo.right_arm.yaw);
	pkg.right_arm.pitch1 = CDevControllerLink::CompressAngle(controllerInfo.right_arm.pitch1);
	pkg.right_arm.pitch2 = CDevControllerLink::CompressAngle(controllerInfo.right_arm.pitch2);
	pkg.right_arm.roll = CDevControllerLink::CompressAngle(controllerInfo.right_arm.roll);
	pkg.right_arm.pitch_end = CDevControllerLink::CompressAngle(controllerInfo.right_arm.pitch_end);

	// 摇杆数据
	pkg.rocker_LX = controllerInfo.rocker_LX;
	pkg.rocker_RX = controllerInfo.rocker_RX;
	pkg.rocker_RY = controllerInfo.rocker_RY;
}

/**
 * @brief 心跳处理
 *
 */
void CSystemControllerLink::HeartbeatHandler_() {
	systemStatus = APP_OK;
}

/**
 * @brief 更新按键信息 (拨杆、夹爪、摇杆)
 */
void CSystemControllerLink::UpdateButtonInfo_() {
	if (systemStatus != APP_OK) return;

	// 从模块层获取摇杆数据（模块层已转换为 -100~100 范围，直接赋值即可）
	auto it_left = ModuleIDMap.find(EModuleID::MOD_CONTROLLER_LEFT);
	if (it_left != ModuleIDMap.end() && it_left->second != nullptr) {
		auto *pController = static_cast<CModController*>(it_left->second);
		// 左臂摇杆X轴
		controllerInfo.rocker_LX = pController->ControllerInfo.rocker_X;
	}

	// 右臂 roll_end 摇杆
	auto it_right = ModuleIDMap.find(EModuleID::MOD_CONTROLLER_RIGHT);
	if (it_right != ModuleIDMap.end() && it_right->second != nullptr) {
		auto *pController = static_cast<CModController*>(it_right->second);
		// 右臂摇杆X轴
		controllerInfo.rocker_RX = pController->ControllerInfo.rocker_X;
		// 右臂摇杆Y轴
		controllerInfo.rocker_RY = pController->ControllerInfo.rocker_Y;
	}

	// 3档拨杆状态 (0=中档, 1=臂Roll末端模式, 2=底盘模式)
	if (CDevFourButton::isSwitchArmRollEnd) {
		controllerInfo.toggle_switch = EToggleSwitch::TOGGLE_ARM_ROLL;  // 左档 臂Roll末端模式
	} else if (CDevFourButton::isSwitchChassis) {
		controllerInfo.toggle_switch = EToggleSwitch::TOGGLE_CHASSIS;  // 右档 底盘模式
	} else {
		controllerInfo.toggle_switch = EToggleSwitch::TOGGLE_MIDDLE;  // 中档
	}

	// 双夹爪按钮状态
	controllerInfo.gripper_left_close = CDevFourButton::isGripperLeftClose;
	controllerInfo.gripper_right_close = CDevFourButton::isGripperRightClose;
}

} // namespace my_engineer
