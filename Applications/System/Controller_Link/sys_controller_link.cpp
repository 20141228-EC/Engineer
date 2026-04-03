/******************************************************************************
 * @brief   控制器通信系统 —— 负责控制器与机器人之间的数据收发
 *
 * @file         sys_controller_link.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V3.0
 * @date         2025-04-05
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-03-19
 *
 * @copyright    Copyright (c) 2025
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
 * @brief 通信主循环（由系统层周期调用）
 *
 * 控制器端执行流程：
 *   1. UpdateButtonInfo_()          采集本地输入（摇杆/按钮） 写入 controllerInfo
 *   2. UpdateRobotInfo_()           解析机器人发来的数据包    写入 robotInfo
 *   3. UpdateControllerDataPkg_()   将 controllerInfo 打包   通过 UART 发送给机器人
 */
void CSystemControllerLink::UpdateHandler_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

#if I_AM_CONTROLLER == 0
	UpdateControllerLinkInfo_();
	UpdateRobotDataPkg_();
	pcontrollerLink_->SendPackage(CDevControllerLink::ID_ROBOT_DATA,
	                               pcontrollerLink_->robotData_info_pkg.header);
#else
	/*--  控制器端 --*/
	//读取摇杆/按钮硬件输入 -> controllerInfo
	UpdateButtonInfo_();
	//解析机器人发来的 RobotDataPkg ->  robotInfo
	UpdateRobotInfo_();

	//将 controllerInfo 打包为 ControllerDataPkg ->  发送给机器人
#if DEBUG_SKIP_INIT_CHECK == 0
	// 调试模式：跳过初始化检查，始终发送
	UpdateControllerDataPkg_();
	pcontrollerLink_->SendPackage(CDevControllerLink::ID_CONTROLLER_DATA,
	                               pcontrollerLink_->controllerData_info_pkg.header);
#else
	// 正常模式：等待机器人初始化完成后才发送
	if (robotInfo.robot_init_ok) {
		UpdateControllerDataPkg_();
		pcontrollerLink_->SendPackage(CDevControllerLink::ID_CONTROLLER_DATA,
		                               pcontrollerLink_->controllerData_info_pkg.header);
	}
#endif
#endif
}

/*=========================== 如果当前的设备是控制器 (I_AM_CONTROLLER=1) ===========================*/

/**
 * @brief 控制器端读取本地硬件写入 controllerInfo
 */
void CSystemControllerLink::UpdateButtonInfo_() {
	if (systemStatus != APP_OK) return;

	// 从模块层获取摇杆数据（模块层已转换为 -100~100 范围，直接赋值即可）
	auto it = ModuleIDMap.find(EModuleID::MOD_CONTROLLER);
	if (it != ModuleIDMap.end() && it->second != nullptr) {
		auto *pController = static_cast<CModController*>(it->second);
		controllerInfo.rocker_X = pController->ControllerInfo.rocker_X;
		controllerInfo.rocker_Y = pController->ControllerInfo.rocker_Y;
	}

	// 3档拨杆状态 (0=中档, 1=臂Roll末端模式, 2=底盘模式)
	if (CDevFourButton::isSwitchArmRollEnd) {
		controllerInfo.toggle_switch = EToggleSwitch::TOGGLE_ARM_ROLL;  // 左档 臂Roll末端模式
	} else if (CDevFourButton::isSwitchChassis) {
		controllerInfo.toggle_switch = EToggleSwitch::TOGGLE_CHASSIS;  // 右档 底盘模式
	} else {
		controllerInfo.toggle_switch = EToggleSwitch::TOGGLE_MIDDLE;  // 中档
	}

	// 夹爪按钮状态（单夹爪，PB9）
	controllerInfo.gripper_close = CDevFourButton::isGripperClose;

	// 二次夹紧（脉冲信号：读取后立即清除源标志）
	if (CDevFourButton::isGripperReGrip) {
		controllerInfo.gripper_regrip = true;
		CDevFourButton::isGripperReGrip = false;
	}
}

/**
 * @brief 控制器端发：controllerInfo -> 打包 -> 发给机器人
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
	if (controllerInfo.gripper_close) pkg.status_flags |= STATUS_GRIPPER;
	// 二次夹紧：脉冲信号，打包后自动清除
	if (controllerInfo.gripper_regrip) {
		pkg.status_flags |= STATUS_REGRIP;
		controllerInfo.gripper_regrip = false;
	}

	// 单臂角度数据 (float直传)
	pkg.arm.yaw       = controllerInfo.arm.yaw;
	pkg.arm.pitch1    = controllerInfo.arm.pitch1;
	pkg.arm.pitch2    = controllerInfo.arm.pitch2;
	pkg.arm.roll      = controllerInfo.arm.roll;
	pkg.arm.pitch_end = controllerInfo.arm.pitch_end;

	// 摇杆数据
	pkg.rocker_X = controllerInfo.rocker_X;
	pkg.rocker_Y = controllerInfo.rocker_Y;
}

/**
 * @brief 控制器端收：机器人发来的包 -> 解析 -> robotInfo
 */
void CSystemControllerLink::UpdateRobotInfo_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->robotData_info_pkg;

	// 解析状态标志位 (使用RobotData专用定义)
	robotInfo.ask_reset_flag = (pkg.status_flags & STATUS_ASK_RESET) != 0;
	robotInfo.controlled_by_controller = (pkg.status_flags & STATUS_CONTROLLED) != 0;
	robotInfo.robot_init_ok = (pkg.status_flags & STATUS_ROBOT_INIT_OK) != 0;

	// 解压角度 (int16 -> float)
	robotInfo.arm.yaw       = CDevControllerLink::DecompressAngle(pkg.arm.yaw);
	robotInfo.arm.pitch1    = CDevControllerLink::DecompressAngle(pkg.arm.pitch1);
	robotInfo.arm.pitch2    = CDevControllerLink::DecompressAngle(pkg.arm.pitch2);
	robotInfo.arm.roll      = CDevControllerLink::DecompressAngle(pkg.arm.roll);
	robotInfo.arm.pitch_end = CDevControllerLink::DecompressAngle(pkg.arm.pitch_end);

	// 力矩/电流反馈 (int16 -> float，保留原始值)
	robotInfo.torque.yaw       = static_cast<float>(pkg.torque.yaw);
	robotInfo.torque.pitch1    = static_cast<float>(pkg.torque.pitch1);
	robotInfo.torque.pitch2    = static_cast<float>(pkg.torque.pitch2);
	robotInfo.torque.roll      = static_cast<float>(pkg.torque.roll);
	robotInfo.torque.pitch_end = static_cast<float>(pkg.torque.pitch_end);
}

/*============================= 如果当前的设备是机器人 (I_AM_CONTROLLER=0) =============================*/

/**
 * @brief 机器人端收：控制器发来的包 -> 解析 -> controllerInfo
 */
void CSystemControllerLink::UpdateControllerLinkInfo_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->controllerData_info_pkg;

	// 解析状态标志位
	controllerInfo.controller_OK = (pkg.status_flags & STATUS_CONTROLLER_OK) != 0;
	controllerInfo.return_success = (pkg.status_flags & STATUS_RETURN_SUCCESS) != 0;
	controllerInfo.toggle_switch = static_cast<EToggleSwitch>((pkg.status_flags & STATUS_TOGGLE_MASK) >> STATUS_TOGGLE_SHIFT);
	controllerInfo.gripper_close = (pkg.status_flags & STATUS_GRIPPER) != 0;

	// 单臂角度数据 (float直传)
	controllerInfo.arm.yaw       = pkg.arm.yaw;
	controllerInfo.arm.pitch1    = pkg.arm.pitch1;
	controllerInfo.arm.pitch2    = pkg.arm.pitch2;
	controllerInfo.arm.roll      = pkg.arm.roll;
	controllerInfo.arm.pitch_end = pkg.arm.pitch_end;

	// 摇杆数据
	controllerInfo.rocker_X = pkg.rocker_X;
	controllerInfo.rocker_Y = pkg.rocker_Y;
}

/**
 * @brief 机器人端发：robotInfo -> 打包 -> 发给控制器
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

	// 压缩角度 (float -> int16)
	pkg.arm.yaw       = CDevControllerLink::CompressAngle(robotInfo.arm.yaw);
	pkg.arm.pitch1    = CDevControllerLink::CompressAngle(robotInfo.arm.pitch1);
	pkg.arm.pitch2    = CDevControllerLink::CompressAngle(robotInfo.arm.pitch2);
	pkg.arm.roll      = CDevControllerLink::CompressAngle(robotInfo.arm.roll);
	pkg.arm.pitch_end = CDevControllerLink::CompressAngle(robotInfo.arm.pitch_end);

	// 力矩/电流 (float -> int16)
	pkg.torque.yaw       = static_cast<int16_t>(robotInfo.torque.yaw);
	pkg.torque.pitch1    = static_cast<int16_t>(robotInfo.torque.pitch1);
	pkg.torque.pitch2    = static_cast<int16_t>(robotInfo.torque.pitch2);
	pkg.torque.roll      = static_cast<int16_t>(robotInfo.torque.roll);
	pkg.torque.pitch_end = static_cast<int16_t>(robotInfo.torque.pitch_end);
}

/*==========================================================================================*/

/**
 * @brief 心跳处理
 */
void CSystemControllerLink::HeartbeatHandler_() {
	systemStatus = APP_OK;
}

} // namespace my_engineer
