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
			// 发送：独立限频 10Hz
		// 系统任务1kHz
		UpdateRobotDataPkg_();

		if(send_flag){
				UpdateRequestInfo_();
				pcontrollerLink_->SendPackage(CDevControllerLink::ID_REQUEST_DATA, pcontrollerLink_->request_info_pkg.header);
				send_flag = 0;
			}

		if (((pcontrollerLink_->controllerData_info_pkg.idx) + 1) % 5 == 0) {	// 每收到5个点就发一次
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

	// 单臂角度数据 (float直传)
	controllerInfo.arm.yaw       = pkg.arm.yaw;
	controllerInfo.arm.pitch1    = pkg.arm.pitch1;
	controllerInfo.arm.pitch2    = pkg.arm.pitch2;
	controllerInfo.arm.pitch3	= pkg.arm.pitch3;
	controllerInfo.arm.roll      = pkg.arm.roll;
	controllerInfo.arm.pitch_end = pkg.arm.pitch_end;
	controllerInfo.arm.roll_end  = pkg.arm.roll_end;
}

/**
 * @brief 更新机器人信息 (设备层 -> 系统层)
 *
 */
void CSystemControllerLink::UpdateRobotInfo_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->robotData_info_pkg;

	// 解压角度 (int16 -> float)
	robotInfo.arm.yaw       = CDevControllerLink::DecompressAngle(pkg.arm.yaw);
	robotInfo.arm.pitch1    = CDevControllerLink::DecompressAngle(pkg.arm.pitch1);
	robotInfo.arm.pitch2    = CDevControllerLink::DecompressAngle(pkg.arm.pitch2);
	robotInfo.arm.pitch3    = CDevControllerLink::DecompressAngle(pkg.arm.pitch3);
	robotInfo.arm.roll      = CDevControllerLink::DecompressAngle(pkg.arm.roll);
	robotInfo.arm.pitch_end = CDevControllerLink::DecompressAngle(pkg.arm.pitch_end);
	robotInfo.arm.roll_end = CDevControllerLink::DecompressAngle(pkg.arm.roll_end);
}

void CSystemControllerLink::UpdateRequestInfo_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->request_info_pkg;

	pkg.arm.yaw       = requestInfo.arm.yaw;
	pkg.arm.pitch1    = requestInfo.arm.pitch1;
	pkg.arm.pitch2    = requestInfo.arm.pitch2;
	pkg.arm.pitch3    = requestInfo.arm.pitch3;
	pkg.arm.roll      = requestInfo.arm.roll;
	pkg.arm.pitch_end = requestInfo.arm.pitch_end;
	pkg.arm.roll_end =  requestInfo.arm.roll_end;
}

/**
 * @brief 更新发送数据包 RobotData (系统层 -> 设备层)
 *
 */
void CSystemControllerLink::UpdateRobotDataPkg_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->robotData_info_pkg;

	// 压缩角度 (float -> int16)
	pkg.arm.yaw       = robotInfo.arm.yaw;
	pkg.arm.pitch1    = 0.2f;
	pkg.arm.pitch2    = 0.1f;
	pkg.arm.pitch3    = 0.f;
	pkg.arm.roll      = robotInfo.arm.roll;
	pkg.arm.pitch_end = robotInfo.arm.pitch_end;
	pkg.arm.roll_end =  robotInfo.arm.roll_end;

}

/**
 * @brief 更新发送数据包 ControllerData (系统层 -> 设备层)
 *
 */
void CSystemControllerLink::UpdateControllerDataPkg_() {
	if (systemStatus != APP_OK) return;
	if (!pcontrollerLink_) return;

	auto &pkg = pcontrollerLink_->controllerData_info_pkg;

	// 单臂角度数据 (float直传)
	pkg.arm.yaw       = controllerInfo.arm.yaw;
	pkg.arm.pitch1    = controllerInfo.arm.pitch1;
	pkg.arm.pitch2    = controllerInfo.arm.pitch2;
	pkg.arm.pitch3    = controllerInfo.arm.pitch3;
	pkg.arm.roll      = controllerInfo.arm.roll;
	pkg.arm.pitch_end = controllerInfo.arm.pitch_end;
	pkg.arm.roll_end = controllerInfo.arm.roll_end;
}

/**
 * @brief 心跳处理
 *
 */
void CSystemControllerLink::HeartbeatHandler_() {
	systemStatus = APP_OK;
}

} // namespace my_engineer
