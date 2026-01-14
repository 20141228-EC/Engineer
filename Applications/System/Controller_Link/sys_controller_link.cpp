/******************************************************************************
 * @brief        
 * 
 * @file         sys_controller_link.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-04-05
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "sys_controller_link.hpp"

namespace my_engineer {

#define I_AM_CONTROLLER 1 // 当前板子是控制器

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
	pcontrollerLink_ = static_cast<CDevControllerLink *>(DeviceIDMap.at(param.controllerLinkDevID));
	pbuttons_ = static_cast<CDevFourButton *>(DeviceIDMap.at(EDeviceID::DEV_MULTI_BUTTON));

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

	UpdateButtonInfo_();

	#if I_AM_CONTROLLER == 0
		// 更新控制器信息
		UpdateControllerLinkInfo_();
		// 更新发送数据包
		UpdateRobotDataPkg_();
		// 发送机器人信息
		pcontrollerLink_->SendPackage(CDevControllerLink::ID_ROBOT_DATA, pcontrollerLink_->robotData_info_pkg.header);
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
 * @brief 更新控制器信息
 *
 */
void CSystemControllerLink::UpdateControllerLinkInfo_() {///<设备层-->系统层
	if (systemStatus != APP_OK) return;

	// 更新控制器信息
	controllerInfo.controller_OK = pcontrollerLink_->controllerData_info_pkg.controller_OK;
	controllerInfo.return_success = pcontrollerLink_->controllerData_info_pkg.return_success;

	// 解压左臂角度数据 (5轴)
	auto &left_compressed = pcontrollerLink_->controllerData_info_pkg.left_arm;
	controllerInfo.left_arm.yaw = DecompressAngle_(left_compressed.yaw);
	controllerInfo.left_arm.pitch1 = DecompressAngle_(left_compressed.pitch1);
	controllerInfo.left_arm.pitch2 = DecompressAngle_(left_compressed.pitch2);
	controllerInfo.left_arm.roll = DecompressAngle_(left_compressed.roll);
	controllerInfo.left_arm.pitch_end = DecompressAngle_(left_compressed.pitch_end);

	// 解压右臂角度数据 (5轴)
	auto &right_compressed = pcontrollerLink_->controllerData_info_pkg.right_arm;
	controllerInfo.right_arm.yaw = DecompressAngle_(right_compressed.yaw);
	controllerInfo.right_arm.pitch1 = DecompressAngle_(right_compressed.pitch1);
	controllerInfo.right_arm.pitch2 = DecompressAngle_(right_compressed.pitch2);
	controllerInfo.right_arm.roll = DecompressAngle_(right_compressed.roll);
	controllerInfo.right_arm.pitch_end = DecompressAngle_(right_compressed.pitch_end);

	// 更新摇杆/拨杆/按钮数据
	controllerInfo.Rocker_X = pcontrollerLink_->controllerData_info_pkg.rocker_X;
	controllerInfo.Rocker_Y = pcontrollerLink_->controllerData_info_pkg.rocker_Y;
	controllerInfo.Rocker_Key = static_cast<KEY_STATUS>(pcontrollerLink_->controllerData_info_pkg.rocker_Key);
	controllerInfo.toggle_switch = pcontrollerLink_->controllerData_info_pkg.toggle_switch;
	controllerInfo.button = pcontrollerLink_->controllerData_info_pkg.button;
}

/**
 * @brief 更新机器人信息
 *
 */
void CSystemControllerLink::UpdateRobotInfo_() {///<设备层-->系统层
	if (systemStatus != APP_OK) return;

	// 更新机器人信息
	robotInfo.ask_reset_flag = pcontrollerLink_->robotData_info_pkg.ask_reset_flag;
	robotInfo.controlled_by_controller = pcontrollerLink_->robotData_info_pkg.controlled_by_controller;
	robotInfo.ask_return_flag = pcontrollerLink_->robotData_info_pkg.ask_return_flag;

	// 解压左臂角度数据 (5轴)
	auto &left_compressed = pcontrollerLink_->robotData_info_pkg.left_arm;
	robotInfo.left_arm.yaw = DecompressAngle_(left_compressed.yaw);
	robotInfo.left_arm.pitch1 = DecompressAngle_(left_compressed.pitch1);
	robotInfo.left_arm.pitch2 = DecompressAngle_(left_compressed.pitch2);
	robotInfo.left_arm.roll = DecompressAngle_(left_compressed.roll);
	robotInfo.left_arm.pitch_end = DecompressAngle_(left_compressed.pitch_end);

	// 解压右臂角度数据 (5轴)
	auto &right_compressed = pcontrollerLink_->robotData_info_pkg.right_arm;
	robotInfo.right_arm.yaw = DecompressAngle_(right_compressed.yaw);
	robotInfo.right_arm.pitch1 = DecompressAngle_(right_compressed.pitch1);
	robotInfo.right_arm.pitch2 = DecompressAngle_(right_compressed.pitch2);
	robotInfo.right_arm.roll = DecompressAngle_(right_compressed.roll);
	robotInfo.right_arm.pitch_end = DecompressAngle_(right_compressed.pitch_end);
}

/**
 * @brief 更新发送数据包 RobotData
 */
void CSystemControllerLink::UpdateRobotDataPkg_() {///<系统层-->设备层
	if (systemStatus != APP_OK) return;

	// 更新发送包的信息
	pcontrollerLink_->robotData_info_pkg.ask_reset_flag = robotInfo.ask_reset_flag;
	pcontrollerLink_->robotData_info_pkg.controlled_by_controller = robotInfo.controlled_by_controller;
	pcontrollerLink_->robotData_info_pkg.ask_return_flag = robotInfo.ask_return_flag;

	// 压缩左臂角度数据 (5轴)
	pcontrollerLink_->robotData_info_pkg.left_arm.yaw = CompressAngle_(robotInfo.left_arm.yaw);
	pcontrollerLink_->robotData_info_pkg.left_arm.pitch1 = CompressAngle_(robotInfo.left_arm.pitch1);
	pcontrollerLink_->robotData_info_pkg.left_arm.pitch2 = CompressAngle_(robotInfo.left_arm.pitch2);
	pcontrollerLink_->robotData_info_pkg.left_arm.roll = CompressAngle_(robotInfo.left_arm.roll);
	pcontrollerLink_->robotData_info_pkg.left_arm.pitch_end = CompressAngle_(robotInfo.left_arm.pitch_end);

	// 压缩右臂角度数据 (5轴)
	pcontrollerLink_->robotData_info_pkg.right_arm.yaw = CompressAngle_(robotInfo.right_arm.yaw);
	pcontrollerLink_->robotData_info_pkg.right_arm.pitch1 = CompressAngle_(robotInfo.right_arm.pitch1);
	pcontrollerLink_->robotData_info_pkg.right_arm.pitch2 = CompressAngle_(robotInfo.right_arm.pitch2);
	pcontrollerLink_->robotData_info_pkg.right_arm.roll = CompressAngle_(robotInfo.right_arm.roll);
	pcontrollerLink_->robotData_info_pkg.right_arm.pitch_end = CompressAngle_(robotInfo.right_arm.pitch_end);
}

/**
 * @brief 更新发送数据包 ControllerData
 */
void CSystemControllerLink::UpdateControllerDataPkg_() {///<系统层-->设备层
	if (systemStatus != APP_OK) return;

	// 更新发送包的信息
	pcontrollerLink_->controllerData_info_pkg.controller_OK = controllerInfo.controller_OK;
	pcontrollerLink_->controllerData_info_pkg.return_success = controllerInfo.return_success;

	// 压缩左臂角度数据 (5轴)
	pcontrollerLink_->controllerData_info_pkg.left_arm.yaw = CompressAngle_(controllerInfo.left_arm.yaw);
	pcontrollerLink_->controllerData_info_pkg.left_arm.pitch1 = CompressAngle_(controllerInfo.left_arm.pitch1);
	pcontrollerLink_->controllerData_info_pkg.left_arm.pitch2 = CompressAngle_(controllerInfo.left_arm.pitch2);
	pcontrollerLink_->controllerData_info_pkg.left_arm.roll = CompressAngle_(controllerInfo.left_arm.roll);
	pcontrollerLink_->controllerData_info_pkg.left_arm.pitch_end = CompressAngle_(controllerInfo.left_arm.pitch_end);

	// 压缩右臂角度数据 (5轴)
	pcontrollerLink_->controllerData_info_pkg.right_arm.yaw = CompressAngle_(controllerInfo.right_arm.yaw);
	pcontrollerLink_->controllerData_info_pkg.right_arm.pitch1 = CompressAngle_(controllerInfo.right_arm.pitch1);
	pcontrollerLink_->controllerData_info_pkg.right_arm.pitch2 = CompressAngle_(controllerInfo.right_arm.pitch2);
	pcontrollerLink_->controllerData_info_pkg.right_arm.roll = CompressAngle_(controllerInfo.right_arm.roll);
	pcontrollerLink_->controllerData_info_pkg.right_arm.pitch_end = CompressAngle_(controllerInfo.right_arm.pitch_end);

	// 更新摇杆/拨杆/按钮数据
	pcontrollerLink_->controllerData_info_pkg.rocker_X = controllerInfo.Rocker_X;
	pcontrollerLink_->controllerData_info_pkg.rocker_Y = controllerInfo.Rocker_Y;
	pcontrollerLink_->controllerData_info_pkg.rocker_Key = static_cast<uint8_t>(controllerInfo.Rocker_Key);
	pcontrollerLink_->controllerData_info_pkg.toggle_switch = controllerInfo.toggle_switch;
	pcontrollerLink_->controllerData_info_pkg.button = controllerInfo.button;
}
/**
 * @brief 心跳处理
 * 
 */
void CSystemControllerLink::HeartbeatHandler_() {
	systemStatus = APP_OK;
}

void CSystemControllerLink::UpdateButtonInfo_() {
	if (systemStatus != APP_OK) return;
	controllerInfo.isReset = pbuttons_->isReset;
	controllerInfo.isLevel4 = pbuttons_->isLevel4;
	controllerInfo.isLevel3 = pbuttons_->isLevel3;
	controllerInfo.isSelf = pbuttons_->isSelf;

}



// void CDevFourButton::HandleButtonPressDown(uint8_t buttonId){
//   if (buttonId < static_cast<uint8_t>(EButtonID::BUTTON_MAX)) {
// 		switch (buttonId)
// 		{
// 		case constant expression:
// 			/* code */
// 			break;
		
// 		default:
// 			break;
// 		}
//   }

// 	// SysControllerLink.pcontrollerLink_->choseLevelData_info_pkg.Key_value1 = ;
// }

} // namespace my_engineer
