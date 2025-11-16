/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-07-16 17:10:02
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-18 01:06:15
 */
/******************************************************************************
 * @brief        
 * 
 * @file         sys_esp32.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-07-12
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "sys_esp32.hpp"

namespace my_engineer {

CSystemESP32 SysESP32; ///< ESP32系统实例

EAppStatus CSystemESP32::InitSystem(SSystemInitParam_Base *pStruct) {
	if (pStruct == nullptr) return APP_ERROR;
	if (pStruct->systemID == ESystemID::SYS_NULL) return APP_ERROR;

	auto *esp32Param = static_cast<SSystemInitParam_ESP32 *>(pStruct);

	systemID = esp32Param->systemID;
	pESP32_ = static_cast<CDevESP32 *>(DeviceIDMap.at(esp32Param->esp32DevID));

	RegisterSystem_();

	systemStatus = APP_OK;

	return APP_OK;
}

void CSystemESP32::UpdateHandler_() {
	if (systemStatus == APP_RESET) return;

	static uint8_t tick = 20; // 更新间隔为10ms
	if (tick-- != 0) return;
	tick = 20; // 重置计数器

	auto arm = reinterpret_cast<CModArm *>(ModuleIDMap.at(EModuleID::MOD_ARM));

	if (pESP32_->armInfo.BlueTooth_Mode != 0) {
		BLE_Mode_Open = true;
		BLEInfo.Yaw = static_cast<float_t>(pESP32_->armInfo.Yaw) / 1000.0f;
		BLEInfo.Pitch1 = static_cast<float_t>(pESP32_->armInfo.Pitch1) / 1000.0f;
		BLEInfo.Pitch2 = static_cast<float_t>(pESP32_->armInfo.Pitch2) / 1000.0f;
		BLEInfo.Roll = static_cast<float_t>(pESP32_->armInfo.Roll) / 1000.0f;
		BLEInfo.End_Pitch = static_cast<float_t>(pESP32_->armInfo.End_Pitch) / 1000.0f;
		BLEInfo.End_Roll = static_cast<float_t>(pESP32_->armInfo.End_Roll) / 1000.0f;
	} else {
		BLE_Mode_Open = false;
		pESP32_->armInfo.Yaw = static_cast<int>(arm->armInfo.angle_Yaw * 1000);
		pESP32_->armInfo.Pitch1 = static_cast<int>(arm->armInfo.angle_Pitch1 * 1000);
		pESP32_->armInfo.Pitch2 = static_cast<int>(arm->armInfo.angle_Pitch2 * 1000);
		pESP32_->armInfo.Roll = static_cast<int>(arm->armInfo.angle_Roll * 1000);
		pESP32_->armInfo.End_Pitch = static_cast<int>(arm->armInfo.angle_end_pitch * 1000);
		pESP32_->armInfo.End_Roll = static_cast<int>(arm->armInfo.angle_end_roll * 1000);
		pESP32_->SendPackage(CDevESP32::EPackageID::ID_Motor_Arm_Pkg);
	}

	pESP32_->robotInfo.Chassis_LF = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_CHAS_MTR_LF))->IsMotorOnline();
	pESP32_->robotInfo.Chassis_RF = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_CHAS_MTR_RF))->IsMotorOnline();
	pESP32_->robotInfo.Chassis_LB = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_CHAS_MTR_LB))->IsMotorOnline();
	pESP32_->robotInfo.Chassis_RB = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_CHAS_MTR_RB))->IsMotorOnline();
	pESP32_->robotInfo.Gimbal = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_GIMBAL_MTR))->IsMotorOnline();
	pESP32_->robotInfo.SubGantry_Lift_L = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_SUBGANTRY_MTR_LIFT_L))->IsMotorOnline();
	pESP32_->robotInfo.SubGantry_Lift_R = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_SUBGANTRY_MTR_LIFT_R))->IsMotorOnline();
	pESP32_->robotInfo.SubGantry_Stretch_L = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_SUBGANTRY_MTR_STRETCH_L))->IsMotorOnline();
	pESP32_->robotInfo.SubGantry_Stretch_R = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_SUBGANTRY_MTR_STRETCH_R))->IsMotorOnline();
	pESP32_->robotInfo.Arm_Yaw = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_ARM_MTR_YAW))->IsMotorOnline();
	pESP32_->robotInfo.Arm_Pitch1 = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_ARM_MTR_PITCH1))->IsMotorOnline();
	pESP32_->robotInfo.Arm_Pitch2 = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_ARM_MTR_PITCH2))->IsMotorOnline();
	pESP32_->robotInfo.Arm_Roll = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_ARM_MTR_ROLL))->IsMotorOnline();
	pESP32_->robotInfo.Arm_End_L = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_ARM_MTR_END_L))->IsMotorOnline();
	pESP32_->robotInfo.Arm_End_R = reinterpret_cast<CDevMtr *>(DeviceIDMap.at(EDeviceID::DEV_ARM_MTR_END_R))->IsMotorOnline();
	pESP32_->SendPackage(CDevESP32::EPackageID::ID_Motor_Info_Pkg);

}

void CSystemESP32::HeartbeatHandler_() {

}


} // namespace my_engineer
