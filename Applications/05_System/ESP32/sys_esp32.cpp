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
	auto it = DeviceIDMap.find(esp32Param->esp32DevID);
	if (it != DeviceIDMap.end() && it->second != nullptr) {
		pESP32_ = static_cast<CDevESP32 *>(it->second);
	}

	RegisterSystem_();

	systemStatus = APP_OK;

	return APP_OK;
}

void CSystemESP32::UpdateHandler_() {
	if (systemStatus == APP_RESET) return;

	static uint8_t tick = 20; // 更新间隔为10ms
	if (tick-- != 0) return;
	tick = 20; // 重置计数器

	CModArm *arm = nullptr;
	auto it_arm = ModuleIDMap.find(EModuleID::MOD_ARM);
	if (it_arm != ModuleIDMap.end() && it_arm->second != nullptr) {
		arm = reinterpret_cast<CModArm *>(it_arm->second);
	}

	if (pESP32_) {
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
			if (arm) {
				pESP32_->armInfo.Yaw = static_cast<int>(arm->armInfo.angle_Yaw * 1000);
				pESP32_->armInfo.Pitch1 = static_cast<int>(arm->armInfo.angle_Pitch1 * 1000);
				pESP32_->armInfo.Pitch2 = static_cast<int>(arm->armInfo.angle_Pitch2 * 1000);
				pESP32_->armInfo.Roll = static_cast<int>(arm->armInfo.angle_Roll * 1000);
				pESP32_->armInfo.End_Pitch = static_cast<int>(arm->armInfo.angle_end_pitch * 1000);
				pESP32_->armInfo.End_Roll = static_cast<int>(arm->armInfo.angle_end_roll * 1000);
			}
			pESP32_->SendPackage(CDevESP32::EPackageID::ID_Motor_Arm_Pkg);
		}
	}

	if (pESP32_) {
		auto it_lf = DeviceIDMap.find(EDeviceID::DEV_CHAS_MTR_LF);
		if (it_lf != DeviceIDMap.end() && it_lf->second != nullptr)
			pESP32_->robotInfo.Chassis_LF = reinterpret_cast<CDevMtr *>(it_lf->second)->IsMotorOnline();

		auto it_rf = DeviceIDMap.find(EDeviceID::DEV_CHAS_MTR_RF);
		if (it_rf != DeviceIDMap.end() && it_rf->second != nullptr)
			pESP32_->robotInfo.Chassis_RF = reinterpret_cast<CDevMtr *>(it_rf->second)->IsMotorOnline();

		auto it_lb = DeviceIDMap.find(EDeviceID::DEV_CHAS_MTR_LB);
		if (it_lb != DeviceIDMap.end() && it_lb->second != nullptr)
			pESP32_->robotInfo.Chassis_LB = reinterpret_cast<CDevMtr *>(it_lb->second)->IsMotorOnline();

		auto it_rb = DeviceIDMap.find(EDeviceID::DEV_CHAS_MTR_RB);
		if (it_rb != DeviceIDMap.end() && it_rb->second != nullptr)
			pESP32_->robotInfo.Chassis_RB = reinterpret_cast<CDevMtr *>(it_rb->second)->IsMotorOnline();

		auto it_gimbal = DeviceIDMap.find(EDeviceID::DEV_GIMBAL_MTR);
		if (it_gimbal != DeviceIDMap.end() && it_gimbal->second != nullptr)
			pESP32_->robotInfo.Gimbal = reinterpret_cast<CDevMtr *>(it_gimbal->second)->IsMotorOnline();

		auto it_lift_l = DeviceIDMap.find(EDeviceID::DEV_SUBGANTRY_MTR_LIFT_L);
		if (it_lift_l != DeviceIDMap.end() && it_lift_l->second != nullptr)
			pESP32_->robotInfo.SubGantry_Lift_L = reinterpret_cast<CDevMtr *>(it_lift_l->second)->IsMotorOnline();

		auto it_lift_r = DeviceIDMap.find(EDeviceID::DEV_SUBGANTRY_MTR_LIFT_R);
		if (it_lift_r != DeviceIDMap.end() && it_lift_r->second != nullptr)
			pESP32_->robotInfo.SubGantry_Lift_R = reinterpret_cast<CDevMtr *>(it_lift_r->second)->IsMotorOnline();

		auto it_stretch_l = DeviceIDMap.find(EDeviceID::DEV_SUBGANTRY_MTR_STRETCH_L);
		if (it_stretch_l != DeviceIDMap.end() && it_stretch_l->second != nullptr)
			pESP32_->robotInfo.SubGantry_Stretch_L = reinterpret_cast<CDevMtr *>(it_stretch_l->second)->IsMotorOnline();

		auto it_stretch_r = DeviceIDMap.find(EDeviceID::DEV_SUBGANTRY_MTR_STRETCH_R);
		if (it_stretch_r != DeviceIDMap.end() && it_stretch_r->second != nullptr)
			pESP32_->robotInfo.SubGantry_Stretch_R = reinterpret_cast<CDevMtr *>(it_stretch_r->second)->IsMotorOnline();

		auto it_arm_yaw = DeviceIDMap.find(EDeviceID::DEV_ARM_MTR_YAW);
		if (it_arm_yaw != DeviceIDMap.end() && it_arm_yaw->second != nullptr)
			pESP32_->robotInfo.Arm_Yaw = reinterpret_cast<CDevMtr *>(it_arm_yaw->second)->IsMotorOnline();

		auto it_arm_p1 = DeviceIDMap.find(EDeviceID::DEV_ARM_MTR_PITCH1);
		if (it_arm_p1 != DeviceIDMap.end() && it_arm_p1->second != nullptr)
			pESP32_->robotInfo.Arm_Pitch1 = reinterpret_cast<CDevMtr *>(it_arm_p1->second)->IsMotorOnline();

		auto it_arm_p2 = DeviceIDMap.find(EDeviceID::DEV_ARM_MTR_PITCH2);
		if (it_arm_p2 != DeviceIDMap.end() && it_arm_p2->second != nullptr)
			pESP32_->robotInfo.Arm_Pitch2 = reinterpret_cast<CDevMtr *>(it_arm_p2->second)->IsMotorOnline();

		auto it_arm_roll = DeviceIDMap.find(EDeviceID::DEV_ARM_MTR_ROLL);
		if (it_arm_roll != DeviceIDMap.end() && it_arm_roll->second != nullptr)
			pESP32_->robotInfo.Arm_Roll = reinterpret_cast<CDevMtr *>(it_arm_roll->second)->IsMotorOnline();

		auto it_arm_end_l = DeviceIDMap.find(EDeviceID::DEV_ARM_MTR_END_L);
		if (it_arm_end_l != DeviceIDMap.end() && it_arm_end_l->second != nullptr)
			pESP32_->robotInfo.Arm_End_L = reinterpret_cast<CDevMtr *>(it_arm_end_l->second)->IsMotorOnline();

		auto it_arm_end_r = DeviceIDMap.find(EDeviceID::DEV_ARM_MTR_END_R);
		if (it_arm_end_r != DeviceIDMap.end() && it_arm_end_r->second != nullptr)
			pESP32_->robotInfo.Arm_End_R = reinterpret_cast<CDevMtr *>(it_arm_end_r->second)->IsMotorOnline();

		pESP32_->SendPackage(CDevESP32::EPackageID::ID_Motor_Info_Pkg);
	}

}

void CSystemESP32::HeartbeatHandler_() {

}


} // namespace my_engineer
