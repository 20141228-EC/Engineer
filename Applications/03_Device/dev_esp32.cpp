/******************************************************************************
 * @brief        
 * 
 * @file         dev_esp32.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-07-12
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "dev_esp32.hpp"

namespace my_engineer {

EAppStatus CDevESP32::InitDevice(const SDevInitParam_Base *pStructInitParam) {
	
	// 检查param是否正确
	if (pStructInitParam == nullptr) return APP_ERROR;
	if (pStructInitParam->deviceID == EDeviceID::DEV_NULL) return APP_ERROR;

	// 类型转换
	auto esp32Param = static_cast<const SDevInitParam_ESP32 *>(pStructInitParam);
	deviceID = esp32Param->deviceID;
	uartInterface_ = reinterpret_cast<CInfUART *>(InterfaceIDMap.at(esp32Param->interfaceID));

	auto callback = [this](auto &buffer, auto len) {
		if (len > 256) return;
		// 先清空
		std::fill(std::begin(rxBuffer_), std::end(rxBuffer_), 0);
		std::copy(buffer.data(), buffer.data() + len , rxBuffer_);
		rxTimestamp_ = HAL_GetTick();
	};
	uartInterface_->RegisterRxSaveDataCallback(callback);
	
	uartInterface_->StartTransfer();

	RegisterDevice_();

	deviceStatus = APP_OK;

	return APP_OK;
}

EAppStatus CDevESP32::SendPackage(EPackageID packageID) {

	if (deviceStatus == APP_RESET) return APP_ERROR;

	switch (packageID) {

		case ID_Motor_Info_Pkg: {
			char sendString[512];
			sprintf(sendString, 
				"Chassis_LF : %s\n"
				"Chassis_RF : %s\n"
				"Chassis_LB : %s\n"
				"Chassis_RB : %s\n"
				"Gimbal : %s\n"
				"SubGantry_Lift_L : %s\n"
				"SubGantry_Lift_R : %s\n"
				"SubGantry_Stretch_L : %s\n"
				"SubGantry_Stretch_R : %s\n"
				"Yaw : %s\n"
				"Pitch1 : %s\n"
				"Pitch2 : %s\n"
				"Roll : %s\n"
				"End_L : %s\n"
				"End_R : %s\n",
				robotInfo.Chassis_LF == 1 ? "online" : "offline",
				robotInfo.Chassis_RF == 1 ? "online" : "offline",
				robotInfo.Chassis_LB == 1 ? "online" : "offline",
				robotInfo.Chassis_RB == 1 ? "online" : "offline",
				robotInfo.Gimbal == 1 ? "online" : "offline",
				robotInfo.SubGantry_Lift_L == 1 ? "online" : "offline",
				robotInfo.SubGantry_Lift_R == 1 ? "online" : "offline",
				robotInfo.SubGantry_Stretch_L == 1 ? "online" : "offline",
				robotInfo.SubGantry_Stretch_R == 1 ? "online" : "offline",
				robotInfo.Arm_Yaw == 1 ? "online" : "offline",
				robotInfo.Arm_Pitch1 == 1 ? "online" : "offline",
				robotInfo.Arm_Pitch2 == 1 ? "online" : "offline",
				robotInfo.Arm_Roll == 1 ? "online" : "offline",
				robotInfo.Arm_End_L == 1 ? "online" : "offline",
				robotInfo.Arm_End_R == 1 ? "online" : "offline"
			);
			return ESP32_Print("%s", sendString);
		}

		case ID_Motor_Arm_Pkg: {
			char sendString[256];
			sprintf(sendString, 
				"Yaw = %d\n"
				"Pitch1 = %d\n"
				"Pitch2 = %d\n"
				"Roll = %d\n"
				"End_Pitch = %d\n"
				"End_Roll = %d\n",
				armInfo.Yaw,
				armInfo.Pitch1,
				armInfo.Pitch2,
				armInfo.Roll,
				armInfo.End_Pitch,
				armInfo.End_Roll
			);
			return ESP32_Print("%s", sendString);
		}

		default:
			return APP_ERROR;
	}
}

EAppStatus CDevESP32::ESP32_Print(const char *fmt, ...) {

	va_list args;
	va_start(args, fmt);
	char buffer[512];
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	uartInterface_->FormatTransmit("%s", buffer);

	return APP_OK;
}

void CDevESP32::UpdateHandler_() {

	if (deviceStatus == APP_RESET) return;

	if (rxTimestamp_ > lastHeartbeatTime_) {
		ResolveRxPackage_();
	}
}

void CDevESP32::HeartbeatHandler_() {
	
}

EAppStatus CDevESP32::ResolveRxPackage_() {

	if (deviceStatus == APP_RESET) return APP_ERROR;

	char *string = reinterpret_cast<char *>(rxBuffer_);

	//slider open. Yaw = -74.88, Pitch1 = 0.00, Pitch2 = 55.08, Roll = 0.00, End_L = 0.00, End_R = 0.00
	if (strstr(string, "slider open")) {
		armInfo.BlueTooth_Mode = 1; // 设置蓝牙模式为1
		sscanf(string, "slider open. Yaw = %d, Pitch1 = %d, Pitch2 = %d, Roll = %d, End_Pitch = %d, End_Roll = %d",
			&armInfo.Yaw,
			&armInfo.Pitch1,
			&armInfo.Pitch2,
			&armInfo.Roll,
			&armInfo.End_Pitch,
			&armInfo.End_Roll
		);
		return APP_OK;
	}

	if (strstr(string, "slider close")) {
		armInfo.BlueTooth_Mode = 0; // 设置蓝牙模式为0
		return APP_OK;
	}

	return APP_ERROR;
}




} // namespace my_engineer
