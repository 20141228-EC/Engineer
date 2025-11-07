/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-07-16 17:10:01
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-18 01:00:20
 */
/******************************************************************************
 * @brief        
 * 
 * @file         dev_esp32.hpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-07-12
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/


#ifndef DEV_ESP32_HPP
#define DEV_ESP32_HPP

#include "dev_common.hpp"

#include "algo_crc.hpp"
#include "inf_uart.hpp"

namespace my_engineer {

/**
 * @brief ESP32设备类
 */
class CDevESP32 : public CDevBase {
public:
	
	struct SDevInitParam_ESP32 : public SDevInitParam_Base {
		EInterfaceID interfaceID = EInterfaceID::INF_NULL; ///< 接口ID
	};

	enum EPackageID : uint8_t {
		ID_NULL = 0,
		ID_Motor_Info_Pkg = 1, ///< 电机信息包
		ID_Motor_Arm_Pkg, ///< 机械臂电机包
	};

	struct SRobotInfo {
		uint8_t Chassis_LF;
		uint8_t Chassis_RF;
		uint8_t Chassis_LB;
		uint8_t Chassis_RB;
		uint8_t Gimbal;
		uint8_t SubGantry_Lift_L;
		uint8_t SubGantry_Lift_R;
		uint8_t SubGantry_Stretch_L;
		uint8_t SubGantry_Stretch_R;
		uint8_t Arm_Yaw;
		uint8_t Arm_Pitch1;
		uint8_t Arm_Pitch2;
		uint8_t Arm_Roll;
		uint8_t Arm_End_L;
		uint8_t Arm_End_R;
	} robotInfo; ///< 机器人信息

	struct SArmInfo {
		uint8_t BlueTooth_Mode;
		int Yaw;
		int Pitch1;
		int Pitch2;
		int Roll;
		int End_Pitch;
		int End_Roll;
	} armInfo; ///< 机械臂信息

	CDevESP32() {
		deviceType = EDevType::DEV_ESP32; ///< 设置设备类型为ESP32
	}

	EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override;

	EAppStatus SendPackage(EPackageID packageID);

	EAppStatus ESP32_Print(const char *fmt, ...);

private:

	CInfUART *uartInterface_ = nullptr; ///< 串口句柄

	uint8_t rxBuffer_[512] = {0}; ///< 接收缓冲区

	uint32_t rxTimestamp_ = 0; ///< 接收时间戳

	void UpdateHandler_() override;

	void HeartbeatHandler_() override;

	EAppStatus ResolveRxPackage_();

};

} // namespace my_engineer

#endif // DEV_ESP32_HPP
