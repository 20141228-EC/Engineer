/******************************************************************************
 * @brief        
 * 
 * @file         sys_esp32.hpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-07-12
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/


#ifndef SYS_ESP32_HPP
#define SYS_ESP32_HPP

#include "sys_common.hpp"

#include "Device.hpp"
#include "Module.hpp"

namespace my_engineer {

/**
 * @brief ESP32系统类
 * 
 */
class CSystemESP32 final: public CSystemBase {
public:

	struct SSystemInitParam_ESP32: public SSystemInitParam_Base {
		EDeviceID esp32DevID = EDeviceID::DEV_NULL; ///< ESP32设备ID
	};

	EAppStatus InitSystem(SSystemInitParam_Base *pStruct) final;

	struct SArmInfo {
		float_t Yaw;            ///< Yaw角度
		float_t Pitch1;        ///< Pitch1角度
		float_t Pitch2;        ///< Pitch2角度
		float_t Roll;          ///< Roll角度
		float_t End_Pitch;         ///< 末端Pitch角度
		float_t End_Roll;       ///< 末端Roll角度
	} BLEInfo; ///< 蓝牙模式信息

	bool BLE_Mode_Open = false; ///< 是否为BLE模式

private:
	CDevESP32 *pESP32_ = nullptr; ///< ESP32设备指针

	void UpdateHandler_() override;

	void HeartbeatHandler_() override;
};

extern CSystemESP32 SysESP32; ///< ESP32系统实例

} // namespace my_engineer

#endif // SYS_ESP32_HPP
