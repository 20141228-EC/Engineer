/******************************************************************************
 * @brief        
 * 
 * @file         dev_controller_link.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V2.0
 * @date         2025-04-05
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-17
 *
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "dev_controller_link.hpp"

namespace my_engineer {

/**
 * @brief 初始化控制器通信设备
 * 
 * @param param 
 * @return EAppStatus 
 */
EAppStatus CDevControllerLink::InitDevice(const SDevInitParam_Base *pStructInitParam){

	// 检查param是否正确
	if (pStructInitParam == nullptr) return APP_ERROR;
	if (pStructInitParam->deviceID == EDeviceID::DEV_NULL) return APP_ERROR;

	// 类型转换
	auto &controllerLinkParam = *static_cast<const SDevInitParam_ControllerLink *>(pStructInitParam);
	deviceID = controllerLinkParam.deviceID;
	uartInterface_ = reinterpret_cast<CInfUART *>(InterfaceIDMap.at(controllerLinkParam.interfaceID));

	auto callback = [this](auto &buffer, auto len) {
		if (len > 512) return;
		// 写入当前写缓冲区，不影响主循环正在读取的另一个缓冲区
		auto &writeBuf = rxBuffers_[rxWriteIdx_];
		std::copy(buffer.data(), buffer.data() + len, writeBuf.data());
		if (len < 512) {
			std::fill(writeBuf.begin() + len, writeBuf.end(), 0);
		}
		rxNewData_ = true;
		rxTimestamp_ = HAL_GetTick();
	};

	RegisterDevice_();
	uartInterface_->RegisterRxSaveDataCallback(callback);

	deviceStatus = APP_OK;
	controllerLinkStatus = EControllerLinkStatus::OFFLINE;

	return APP_OK;
}

/**
 * @brief 发送数据
 * 
 * @param buffer 
 * @param len 
 * @return EAppStatus 
 */
EAppStatus CDevControllerLink::SendPackage(EPackageID packageID, SPkgHeader &packageHeader) {

	// 检查设备状态
	if (deviceStatus == APP_RESET) return APP_ERROR;

	switch (packageID) {

		case ID_CONTROLLER_DATA: {
			auto pkg = reinterpret_cast<SControllerDataPkg *>(&packageHeader);
			pkg->header.SOF = 0xA5;
			pkg->header.seq++;
			pkg->header.pkgLen = sizeof(SControllerDataPkg) - sizeof(SPkgHeader) - 2; // 30 bytes
			pkg->header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&(pkg->header)), 4);
			pkg->header.cmd_Id = 0x0302;
			pkg->CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(pkg), sizeof(SControllerDataPkg) - 2);

			return uartInterface_->Transmit(reinterpret_cast<uint8_t *>(pkg), sizeof(SControllerDataPkg));
		}

		case ID_ROBOT_DATA: {
			auto pkg = reinterpret_cast<SRobotDataPkg *>(&packageHeader);
			pkg->header.SOF = 0xA5;
			pkg->header.seq++;
			pkg->header.pkgLen = sizeof(SRobotDataPkg) - sizeof(SPkgHeader) - 2; // 30 bytes
			pkg->header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&(pkg->header)), 4);
			pkg->header.cmd_Id = 0x0309;
			pkg->CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(pkg), sizeof(SRobotDataPkg) - 2);

			return uartInterface_->Transmit(reinterpret_cast<uint8_t *>(pkg), sizeof(SRobotDataPkg));
		}

		default:
			return APP_ERROR;
	}
}

/**
 * @brief 更新处理
 *
 * 双缓冲交换策略：
 *   1. ISR 始终写入 rxBuffers_[rxWriteIdx_]
 *   2. 主循环检测到新数据后，在临界区内交换索引
 *   3. 主循环从旧写缓冲区（现读缓冲区）解析数据
 *   4. ISR 此后写入另一个缓冲区，互不干扰
 */
void CDevControllerLink::UpdateHandler_(){
	if (deviceStatus == APP_RESET) return;

	if (rxNewData_ && rxTimestamp_ > lastHeartbeatTime_) {
		// 临界区：仅交换索引 + 清标志位，耗时极短
		__disable_irq();
		uint8_t readIdx = rxWriteIdx_;        // 拿到刚写完的缓冲区
		rxWriteIdx_ = 1 - rxWriteIdx_;        // ISR 下次写入另一个缓冲区
		rxNewData_ = false;
		__enable_irq();

		ResolveRxPackage_(rxBuffers_[readIdx]);
	}
}

/**
 * @brief 心跳处理
 * 
 */
void CDevControllerLink::HeartbeatHandler_(){

	if (deviceStatus == APP_RESET) return;

	if (HAL_GetTick() - lastHeartbeatTime_ > 1000) {
		deviceStatus = APP_ERROR;
		controllerLinkStatus = EControllerLinkStatus::OFFLINE;
	}
	else {
		deviceStatus = APP_OK;
		controllerLinkStatus = EControllerLinkStatus::ONLINE;
	}
}

/**
 * @brief 解析接收数据包
 *
 * @param buffer 待解析的缓冲区引用（由 UpdateHandler_ 传入已交换的读缓冲区）
 * @return EAppStatus
 */
EAppStatus CDevControllerLink::ResolveRxPackage_(std::array<uint8_t, 512> &buffer){

	if (deviceStatus == APP_RESET) return APP_ERROR;

	for (size_t i = 0; i < buffer.size(); i++)
	{
		if (buffer[i] != 0xA5) {
			continue;
		}

		auto header = reinterpret_cast<SPkgHeader *>(&buffer[i]);
		if (CCrcValidator::Crc8Verify(&buffer[i], header->CRC8, 4) != APP_OK) {
			continue;
		}

		switch (header->cmd_Id) {

			case 0x0302: {
				if (i + sizeof(SControllerDataPkg) > buffer.size())
					break;
				auto pkg = reinterpret_cast<SControllerDataPkg *>(header);
				if (CCrcValidator::Crc16Verify(reinterpret_cast<uint8_t *>(pkg), pkg->CRC16, sizeof(SControllerDataPkg) - 2) != APP_OK)
					break;
				controllerData_info_pkg = *pkg;
				i += sizeof(SControllerDataPkg) - 1;
				break;
			}

			case 0x0309: {
				if (i + sizeof(SRobotDataPkg) > buffer.size())
					break;
				auto pkg = reinterpret_cast<SRobotDataPkg *>(header);
				if (CCrcValidator::Crc16Verify(reinterpret_cast<uint8_t *>(pkg), pkg->CRC16, sizeof(SRobotDataPkg) - 2) != APP_OK)
					break;
				robotData_info_pkg = *pkg;
				i += sizeof(SRobotDataPkg) - 1;
				break;
			}

			default: {
				break;
			}
		}
	}

	buffer.fill(0);
	lastHeartbeatTime_ = HAL_GetTick();
	return APP_OK;

}

} // namespace my_engineer