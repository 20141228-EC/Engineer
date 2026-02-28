/******************************************************************************
 * @brief        板间通信设备
 * 
 * @file         dev_board_link.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-06
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "dev_board_link.hpp"

namespace my_engineer{

/**
 * @brief 初始化板间通信设备
 * 
 * @retval EAppStatus
 */
EAppStatus CDevBoardLink::InitDevice(const SDevInitParam_Base *pStructInitParam){

    // 检查param是否正确
	if (pStructInitParam == nullptr) return APP_ERROR;
	if (pStructInitParam->deviceID == EDeviceID::DEV_NULL) return APP_ERROR;

	// 类型转换
	auto &boardLinkParam = *static_cast<const SDevInitParam_BoardLink *>(pStructInitParam);
	deviceID = boardLinkParam.deviceID;
	canInterface_ = reinterpret_cast<CInfCAN *>(InterfaceIDMap.at(boardLinkParam.interfaceID));

	// 初始化CAN接收节点
    auto canRxID = 0x302;
    canRxNode_.InitRxNode(boardLinkParam.interfaceID, canRxID, 
                                CInfCAN::ECanFrameType::DATA, 
                                CInfCAN::ECanFrameDlc::DLC_8);

	// 初始化CAN发送节点
	auto canTxID = 0x300;
	canTxNode_.InitTxNode(boardLinkParam.interfaceID, canTxID, 
                                CInfCAN::ECanFrameType::DATA, 
                                CInfCAN::ECanFrameDlc::DLC_8);

	RegisterDevice_(); ///< 注册设备

	deviceStatus = APP_OK;
	boardLinkStatus = EBoardLinkStatus::OFFLINE;

	return APP_OK;
}

/**
 * @brief 发送信息
 * 
 * @details 将从系统层获取，已经存到设备层结构体中的数据填入can发送缓冲区
 * 
 * @retval EAppStatus
 */
EAppStatus CDevBoardLink::SendPackage(EPacketID pack_id){

	// 检查设备状态
	if (deviceStatus == APP_RESET) return APP_ERROR;

	std::array<uint8_t, 8> data_buf{};

	switch (pack_id) ///< 这些获取的逻辑还得具体实现
	{
	case PKT_CTRL_FLAGS:{

		// 获取数据
		ctrlFlags_pkt.pack_id = PKT_CTRL_FLAGS;
		memcpy(data_buf.data(), &ctrlFlags_pkt, sizeof(ctrlFlags_pkt));

		// 填充数据帧
		Modify_CanTxData(data_buf.data());
		break;
	}
	case PKT_CTRLER_L_B:{
		// 获取数据
		controllerbackcmd_l_b_pkt.pack_id = PKT_CTRLER_L_B;
		memcpy(data_buf.data(), &controllerbackcmd_l_b_pkt, sizeof(controllerbackcmd_l_b_pkt));

		// 填充数据帧
		Modify_CanTxData(data_buf.data());
		break;
	}
	case PKT_CTRLER_L_F:{
		// 获取数据
		controllerfrontcmd_l_f_pkt.pack_id = PKT_CTRLER_L_F;
		memcpy(data_buf.data(), &controllerfrontcmd_l_f_pkt, sizeof(controllerfrontcmd_l_f_pkt));

		// 填充数据帧
		Modify_CanTxData(data_buf.data());
		break;
	}
	default:
		return APP_ERROR;
	}

	canTxNode_.Transmit(); ///< 发送数据

	return APP_OK;
}

/**
 * @brief 更新设备
 * 
 * @retval EAppStatus
 */
void CDevBoardLink::UpdateHandler_(){

	if (deviceStatus == APP_RESET) return;

	// if (rxTimestamp_ > lastHeartbeatTime_) {
		ResolveRxPackage_();
	// }
}

/**
 * @brief 设备心跳
 * 
 * @retval EAppStatus
 */
void CDevBoardLink::HeartbeatHandler_(){

	if (deviceStatus == APP_RESET) return;

	if (HAL_GetTick() - lastHeartbeatTime_ > 1000) {
		deviceStatus = APP_ERROR;
		boardLinkStatus = EBoardLinkStatus::OFFLINE;
	}
	else {
		deviceStatus = APP_OK;
		boardLinkStatus = EBoardLinkStatus::ONLINE;
	}
}

/**
 * @brief 解析接收数据包
 * 
 * @retval EAppStatus
 */
EAppStatus CDevBoardLink::ResolveRxPackage_(){

	// 检查设备状态
	if (deviceStatus == APP_RESET) return APP_ERROR;

	uint8_t pack_id = canRxNode_.dataBuffer[0];

	if (canRxNode_.timestamp >= lastHeartbeatTime_) {
		switch (pack_id)
		{
		case PKT_FEEDBACK:{

			// 将databuffer转化成结构体指针并解引用
			SFeedbackPack feedbackInfo = *reinterpret_cast<SFeedbackPack*>(canRxNode_.dataBuffer.data());
			// 后续如果发现不能正确读取或位运算有错的话，可以试试换成用feedbackInfo来接收
			fdbInfo_pkt.pack_id = canRxNode_.dataBuffer[0];
			fdbInfo_pkt.pack0_status = canRxNode_.dataBuffer[1] & 0x01;
			fdbInfo_pkt.pack1_status = (canRxNode_.dataBuffer[1] >> 1) & 0x01;
			fdbInfo_pkt.pack2_status = (canRxNode_.dataBuffer[1] >> 2) & 0x01;
			fdbInfo_pkt.pack3_status = (canRxNode_.dataBuffer[1] >> 3) & 0x01;

			// fdbInfo_pkt.pack0_status = feedbackInfo.pack0_status;
			// fdbInfo_pkt.pack1_status = feedbackInfo.pack1_status;
			// fdbInfo_pkt.pack2_status = feedbackInfo.pack2_status;
			// fdbInfo_pkt.pack3_status = feedbackInfo.pack3_status;
			break;
		}
		default:
			break;
		}
		rxTimestamp_ = canRxNode_.timestamp; ///< 更新时间戳
	}

    // 在所有case的外部，只要是这个设备的消息，就更新时间戳
    lastHeartbeatTime_ = canRxNode_.timestamp;

	return APP_OK;
}

}
