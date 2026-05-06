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
	timeoutParam_.offlineTimeout = boardLinkParam.offlineTimeout;

	// 初始化CAN接收节点
    constexpr uint32_t kCanRxID = 0x300;
    rxNode_.InitRxNode(boardLinkParam.interfaceID, kCanRxID,
                                CInfCAN::ECanFrameType::DATA, 
                                CInfCAN::ECanFrameDlc::DLC_8);

	// 初始化CAN发送节点
    constexpr uint32_t kCanTxID = 0x300;
	txNode_.InitTxNode(boardLinkParam.interfaceID, kCanTxID,
                                CInfCAN::ECanFrameType::DATA, 
                                CInfCAN::ECanFrameDlc::DLC_8);

	RegisterDevice_(); ///< 注册设备

	deviceStatus = APP_OK;
	linkStatus = EBoardLinkStatus::OFFLINE;

    feedbackPack_.pack_id = PKT_FEEDBACK;

	return APP_OK;
}

/**
 * @brief 发送信息
 * 
 * @details 将从系统层获取，已经存到设备层结构体中的数据填入can发送缓冲区
 * 
 * @retval EAppStatus
 */
EAppStatus CDevBoardLink::SendPackage(){

	// 检查设备状态
	if (deviceStatus == APP_RESET) return APP_ERROR;

	std::array<uint8_t, 8> data_buf{};
    feedbackPack_.pack_id = PKT_FEEDBACK;
    feedbackPack_.rx_status = 0;
    if (ctrlInfo_.pack_id == PKT_CTRL_INFOS) feedbackPack_.rx_status |= (1u << 0);
    if (angleInfo_.pack_id == PKT_JOINT_INFOS) feedbackPack_.rx_status |= (1u << 1);
    if (otherInfo_.pack_id == PKT_OTHER_INFOS) feedbackPack_.rx_status |= (1u << 2);
    feedbackPack_.link_status = static_cast<uint8_t>(linkStatus);

    memcpy(data_buf.data(), &feedbackPack_, sizeof(feedbackPack_));
	Modify_CanTxData(data_buf.data());
	// txNode_.Transmit(); ///< 发送反馈数据

	return APP_OK;
}

/**
 * @brief 更新设备
 * 
 * @retval EAppStatus
 */
void CDevBoardLink::UpdateHandler_(){

	if (deviceStatus == APP_RESET) return;

    ParseRxPacket_();
}

/**
 * @brief 设备心跳
 * 
 * @retval EAppStatus
 */
void CDevBoardLink::HeartbeatHandler_(){

	if (deviceStatus == APP_RESET) return;

	if (HAL_GetTick() - timeoutParam_.rxTimestamp > timeoutParam_.offlineTimeout) {
		deviceStatus = APP_ERROR;
		linkStatus = EBoardLinkStatus::OFFLINE;
	}
	else {
		deviceStatus = APP_OK;
		linkStatus = EBoardLinkStatus::ONLINE;
	}
}

/**
 * @brief 解析接收数据包
 * 
 * @retval EAppStatus
 */
EAppStatus CDevBoardLink::ParseRxPacket_(){

	// 检查设备状态
	if (deviceStatus == APP_RESET) return APP_ERROR;

	const uint8_t pack_id = rxNode_.dataBuffer[0];

	if (rxNode_.timestamp == timeoutParam_.lastParseTime) {
        return APP_OK;
    }

    switch (pack_id)
	{
        case PKT_CTRL_INFOS: {
            ctrlInfo_ = *reinterpret_cast<SCtrlInfo *>(rxNode_.dataBuffer.data());
            break;
        }
        case PKT_JOINT_INFOS: {
            angleInfo_ = *reinterpret_cast<SAngleInfo *>(rxNode_.dataBuffer.data());
            break;
        }
        case PKT_OTHER_INFOS: {
            otherInfo_ = *reinterpret_cast<SOtherInfo *>(rxNode_.dataBuffer.data());
            break;
        }
        default:
			break;
	}

    timeoutParam_.lastParseTime = rxNode_.timestamp;
    timeoutParam_.rxTimestamp = rxNode_.timestamp;

	return APP_OK;
}

}
