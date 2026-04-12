/******************************************************************************
 * @brief   板间通信设备类实现
 *
 * @file    dev_board_link.cpp
 * @author  sllllr (299708711@qq.com)
 * @version V1.0
 * @date    2026-04-11
 *
 *
 * @copyright Copyright (c) 2026
 *
 ******************************************************************************/

#include "dev_board_link.hpp"

namespace my_engineer {

/**
 * @brief 初始化板间通信设备
 */
EAppStatus CDevBoardLink::InitDevice(const SDevInitParam_Base *pStructInitParam) {

    // 参数检查
    if (pStructInitParam == nullptr) return APP_ERROR;
    if (pStructInitParam->deviceID == EDeviceID::DEV_NULL) return APP_ERROR;

    // 类型转换
    auto &param = *static_cast<const SDevInitParam_BoardLink *>(pStructInitParam);

    // 检查接口ID有效性
    if (param.interfaceID == EInterfaceID::INF_NULL) return APP_ERROR;

    // 保存配置
    deviceID = param.deviceID;
    timeoutParam_.offlineTimeout = param.offlineTimeout;

    // 初始化CAN接收节点
    txNode_.InitTxNode(
        param.interfaceID,
        0x300,
        CInfCAN::ECanFrameType::DATA,
        CInfCAN::ECanFrameDlc::DLC_8
    );


    // 初始化CAN接收节点
    rxNode_.InitRxNode(
        param.interfaceID,
        0x300,
        CInfCAN::ECanFrameType::DATA,
        CInfCAN::ECanFrameDlc::DLC_8
    );

    // 注册设备
    RegisterDevice_();

    // 更新状态
    deviceStatus = APP_OK;
    linkStatus = EBoardLinkStatus::OFFLINE;

    return APP_OK;
}

/**
 * @brief 更新处理
 */
void CDevBoardLink::UpdateHandler_() {

    if (deviceStatus == APP_RESET) return;

    // 检查是否有新数据（通过比较时间戳）
    if (rxNode_.timestamp > timeoutParam_.lastParseTime) {
        ParseRxPacket_();
        timeoutParam_.lastParseTime = rxNode_.timestamp;//接收中断中的时间戳
        timeoutParam_.rxTimestamp = rxNode_.timestamp;
    }
}

/**
 * @brief 心跳处理
 * @note  仅用于离线检测，不发送反馈（反馈在系统层Update中发送，500Hz）
 */
void CDevBoardLink::HeartbeatHandler_() {

    if (deviceStatus == APP_RESET) return;

    // 检查离线
    uint32_t currentTime = HAL_GetTick();

    if (currentTime - timeoutParam_.rxTimestamp > timeoutParam_.offlineTimeout) {
        // 超时，离线
        deviceStatus = APP_ERROR;
        linkStatus = EBoardLinkStatus::OFFLINE;
    }
    else {
        // 在线
        deviceStatus = APP_OK;
        linkStatus = EBoardLinkStatus::ONLINE;
    }

}

/**
 * @brief 发送信息
 * 
 * @details 将从系统层获取，已经存到设备层结构体中的数据填入can发送缓冲区
 * 
 * @retval EAppStatus
 */
EAppStatus CDevBoardLink::SendPackage(void){

	// 检查设备状态
	if (deviceStatus == APP_RESET) return APP_ERROR;

	std::array<uint8_t, 8> data_buf{};

    memcpy(data_buf.data(), &ctrlInfo_, sizeof(ctrlInfo_));

    Modify_CanTxData(data_buf.data());

	txNode_.Transmit(); ///< 发送数据

	return APP_OK;
}

/**
 * @brief 解析接收到的数据包
 */
EAppStatus CDevBoardLink::ParseRxPacket_() {

    if (deviceStatus == APP_RESET) return APP_ERROR;

    // 获取pack_id（第一个字节）
    uint8_t packId = rxNode_.dataBuffer[0];

    // 根据pack_id分发到不同的数据包
    switch (packId) {

        default:
            return APP_ERROR;
    }

    return APP_OK;
}

} // namespace my_engineer
