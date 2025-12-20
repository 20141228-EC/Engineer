/******************************************************************************
 * @brief   板间通信设备类实现
 *
 * @file    dev_board_link.cpp
 * @author  Ciallo～(∠·ω< )⌒☆(1002046597@qq.com)
 * @version V2.0
 * @date    2025-12-09
 *
 * @copyright Copyright (c) 2025
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
    txNode_ = param.txNode;

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
        rxStatus_.Clear();  // 离线时清除接收状态
    }
    else {
        // 在线
        deviceStatus = APP_OK;
        linkStatus = EBoardLinkStatus::ONLINE;
    }

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

        case PKT_REMOTE_1: {
            ///<直接通过内存的访问方式进行转换，因为数据包是对齐的
            auto pkg = reinterpret_cast<SRemoteJoystick1 *>(rxNode_.dataBuffer.data());
            remoteJoystick1 = *pkg;
            rxStatus_.SetReceived(PKT_REMOTE_1);
            break;
        }

        case PKT_REMOTE_2: {
            auto pkg = reinterpret_cast<SRemoteJoystick2 *>(rxNode_.dataBuffer.data());
            remoteJoystick2 = *pkg;
            rxStatus_.SetReceived(PKT_REMOTE_2);
            break;
        }

        case PKT_CTRL_FLAGS: {
            auto pkg = reinterpret_cast<SControlFlags *>(rxNode_.dataBuffer.data());
            ctrlFlags = *pkg;
            rxStatus_.SetReceived(PKT_CTRL_FLAGS);
            break;
        }

        default:
            return APP_ERROR;
    }

    return APP_OK;
}

/**
 * @brief 填充反馈数据到发送缓冲区
 * @note  只填充数据到发送缓冲区，不发送。发送由CAN接口层统一管理,同时这里采用了临时类型转换的方式，避免了冗余的内存拷贝。
 */
void CDevBoardLink::FillFeedbackBuffer() {

    if (txNode_ == nullptr) return;

    // 直接把发送缓冲区当作结构体来填充
    auto pkg = reinterpret_cast<SFeedbackPack *>(txNode_->dataBuffer.data());
    pkg->pack_id = PKT_FEEDBACK;
    pkg->rx_status = rxStatus_.GetBits();
    pkg->link_status = static_cast<uint8_t>(linkStatus);

    // 发送反馈后清零接收状态，下一周期重新统计
    rxStatus_.Clear();
}

} // namespace my_engineer
