/******************************************************************************
 * @brief   板间通信设备类实现
 *
 * @file    dev_board_link.cpp
 * @author  Zoe
 * @version V1.0
 * @date    2025-12-06
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
        timeoutParam_.lastParseTime = rxNode_.timestamp;
        timeoutParam_.rxTimestamp = rxNode_.timestamp;
    }
}

/**
 * @brief 心跳处理
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

    // 发送反馈给主板
    SendFeedback();
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

        case PKT_ARM_JOINT1: {
            auto pkg = reinterpret_cast<SArmJoint1Target *>(rxNode_.dataBuffer.data());///<直接通过内存的访问方式进行转换，因为数据包是对齐的
            armJoint1Target = *pkg;
            rxStatus_.SetReceived(PKT_ARM_JOINT1);
            break;
        }

        case PKT_ARM_JOINT2: {
            auto pkg = reinterpret_cast<SArmJoint2Target *>(rxNode_.dataBuffer.data());
            armJoint2Target = *pkg;
            rxStatus_.SetReceived(PKT_ARM_JOINT2);
            break;
        }

        case PKT_GIMBAL: {
            auto pkg = reinterpret_cast<SGimbalTarget *>(rxNode_.dataBuffer.data());
            gimbalTarget = *pkg;
            rxStatus_.SetReceived(PKT_GIMBAL);
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
 * @note  只填充数据，不发送。发送由 sys_task.cpp 统一管理（500Hz）
 */
void CDevBoardLink::SendFeedback() {

    if (txNode_ == nullptr) return;

    // 直接把发送缓冲区当作结构体来填充
    auto pkg = reinterpret_cast<SFeedbackPack *>(txNode_->dataBuffer.data());
    pkg->pack_id = PKT_FEEDBACK;
    pkg->rx_status = rxStatus_.GetBits();
    pkg->link_status = static_cast<uint8_t>(linkStatus);

    // test板通是否良好
    //txNode_->Transmit();
}

} // namespace my_engineer
