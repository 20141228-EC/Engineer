/**
 * @file sys_board_link.cpp
 * @author Ciallo(1002046597@qq.com)
 * @brief 板间通信系统层源文件
 * @version 1.0
 * @date 2025-12-11
 *
 * @details 封装板间通信设备，提供统一的遥控器数据接口
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Core.hpp"

/**
 * @brief 使用板间通信作为遥控器数据源
 * @note  启用(1): 遥控器数据从板间通信(CAN)获取，用于副板
 *        禁用(0): 遥控器数据从本地DBUS获取，用于主板或单板调试
 */
#define USE_BOARD_LINK_REMOTE   0

namespace my_engineer {

// 实例化板间通信系统
CSystemBoardLink SysBoardLink;

/**
 * @brief 初始化板间通信系统
 *
 * @param pStruct 初始化参数指针
 * @return EAppStatus 初始化状态
 */
EAppStatus CSystemBoardLink::InitSystem(SSystemInitParam_Base *pStruct) {

    // 检查参数及ID是否为空
    if (pStruct == nullptr) return APP_ERROR;
    if (pStruct->systemID == ESystemID::SYS_NULL) return APP_ERROR;

    // 类型转换
    auto &param = *reinterpret_cast<SSystemInitParam_BoardLink *>(pStruct);

    // 获取板间通信设备指针
    systemID = param.systemID;
    auto it = DeviceIDMap.find(param.boardLinkDevID);
    if (it != DeviceIDMap.end() && it->second != nullptr) {
        pBoardLinkDev_ = static_cast<CDevBoardLink *>(it->second);
    }

    // 注册系统
    RegisterSystem_();

    // 初始状态为离线，等待心跳确认
    systemStatus = APP_ERROR;
    return APP_OK;
}

/**
 * @brief 检查板间通信是否在线
 *
 * @return true - 在线
 * @return false - 离线或复位状态
 */
bool CSystemBoardLink::IsOnline() const {
    if (pBoardLinkDev_ == nullptr) return false;
    return pBoardLinkDev_->linkStatus == CDevBoardLink::EBoardLinkStatus::ONLINE;
}

/**
 * @brief 获取通信状态
 *
 * @return CDevBoardLink::EBoardLinkStatus 通信状态枚举
 */
CDevBoardLink::EBoardLinkStatus CSystemBoardLink::GetLinkStatus() const {
    if (pBoardLinkDev_ == nullptr) return CDevBoardLink::EBoardLinkStatus::RESET;
    return pBoardLinkDev_->linkStatus;
}

/**
 * @brief 更新处理
 * @note  在SystemTask中以500Hz频率调用
 */
void CSystemBoardLink::UpdateHandler_() {
    // 检查系统状态
    if (systemStatus != APP_OK) return;

    UpdateRemoteData_();
    UpdateCtrlFlags_();

    // 发送反馈给主板
    if (pBoardLinkDev_ != nullptr) {
        pBoardLinkDev_->FillFeedbackBuffer();
    }
}

/**
 * @brief 心跳处理
 *
 */
void CSystemBoardLink::HeartbeatHandler_() {
    // 检查系统状态
    if (systemStatus == APP_RESET) return;
    if (pBoardLinkDev_ == nullptr) return;

    // 根据设备通信状态更新系统状态
    if (pBoardLinkDev_->linkStatus == CDevBoardLink::EBoardLinkStatus::ONLINE) {
        systemStatus = APP_OK;
    } else {
        systemStatus = APP_ERROR;
    }
}

/**
 * @brief 更新遥控器数据
 * @note  将设备层的原始摇杆值转换为系统层格式
 *        主板发送值 × 3 ÷ 100 = 原始摇杆值 (-660 ~ 660)
 *        原始摇杆值 ÷ 6.6 = 百分比值 (-100 ~ 100)
 *        合并计算: 接收值 ÷ 220 = 百分比值
 *
 * @return EAppStatus 更新状态
 */
EAppStatus CSystemBoardLink::UpdateRemoteData_() {

    if (pBoardLinkDev_ == nullptr) return APP_ERROR;

    // 转换摇杆数据: 接收值 * 3 / 100 / 6.6 = 接收值 / 220
    constexpr float_t kJoystickScale = 220.0f;
    remoteInfo.joystick_RX = static_cast<float_t>(pBoardLinkDev_->remoteJoystick1.joystick_RX) / kJoystickScale;
    remoteInfo.joystick_RY = static_cast<float_t>(pBoardLinkDev_->remoteJoystick1.joystick_RY) / kJoystickScale;
    remoteInfo.joystick_LX = static_cast<float_t>(pBoardLinkDev_->remoteJoystick1.joystick_LX) / kJoystickScale;
    remoteInfo.joystick_LY = static_cast<float_t>(pBoardLinkDev_->remoteJoystick2.joystick_LY) / kJoystickScale;
    remoteInfo.thumbWheel  = static_cast<float_t>(pBoardLinkDev_->remoteJoystick2.thumbWheel) / kJoystickScale;

    // 拨杆状态（直接复制，值为1/2/3）
    remoteInfo.switch_L = pBoardLinkDev_->remoteJoystick2.switch_L;
    remoteInfo.switch_R = pBoardLinkDev_->remoteJoystick2.switch_R;

    return APP_OK;
}

/**
 * @brief 更新控制标志
 * @note  解析设备层的控制标志包
 *
 * @return EAppStatus 更新状态
 */
EAppStatus CSystemBoardLink::UpdateCtrlFlags_() {

    if (pBoardLinkDev_ == nullptr) return APP_ERROR;

    const auto &flags = pBoardLinkDev_->ctrlFlags;

    // 控制模式
    ctrlFlags.chassis_ctrl    = flags.chassis_ctrl;
    ctrlFlags.gimbal_ctrl     = flags.gimbal_ctrl;
    ctrlFlags.arm_front_ctrl  = flags.arm_front_ctrl;
    ctrlFlags.arm_rear_ctrl   = flags.arm_rear_ctrl;

    // 使能标志
    ctrlFlags.arm_enable      = flags.arm_enable;
    ctrlFlags.gimbal_enable   = flags.gimbal_enable;
    ctrlFlags.chassis_enable  = flags.chassis_enable;

    // 状态标志
    ctrlFlags.rc_online       = flags.rc_online;
    ctrlFlags.is_rc_ctrl      = flags.is_rc_ctrl;
    ctrlFlags.is_key_ctrl     = flags.is_key_ctrl;

    return APP_OK;
}

}   // namespace my_engineer
    