/**
 * @file sys_board_link.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 板间通信系统层源文件
 * @version 1.0
 * @date 2026-04-11
 *
 * @details 封装板间通信设备，提供统一的遥控器数据接口
 *
 * @copyright Copyright (c) 2026
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

    UpdateCtrlInfos_();

    // 发送信息（轮询发送或者根据需求分别发送，这里依次发送）
    if (pBoardLinkDev_ != nullptr) {
        static uint8_t send_cnt = 0;
        
        switch(send_cnt % 3) {
            case 0:
                pBoardLinkDev_->SendPackage(CDevBoardLink::PKT_CTRL_INFOS);
                break;
            case 1:
                pBoardLinkDev_->SendPackage(CDevBoardLink::PKT_JOINT_INFOS);
                break;
            case 2:
                pBoardLinkDev_->SendPackage(CDevBoardLink::PKT_OTHER_INFOS);
                break;
        }
        send_cnt++;
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

    // // 根据设备通信状态更新系统状态
    // if (pBoardLinkDev_->linkStatus == CDevBoardLink::EBoardLinkStatus::ONLINE) {
        systemStatus = APP_OK;
    // } else {
    //     systemStatus = APP_ERROR;
    // }
}

/**
 * @brief 更新控制信息
 * @note  填充设备层的控制信息包
 *
 * @return EAppStatus 更新状态
 */
EAppStatus CSystemBoardLink::UpdateCtrlInfos_() {

    if (pBoardLinkDev_ == nullptr) return APP_ERROR;

    // 控制信息
    pBoardLinkDev_->ctrlInfo_.remote_is_online = ctrlInfos.remote_is_online;
    pBoardLinkDev_->ctrlInfo_.speed_x = ctrlInfos.speed_x;
    pBoardLinkDev_->ctrlInfo_.speed_y = ctrlInfos.speed_y;
    pBoardLinkDev_->ctrlInfo_.speed_w = ctrlInfos.speed_w;

    // 关节信息 (包括夹爪)
    pBoardLinkDev_->angleInfo_.grip_close = angleInfos.grip_close;
    pBoardLinkDev_->angleInfo_.pitch1 = angleInfos.pitch1;
    pBoardLinkDev_->angleInfo_.pitch2 = angleInfos.pitch2;
    pBoardLinkDev_->angleInfo_.pitch3 = angleInfos.pitch3;

    // 其他信息 (包括陀螺仪、小陀螺)
    pBoardLinkDev_->otherInfo_.yaw_gyro = otherInfos.yaw_gyro;
    pBoardLinkDev_->otherInfo_.is_spin_on = otherInfos.is_spin_on;
    pBoardLinkDev_->otherInfo_.autoTask = otherInfos.autoTask;

    return APP_OK;
}

}   // namespace my_engineer
    