/**
 * @file Core.cpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 系统核心源文件
 * @version 1.0
 * @date 2024-11-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "Core.hpp"

namespace my_engineer {

// 实例化系统核心
CSystemCore SystemCore;

/**
 * @brief 初始化系统核心
 * 
 * @return EAppStatus 
 */
EAppStatus CSystemCore::InitSystemCore() {
    
    // 启动通信接口发送
    for (const auto &item : InterfaceIDMap) {
        item.second->StartTransfer();
    }


    // 初始化所有系统
    CSystemRemote::SSystemInitParam_Remote remoteInitParam;
    remoteInitParam.systemID = ESystemID::SYS_REMOTE;
    remoteInitParam.remoteDevID = EDeviceID::DEV_RC_DR16;
    SysRemote.InitSystem(&remoteInitParam);

    CSystemVision::SSystemInitParam_Vision visionInitParam;
    visionInitParam.systemID = ESystemID::SYS_VISION;
    visionInitParam.visionDevID = EDeviceID::DEV_VISION;
    SysVision.InitSystem(&visionInitParam);

    CSystemReferee::SSystemInitParam_Referee refereeInitParam;
    refereeInitParam.systemID = ESystemID::SYS_REFEREE;
    refereeInitParam.refereeDevID = EDeviceID::DEV_RM_REFEREE;
    SysReferee.InitSystem(&refereeInitParam);

    CSystemControllerLink::SSystemInitParam_ControllerLink controllerLinkInitParam;
    controllerLinkInitParam.systemID = ESystemID::SYS_CONTROLLER_LINK;
    controllerLinkInitParam.controllerLinkDevID = EDeviceID::DEV_CONTROLLER_LINK;
    SysControllerLink.InitSystem(&controllerLinkInitParam);

    // CSystemESP32::SSystemInitParam_ESP32 esp32InitParam;
    // esp32InitParam.systemID = ESystemID::SYS_ESP32;
    // esp32InitParam.esp32DevID = EDeviceID::DEV_ESP32;
    // SysESP32.InitSystem(&esp32InitParam);

    CSystemBoardLink::SSystemInitParam_BoardLink boardLinkInitParam;
    boardLinkInitParam.systemID = ESystemID::SYS_BOARD_LINK;
    boardLinkInitParam.boardLinkDevID = EDeviceID::DEV_BOARD_LINK;
    SysBoardLink.InitSystem(&boardLinkInitParam);

    // 获取模块的指针（安全查找，避免异常）
    auto it_chassis = ModuleIDMap.find(EModuleID::MOD_CHASSIS);
    if (it_chassis != ModuleIDMap.end() && it_chassis->second != nullptr) {
        pchassis_ = reinterpret_cast<CModChassis *>(it_chassis->second);
    }

    proc_waitMs(1200); // 等待系统初始化完成

    coreStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 更新处理
 * 
 */
void CSystemCore::UpdateHandler_() {

    // 检查系统核心状态
    if (coreStatus == APP_RESET) return;

    // 检查遥控器系统状态
    if (SysBoardLink.ctrlInfos.remote_is_online == APP_ERROR) return;

            // 左下右上键盘控制
        if (SysRemote.remoteInfo.remote.switch_L == 2
        && SysRemote.remoteInfo.remote.switch_R == 1)
        {
            ControlFromKeyboard_();
            ctrlmode_ = ECtrlMode::KEY_CTRL; ///< 键鼠控制
        }
        else ///< 其他情况均为遥控器控制
        {
            ControlFromRemote_();
            ctrlmode_ = ECtrlMode::RC_CTRL; ///< 遥控器控制
        }
        

    ResetFlag = SysBoardLink.otherInfo.ResetFlag;   // 此处获取上板发来的复位指令
    
    if (ResetFlag)
    {
        // RESET_SYSTEM();     // 上板判断跳变沿发来 每次按下只触发一次 因此这里不清除标志位
        ResetFlag = false;  // 清位 防止反复触发
    }

}

/**
 * @brief 心跳处理
 * 
 */
void CSystemCore::HeartbeatHandler_() {
    // 检查系统核心状态
    if (coreStatus == APP_RESET) return;

    EVarStatus lastRemoteState = false;
    auto currentRemoteState = SysBoardLink.ctrlInfos.remote_is_online;

    // 遥控器掉线
    if (currentRemoteState != APP_OK) {
        // 停止所有自动操作
        StopAutoCtrlTask_();
        
        // 停止所有模块（添加空指针检查）
        if (pchassis_) pchassis_->StopModule();
        
    }

    lastRemoteState = currentRemoteState;
}

// 软件复位
void CSystemCore::RESET_SYSTEM() {

    if (pchassis_) pchassis_->StopModule();

    // 给段延迟让电机收到停止指令
    static uint16_t resetCnt = 200;
    if (resetCnt-- != 0) {
        return ;
    }

    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

/**
 * @brief 启动自动任务
 * 
 * @retval EAppStatus
 */
EAppStatus CSystemCore::StartAutoCtrlTask_(EAutoCtrlProcess process) {

    if (currentAutoCtrlProcess_ != EAutoCtrlProcess::NONE)
        return APP_BUSY;

    StopAutoCtrlTask_();    ///< 停止当前任务

    return APP_OK;
}

/**
 * @brief 停止自动任务
 * 
 * @retval EAppStatus
 */
EAppStatus CSystemCore::StopAutoCtrlTask_() {
    if (autoCtrlTaskHandle_ == nullptr) return APP_ERROR;

    vTaskDelete(autoCtrlTaskHandle_);
    autoCtrlTaskHandle_ = nullptr;
    currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

    // 清除所有模块的自动控制标志（添加空指针检查）
    if (pchassis_) pchassis_->chassisCmd.isAutoCtrl = false;

    return APP_OK;
}

/**
 * @brief 更新板通系统层
 * 
 * @retval null
 */
void CSystemCore::BoardLink_Info_Update_(){
    // 检查系统核心状态
    if (coreStatus == APP_RESET) return;

    // 当前版本板间通信主要用于接收副板控制信息，此处暂不主动组包发送业务数据。
}


}   // namespace my_engineer
