/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-05-14 01:29:28
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-16 22:44:04
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

    // 启动BMI088设备
    DeviceIDMap.at(EDeviceID::DEV_MEMS_BMI088)->StartDevice();

    // 初始化所有系统
    CSystemControllerLink::SSystemInitParam_ControllerLink controllerLinkInitParam;
    controllerLinkInitParam.systemID = ESystemID::SYS_CONTROLLERLINK;
    controllerLinkInitParam.controllerLinkDevID = EDeviceID::DEV_CONTROLLER_LINK;
    SysControllerLink.InitSystem(&controllerLinkInitParam);


    // 获取模块的指针
    pcontroller_ = reinterpret_cast<CModController *>(ModuleIDMap.at(EModuleID::MOD_CONTROLLER));
    proc_waitMs(1000); // 等待系统初始化完成

    coreStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 更新处理
 * 
 */
void CSystemCore::UpdateHandler_() {
    
    if(!pcontroller_->ControllerInfo.isModuleAvailable 
        && pcontroller_->moduleStatus == APP_OK) {
        pcontroller_->StartModule();
    }

    SysControllerLink.controllerInfo.controller_OK = pcontroller_->ControllerInfo.isModuleAvailable;
    SysControllerLink.controllerInfo.return_success = pcontroller_->ControllerInfo.isReturnSuccess;
    // SysControllerLink.controllerInfo.Rocker_X = pcontroller_->ControllerInfo.rocker_X;
    // SysControllerLink.controllerInfo.Rocker_Y = pcontroller_->ControllerInfo.rocker_Y;
    // SysControllerLink.controllerInfo.Rocker_Key = static_cast<CSystemControllerLink::KEY_STATUS>(pcontroller_->ControllerInfo.rocker_Key);
    SysControllerLink.controllerInfo.angle_yaw = pcontroller_->ControllerInfo.posit_yaw;
    SysControllerLink.controllerInfo.angle_pitch1 = pcontroller_->ControllerInfo.posit_pitch1;
    SysControllerLink.controllerInfo.angle_pitch2 = pcontroller_->ControllerInfo.posit_pitch2;
    SysControllerLink.controllerInfo.angle_roll = pcontroller_->ControllerInfo.posit_roll;
    SysControllerLink.controllerInfo.angle_pitch_end = pcontroller_->ControllerInfo.posit_pitch_end;

    if (SysControllerLink.robotInfo.ask_reset_flag) {
        // 复位整个系统
        RESET_SYSTEM();
    }
    pcontroller_->ControllerCmd.StartControl = SysControllerLink.robotInfo.controlled_by_controller;
    pcontroller_->ControllerCmd.cmd_pitch1 = SysControllerLink.robotInfo.angle_pitch1;
    pcontroller_->ControllerCmd.cmd_pitch2 = SysControllerLink.robotInfo.angle_pitch2;
    pcontroller_->ControllerCmd.cmd_roll = SysControllerLink.robotInfo.angle_roll;
    pcontroller_->ControllerCmd.cmd_yaw = SysControllerLink.robotInfo.angle_yaw;
    pcontroller_->ControllerCmd.cmd_pitch_end = SysControllerLink.robotInfo.angle_pitch_end;

}

/**
 * @brief 心跳处理
 * 
 */
void CSystemCore::HeartbeatHandler_() {
    
}

// 复位
void CSystemCore::RESET_SYSTEM() {

    
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

}   // namespace my_engineer
