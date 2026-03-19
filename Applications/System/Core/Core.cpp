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

    // 左臂控制器
    auto it_left = ModuleIDMap.find(EModuleID::MOD_CONTROLLER_LEFT);
    if (it_left != ModuleIDMap.end() && it_left->second != nullptr) {
        pcontroller_left_ = reinterpret_cast<CModController *>(it_left->second);
    }
    // 右臂控制器
    auto it_right = ModuleIDMap.find(EModuleID::MOD_CONTROLLER_RIGHT);
    if (it_right != ModuleIDMap.end() && it_right->second != nullptr) {
        pcontroller_right_ = reinterpret_cast<CModController *>(it_right->second);
    }
    // 单控制器调试模式（DEBUG_TEST_ARM: 0=禁用, 1=左臂, 2=右臂）
#define DEBUG_TEST_ARM 0

#if DEBUG_TEST_ARM > 0
    auto it_debug = ModuleIDMap.find(EModuleID::MOD_CONTROLLER);
    if (it_debug != ModuleIDMap.end() && it_debug->second != nullptr) {
    #if DEBUG_TEST_ARM == 1
        pcontroller_left_ = reinterpret_cast<CModController *>(it_debug->second);
    #elif DEBUG_TEST_ARM == 2
        pcontroller_right_ = reinterpret_cast<CModController *>(it_debug->second);
    #endif
    }
#endif

    proc_waitMs(1000); // 等待系统初始化完成

    coreStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 更新处理
 * 
 */
void CSystemCore::UpdateHandler_() {

    /*------------左臂控制器---------*/
    if (pcontroller_left_) {
        // 启动模块
        if (!pcontroller_left_->ControllerInfo.isModuleAvailable
            && pcontroller_left_->moduleStatus == APP_OK) {
            pcontroller_left_->StartModule();
        }
        // 上传左臂数据 (控制器 -> 机器人)，pitch1/2减去零点标定偏移
        SysControllerLink.controllerInfo.left_arm.yaw = pcontroller_left_->ControllerInfo.posit_yaw;
        SysControllerLink.controllerInfo.left_arm.pitch1 = pcontroller_left_->ControllerInfo.posit_pitch1 - CONTROLLER_PITCH1_ZERO_OFFSET;
        SysControllerLink.controllerInfo.left_arm.pitch2 = pcontroller_left_->ControllerInfo.posit_pitch2 - CONTROLLER_PITCH2_ZERO_OFFSET;
        SysControllerLink.controllerInfo.left_arm.roll = pcontroller_left_->ControllerInfo.posit_roll;
        SysControllerLink.controllerInfo.left_arm.pitch_end = pcontroller_left_->ControllerInfo.posit_pitch_end;
        // 接收左臂数据 (机器人 -> 控制器)
        pcontroller_left_->ControllerCmd.StartControl = SysControllerLink.robotInfo.controlled_by_controller;
        pcontroller_left_->ControllerCmd.cmd_yaw = SysControllerLink.robotInfo.left_arm.yaw;
        pcontroller_left_->ControllerCmd.cmd_pitch1 = SysControllerLink.robotInfo.left_arm.pitch1;
        pcontroller_left_->ControllerCmd.cmd_pitch2 = SysControllerLink.robotInfo.left_arm.pitch2;
        pcontroller_left_->ControllerCmd.cmd_roll = SysControllerLink.robotInfo.left_arm.roll;
        pcontroller_left_->ControllerCmd.cmd_pitch_end = SysControllerLink.robotInfo.left_arm.pitch_end;
    }

    /*------------右臂控制器---------*/
    if (pcontroller_right_) {
        // 启动模块
        if (!pcontroller_right_->ControllerInfo.isModuleAvailable
            && pcontroller_right_->moduleStatus == APP_OK) {
            pcontroller_right_->StartModule();
        }
        // 上传右臂数据 (控制器 -> 机器人)，pitch1/2减去零点标定偏移
        SysControllerLink.controllerInfo.right_arm.yaw = pcontroller_right_->ControllerInfo.posit_yaw;
        SysControllerLink.controllerInfo.right_arm.pitch1 = pcontroller_right_->ControllerInfo.posit_pitch1 - CONTROLLER_PITCH1_ZERO_OFFSET;
        SysControllerLink.controllerInfo.right_arm.pitch2 = pcontroller_right_->ControllerInfo.posit_pitch2 - CONTROLLER_PITCH2_ZERO_OFFSET;
        SysControllerLink.controllerInfo.right_arm.roll = -pcontroller_right_->ControllerInfo.posit_roll;
        SysControllerLink.controllerInfo.right_arm.pitch_end = -pcontroller_right_->ControllerInfo.posit_pitch_end;
        // 接收右臂数据 (机器人 -> 控制器)
        pcontroller_right_->ControllerCmd.StartControl = SysControllerLink.robotInfo.controlled_by_controller;
        pcontroller_right_->ControllerCmd.cmd_yaw = SysControllerLink.robotInfo.right_arm.yaw;
        pcontroller_right_->ControllerCmd.cmd_pitch1 = SysControllerLink.robotInfo.right_arm.pitch1;
        pcontroller_right_->ControllerCmd.cmd_pitch2 = SysControllerLink.robotInfo.right_arm.pitch2;
        pcontroller_right_->ControllerCmd.cmd_roll = SysControllerLink.robotInfo.right_arm.roll;
        pcontroller_right_->ControllerCmd.cmd_pitch_end = SysControllerLink.robotInfo.right_arm.pitch_end;
    }

    /*------------状态汇总------------*/
    // 只有两个控制器都可用时才认为整体OK
    bool left_ok = pcontroller_left_ ? pcontroller_left_->ControllerInfo.isModuleAvailable : true;
    bool right_ok = pcontroller_right_ ? pcontroller_right_->ControllerInfo.isModuleAvailable : true;
    SysControllerLink.controllerInfo.controller_OK = left_ok && right_ok;

    // 归位成功状态
    bool left_return = pcontroller_left_ ? pcontroller_left_->ControllerInfo.isReturnSuccess : true;
    bool right_return = pcontroller_right_ ? pcontroller_right_->ControllerInfo.isReturnSuccess : true;
    SysControllerLink.controllerInfo.return_success = left_return && right_return;

    // 复位检测
    if (SysControllerLink.robotInfo.ask_reset_flag) {
        RESET_SYSTEM();
    }
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
