/**
 * @file control.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 在这里定义遥控器和键盘的操作函数
 * @version 1.0
 * @date 2025-12-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "Core.hpp"

// int16_t 
namespace my_engineer {

namespace {

static bool g_useBoardLinkChassis = true;

inline void ApplyBoardLinkChassisControl(CModChassis *chassis) {
    if (!chassis) {
        return;
    }

    const bool ctrlValid = (SysBoardLink.ctrlInfos.pack_id == CDevBoardLink::PKT_CTRL_INFOS)
        && (SysBoardLink.ctrlInfos.remote_is_online == 1);
    if (ctrlValid) {
        chassis->chassisCmd.speed_X = static_cast<float>(SysBoardLink.ctrlInfos.speed_x);
        chassis->chassisCmd.speed_Y = static_cast<float>(SysBoardLink.ctrlInfos.speed_y);
        chassis->chassisCmd.speed_W = static_cast<float>(SysBoardLink.ctrlInfos.speed_w);
    } else {
        chassis->chassisCmd.speed_X = 0.0f;
        chassis->chassisCmd.speed_Y = 0.0f;
        chassis->chassisCmd.speed_W = 0.0f;
    }
}

} // namespace

void CSystemCore::StartRobot(bool if_remote_control, bool I_dont_have_a_remote) {

    if (pchassis_) {
        if (!pchassis_->chassisInfo.isModuleAvailable               ///<说明模块已经注册了
            && pchassis_->moduleStatus == APP_OK) {
            pchassis_->StartModule();                               ///<在创建任务的时候还会再调用一次初始化函数
        }
    }
    
}

/**
 * @brief 遥控器操作
 * 
 */
void CSystemCore::ControlFromRemote_() {
    if (g_useBoardLinkChassis) {
        StartRobot(true);
        if (pchassis_ && !pchassis_->chassisCmd.isAutoCtrl) {
            ApplyBoardLinkChassisControl(pchassis_);
        }
        return;
    }
}

/**
 * @brief 键盘操作
 * 
 */
void CSystemCore::ControlFromKeyboard_() {
    if (g_useBoardLinkChassis) {
        if (SysRemote.systemStatus == APP_OK) {
            StartRobot(false);
        }
        if (pchassis_ && !pchassis_->chassisCmd.isAutoCtrl) {
            ApplyBoardLinkChassisControl(pchassis_);
        }
        return;
    }

}

/**
 * @brief 自定义控制器操作
 */
void CSystemCore::ControlFromController_() {
    if (g_useBoardLinkChassis) {
        if (pchassis_ && !pchassis_->chassisCmd.isAutoCtrl) {
            ApplyBoardLinkChassisControl(pchassis_);
        }
        return;
    }
}

void CSystemCore::ControlFromEsp32_() {
    if (g_useBoardLinkChassis) {
        if (pchassis_ && !pchassis_->chassisCmd.isAutoCtrl) {
            ApplyBoardLinkChassisControl(pchassis_);
        }
        return;
    }
}


}   // namespace my_engineer
