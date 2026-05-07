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

/*浮点数线性映射成整数*/
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits
    ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/*整数线性映射成浮点数*/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void ApplyBoardLinkChassisControl(CModChassis *chassis) {
    if (!chassis) {
        return;
    }

    const bool ctrlValid = (SysBoardLink.ctrlInfos.pack_id == CDevBoardLink::PKT_CTRL_INFOS)
        && (SysBoardLink.ctrlInfos.remote_is_online == 1);
    if (ctrlValid) {
        // chassis->chassisCmd.speed_X = uint_to_float(SysBoardLink.ctrlInfos.speed_x, -660, 660, 16);
        // chassis->chassisCmd.speed_Y = uint_to_float(SysBoardLink.ctrlInfos.speed_y, -660, 660, 16);
        // chassis->chassisCmd.speed_W = uint_to_float(SysBoardLink.ctrlInfos.speed_w, -660, 660, 16);
        chassis->chassisCmd.speed_X = static_cast<float_t>(SysBoardLink.ctrlInfos.speed_x);
        chassis->chassisCmd.speed_Y = static_cast<float_t>(SysBoardLink.ctrlInfos.speed_y);
        chassis->chassisCmd.speed_W = static_cast<float_t>(SysBoardLink.ctrlInfos.speed_w);
    } else {
        
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
