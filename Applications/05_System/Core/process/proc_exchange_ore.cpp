/******************************************************************************
 * @brief        
 * 
 * @file         proc_exchange_ore.cpp
 * @author       sllllr
 * @version      V1.0
 * @date         2025-12-15
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    兑换矿石任务
 ******************************************************************************/
void CSystemCore::StartExchangeOreTask(void *arg) {

    if (arg == nullptr) proc_return();

    // 获取SystemCore句柄
    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;
    auto cnt = 0;
    const auto timeout = 60000 / 5; // unit: ms

    core.parm_->armCmd.isAutoCtrl = true;           ///< 臂自动控制
    core.pchassis_->chassisCmd.isAutoCtrl = true;   ///< 底盘自动控制
    core.gimbal_auto_ctrl = true;					///< 云台自动控制
    
    /*Set Arm*/
    core.parm_->armCmd.set_angle_Yaw = EXCHANGE_ORE_YAW_ANGLE;
    core.parm_->armCmd.set_angle_Pitch1 = EXCHANGE_ORE_PITCH1_ANGLE;
    core.parm_->armCmd.set_angle_Pitch2 = EXCHANGE_ORE_PITCH2_ANGLE;
    core.parm_->armCmd.set_angle_Roll = EXCHANGE_ORE_ROLL_ANGLE;
    core.parm_->armCmd.set_angle_end_pitch = EXCHANGE_ORE_END_PITCH_ANGLE;
    core.parm_->armCmd.set_angle_end_roll = EXCHANGE_ORE_END_ROLL_ANGLE;
    core.parm_->armCmd.set_length_grip = EXCHANGE_ORE_GRIP_LENGTH;

// 退出
proc_exit:
    core.parm_->armCmd.isAutoCtrl = false;
    core.pchassis_->chassisCmd.isAutoCtrl = false;
    core.gimbal_auto_ctrl = false;
    core.autoCtrlTaskHandle_ = nullptr;
    core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
    core.movemode_ = EMoveMode::NONE;
    proc_return();

}

} // namespace my_engineer