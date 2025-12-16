/******************************************************************************
 * @brief        
 * 
 * @file         proc_energy_unit.cpp
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
 * @brief    抓取能量单元任务
 ******************************************************************************/
void CSystemCore::StartEnergyUnitTask(void *arg) {

    if (arg == nullptr) proc_return();

    // 获取SystemCore句柄
    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;
    auto cnt = 0;
    const auto timeout = 60000 / 5; // unit: ms

    core.parm_->armCmd.isAutoCtrl = true;           ///< 臂自动控制
    core.gimbal_auto_ctrl = true;					///< 云台自动控制
    
    /*Set Arm*/
    core.parm_->armCmd.set_angle_Yaw = GRAB_ENERGY_UNIT_YAW_ANGLE;
    core.parm_->armCmd.set_angle_Pitch1 = GRAB_ENERGY_UNIT_PITCH1_ANGLE;
    core.parm_->armCmd.set_angle_Pitch2 = GRAB_ENERGY_UNIT_PITCH2_ANGLE;
    core.parm_->armCmd.set_angle_Roll = GRAB_ENERGY_UNIT_ROLL_ANGLE;
    core.parm_->armCmd.set_angle_end_pitch = GRAB_ENERGY_UNIT_END_PITCH_ANGLE;
    core.parm_->armCmd.set_angle_end_roll = GRAB_ENERGY_UNIT_END_ROLL_ANGLE;
    core.parm_->armCmd.set_length_grip = GRAB_ENERGY_UNIT_GRIP_LENGTH;
    // 由于能量机关是轮盘状，因此此自动任务只将臂伸出并抬到固定位置，随后由操作手进行微操

// 退出
proc_exit:
    core.parm_->armCmd.isAutoCtrl = false;
    core.gimbal_auto_ctrl = false;
    core.autoCtrlTaskHandle_ = nullptr;
    core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
    core.movemode_ = EMoveMode::NONE;
    proc_return();

}

} // namespace my_engineer