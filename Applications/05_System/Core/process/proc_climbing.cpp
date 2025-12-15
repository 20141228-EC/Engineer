/******************************************************************************
 * @brief        
 * 
 * @file         proc_climbing.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-14
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    上台阶任务
 ******************************************************************************/
void CSystemCore::StartClimbingTask(void *arg) {

	if (arg == nullptr) proc_return();

	// 获取SystemCore句柄
	auto &core = *reinterpret_cast<CSystemCore *>(arg);
	auto &keyboard = SysRemote.remoteInfo.keyboard;
	auto cnt = 0;
	const auto timeout = 60000 / 5; // unit: ms

	core.parm_->armCmd.isAutoCtrl = true;           ///< 臂自动控制
	core.pchassis_->chassisCmd.isAutoCtrl = true;   ///< 底盘自动控制
	core.gimbal_auto_ctrl = true;					///< 云台自动控制
	core.movemode_ = EMoveMode::CLIMBING;
	core.pchassis_->MovMode = CModChassis::EmovMode::CLIMBING;
	
	/*Set Arm*/
	core.parm_->armCmd.set_angle_Yaw = CLIMBING_YAW_ANGLE;
	core.parm_->armCmd.set_angle_Pitch1 = CLIMBING_PITCH1_ANGLE;
	core.parm_->armCmd.set_angle_Pitch2 = CLIMBING_PITCH2_ANGLE;
	core.parm_->armCmd.set_angle_Roll = CLIMBING_ROLL_ANGLE;
	core.parm_->armCmd.set_angle_end_pitch = CLIMBING_END_PITCH_ANGLE;
	core.parm_->armCmd.set_angle_end_roll = CLIMBING_END_ROLL_ANGLE;
    core.parm_->armCmd.set_length_grip = CLIMBING_GRIP_LENGTH;

	/*Set Chassis*/
	core.pchassis_->chassisCmd.speed_X = CLIMBING_SPEED;

	// 上台阶任务比较特殊，由操作手来决定何时退出任务
    while (true)
    {
        if (keyboard.key_V) // 按下v来退出任务
        {
            break; // 检测到v键按下，跳出循环
        }
        proc_waitMs(20);
    }

// 退出
proc_exit:
	core.parm_->armCmd.isAutoCtrl = false;
	core.pchassis_->chassisCmd.isAutoCtrl = false;
	core.gimbal_auto_ctrl = false;
	core.autoCtrlTaskHandle_ = nullptr;
	core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
	core.movemode_ = EMoveMode::NONE;
	core.pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
	proc_return();

}

} // namespace my_engineer
