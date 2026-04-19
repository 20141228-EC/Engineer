/******************************************************************************
 * @brief        
 * 
 * @file         proc_downstair.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2026-03-22
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    下台阶任务
 ******************************************************************************/
void CSystemCore::StartDownStairTask(void *arg) {

	if (arg == nullptr) proc_return();

	// 获取SystemCore句柄
	auto &core = *reinterpret_cast<CSystemCore *>(arg);
	auto &keyboard = SysRemote.remoteInfo.keyboard;
	auto cnt = 0;
	const auto timeout = 30000 / 5; // unit: ms

	core.parm_->armCmd.isAutoCtrl = true;           ///< 臂自动控制
	core.pchassis_->chassisCmd.isAutoCtrl = true;   ///< 底盘自动控制
	core.gimbal_auto_ctrl = true;					///< 云台自动控制

    core.movemode_ = EMoveMode::DOWNSTAIR;  // 更新系统层标志位
	core.pchassis_->MovMode = CModChassis::EmovMode::DOWNSTAIR;     // 更新模块层标志位
	
	/*Set Arm*/
	core.parm_->armCmd.set_angle_Yaw = DOWNSTAIR_YAW_ANGLE;
	core.parm_->armCmd.set_angle_Pitch1 = DOWNSTAIR_PITCH1_ANGLE;
	core.parm_->armCmd.set_angle_Pitch2 = DOWNSTAIR_PITCH2_ANGLE;
	core.parm_->armCmd.set_angle_Roll = DOWNSTAIR_ROLL_ANGLE;
	core.parm_->armCmd.set_angle_end_pitch = DOWNSTAIR_END_PITCH_ANGLE;
	core.parm_->armCmd.set_angle_end_roll = DOWNSTAIR_END_ROLL_ANGLE;
    core.parm_->armCmd.set_length_grip = DOWNSTAIR_GRIP_LENGTH;
	// 后续看情况得改 在初始化位置可能会干涉

	/*Set Chassis*/
	// core.pchassis_->chassisCmd.L_length = DOWNSTAIR_HIP_ANGLE;

	/*Set Gimbal*/
	// 在副板设置云台

	proc_waitMs(300);

	// 下台阶任务比较特殊，由操作手来决定何时退出任务
    while (keyboard.key_Ctrl)				///< 按住ctrl
    {
		if(core.pchassis_->chassisInfo.L_Length < 80.f){
			core.pchassis_->chassisCmd.L_length += 120.f / 1000.f;
		}
		else{
			core.pchassis_->chassisCmd.speed_Y = DOWNSTAIR_SPEED;	// 保持底盘速度
			// if(core.pchassis_->Leg_is_soar){    // 等待后腿腾空
			// 	core.pchassis_->chassisCmd.L_length -= 120.f / 1000.f;	// 收腿
			// }
			
		}
		proc_waitMs(20);
    }
	// 松开ctrl退出下台阶模式

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
