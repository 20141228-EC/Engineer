/******************************************************************************
 * @brief        
 * 
 * @file         proc_return_origin.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-15
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    模块全部复位任务
 ******************************************************************************/
void CSystemCore::StartReturnOriginTask(void *arg) {

	if (arg == nullptr) proc_return();

	// 获取SystemCore句柄
	auto &core = *reinterpret_cast<CSystemCore *>(arg);
	auto &keyboard = SysRemote.remoteInfo.keyboard;
	auto cnt = 0;
	const auto timeout = 10000 / 5; // unit: ms

	core.parm_->armCmd.isAutoCtrl = true;           ///< 臂自动控制
	core.pgimbal_->gimbalCmd.isAutoCtrl = true;		///< 云台自动控制
	
	/*Set Arm*/
	core.parm_->armCmd.set_angle_Yaw = RETURN_ORIGIN_YAW_ANGLE;
	core.parm_->armCmd.set_angle_Pitch1 = RETURN_ORIGIN_PITCH1_ANGLE;
	core.parm_->armCmd.set_angle_Pitch2 = RETURN_ORIGIN_PITCH2_ANGLE;
	core.parm_->armCmd.set_angle_Roll = RETURN_ORIGIN_ROLL_ANGLE;
	core.parm_->armCmd.set_angle_end_pitch = RETURN_ORIGIN_END_PITCH_ANGLE;
	core.parm_->armCmd.set_angle_end_roll = RETURN_ORIGIN_END_ROLL_ANGLE;
    core.parm_->armCmd.set_length_grip = RETURN_ORIGIN_GRIP_LENGTH;

	/* Set Gimbal  */
	core.pgimbal_->gimbalCmd.set_posit_yaw = GIMBAL_YAW_INIT_ANGLE;
    // 全部回到初始化位置

	proc_waitMs(250);

// 退出
proc_exit:
	core.parm_->armCmd.isAutoCtrl = false;
    core.pgimbal_->gimbalCmd.isAutoCtrl = false;
	core.autoCtrlTaskHandle_ = nullptr;
	core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
	core.movemode_ = EMoveMode::NONE;
	proc_return();

}

} // namespace my_engineer