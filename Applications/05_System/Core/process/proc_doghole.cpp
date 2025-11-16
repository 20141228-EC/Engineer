/******************************************************************************
 * @brief        
 * 
 * @file         proc_doghole.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-07-10
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    狗洞任务
 ******************************************************************************/
void CSystemCore::StartDogHoleTask(void *arg) {

	if (arg == nullptr) proc_return();

	// 获取SystemCore句柄
	auto &core = *reinterpret_cast<CSystemCore *>(arg);
	auto &keyboard = SysRemote.remoteInfo.keyboard;
	auto cnt = 0;
	const auto timeout = 60000 / 5; // unit: ms

	core.parm_->armCmd.isAutoCtrl = true;
	core.psubgantry_->subGantryCmd.isAutoCtrl = true;
	core.pgimbal_->gimbalCmd.isAutoCtrl = true;
	
	/*Set SubGantry*/
	core.psubgantry_->subGantryCmd.setStretchPosit_L = 0.0f;
	core.psubgantry_->subGantryCmd.setLiftPosit_L = 0.0f;
	core.psubgantry_->subGantryCmd.setStretchPosit_R = 0.0f;
	core.psubgantry_->subGantryCmd.setLiftPosit_R = 0.0f;
	
	/*Set Arm*/
	core.parm_->armCmd.set_angle_Yaw = 0.0f;
	core.parm_->armCmd.set_angle_Pitch1 = 4.000f;
	core.parm_->armCmd.set_angle_Pitch2 = 11.000f;
	core.parm_->armCmd.set_angle_Roll = -2.527f;
	core.parm_->armCmd.set_angle_end_pitch = -60.270f;
	core.parm_->armCmd.set_angle_end_roll = 0.000f;


	/*Set Gimbal*/
	core.pgimbal_->gimbalCmd.set_posit_lift = 0.0f;  ///< Pull down the gimbal

	proc_waitMs(250);

// 退出
proc_exit:
	core.parm_->armCmd.isAutoCtrl = false;
	core.psubgantry_->subGantryCmd.isAutoCtrl = false;
	core.pchassis_->chassisCmd.isAutoCtrl = false;
	core.pgimbal_->gimbalCmd.isAutoCtrl = false;
	core.autoCtrlTaskHandle_ = nullptr;
	core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
	proc_return();

}

} // namespace my_engineer
