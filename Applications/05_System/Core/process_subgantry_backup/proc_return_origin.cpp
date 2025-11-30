/******************************************************************************
 * @brief        
 * 
 * @file         proc_return_origin.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-05-11
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    返回原点任务
 ******************************************************************************/
void CSystemCore::StartReturnOriginTask(void *arg) {

	if (arg == nullptr) proc_return();

	/* Get System Core Handle */
	auto &core = *reinterpret_cast<CSystemCore *>(arg);
	const auto timeout = 60000 / 5;    // unit: ms
	auto cnt = 0;

	core.parm_->armCmd.isAutoCtrl = true;
	core.psubgantry_->subGantryCmd.isAutoCtrl = true;
	core.pgimbal_->gimbalCmd.isAutoCtrl = true;

	/* Set Auto Control Flag */
	core.psubgantry_->subGantryCmd.isAutoCtrl = true;
	core.parm_->armCmd.isAutoCtrl = true;
	// core.pgimbal_->gimbalCmd.isAutoCtrl = true;
	
	/*Set SubGantry*/
	core.psubgantry_->subGantryCmd.setStretchPosit_L = 0.0f;
	core.psubgantry_->subGantryCmd.setLiftPosit_L = 0.0f;
	core.psubgantry_->subGantryCmd.setStretchPosit_R = 0.0f;
	core.psubgantry_->subGantryCmd.setLiftPosit_R = 0.0f;

	/*Set Arm*/
	core.parm_->armCmd.set_angle_Yaw = 0.0f;
	core.parm_->armCmd.set_angle_Pitch1 = 21.0f;
	core.parm_->armCmd.set_angle_Pitch2 = 28.0f;
	core.parm_->armCmd.set_angle_Roll = 0.0f;
	core.parm_->armCmd.set_angle_end_pitch = -70.0f;
	core.parm_->armCmd.set_angle_end_roll = 0.0f;
	proc_waitMs(250);

	/*Set Gimbal*/
	core.pgimbal_->gimbalCmd.set_posit_lift = GIMBAL_LIFT_PHYSICAL_RANGE;  ///< Pull up the gimbal

/* Process Exit */
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
