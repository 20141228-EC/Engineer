/******************************************************************************
 * @brief        
 * 
 * @file         proc_ground_ore.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-03-19
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/**
 * @brief 创建地面矿石任务
 * 
 * @param argument 
 */
void CSystemCore::StartGroundOreTask(void *arg) {

	if (arg == nullptr) proc_return();

	// 获取SystemCore句柄
	auto &core = *reinterpret_cast<CSystemCore *>(arg);
	auto &keyboard = SysRemote.remoteInfo.keyboard;
	auto cnt = 0;
	const auto timeout = 60000 / 5; // unit: ms

	while (keyboard.key_Ctrl) {

		if (keyboard.mouse_L) {

			/*step 1*/
			core.pgimbal_->gimbalCmd.set_posit_lift = GIMBAL_LIFT_PHYSICAL_RANGE;    ///< Pull up the gimbal

			core.psubgantry_->subGantryCmd.setLiftPosit_L = SUB_GANTRY_LIFT_PHYSICAL_RANGE_L;
			core.psubgantry_->subGantryCmd.setLiftPosit_R = SUB_GANTRY_LIFT_PHYSICAL_RANGE_R;
			core.parm_->armCmd.set_angle_Yaw = 0.f;
			core.parm_->armCmd.set_angle_Pitch1 = 76.390f;
			core.parm_->armCmd.set_angle_Pitch2 = 43.207f;
			core.parm_->armCmd.set_angle_Roll = -1.480f;
			core.parm_->armCmd.set_angle_end_pitch = -60.815f;
			core.parm_->armCmd.set_angle_end_roll = 0.000f;

			core.psubgantry_->subGantryCmd.setPumpOn_Arm = true; ///< Set the arm pump on

			proc_waitMs(250);

			/* Wait for User Confirmation */
			cnt = timeout;
			core.parm_->armCmd.isAutoCtrl = false;
			core.psubgantry_->subGantryCmd.isAutoCtrl = false;
			core.pgimbal_->gimbalCmd.isAutoCtrl = false;      
			while (cnt--) {
				if (keyboard.key_Ctrl) {
					if (keyboard.mouse_L) break;
					if (keyboard.mouse_R) goto proc_exit;
				}
				proc_waitMs(5);
			}
			if (cnt == 0) goto proc_exit;

			/*step 2*/
			core.parm_->armCmd.isAutoCtrl = true;
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.pgimbal_->gimbalCmd.isAutoCtrl = true;
			
			core.parm_->armCmd.set_angle_Yaw = 0.f;
			core.parm_->armCmd.set_angle_Pitch1 = 90.0f;
			core.parm_->armCmd.set_angle_Pitch2 = 45.0f;
			core.parm_->armCmd.set_angle_Roll = -1.480f;
			core.parm_->armCmd.set_angle_end_pitch = -58.180f;
			core.parm_->armCmd.set_angle_end_roll = 0.000f;
			proc_waitMs(300);

			do {
				proc_waitMs(1);
				if(core.parm_->armInfo.angle_Pitch1 >= 59.f)
					core.parm_->armCmd.set_angle_Pitch1 -= 30.0f / core.freq;
				if(core.parm_->armInfo.angle_Pitch2 <= 58.298f)
					core.parm_->armCmd.set_angle_Pitch2 += 12.0f / core.freq;
				if(core.parm_->armInfo.angle_end_pitch >= -93.75f)
					core.parm_->armCmd.set_angle_end_pitch -= 30.0f / core.freq;
			} while (core.parm_->armCmd.set_angle_Pitch1 >= 59.f 
				&& core.parm_->armCmd.set_angle_Pitch2 <= 58.298f
				&& core.parm_->armCmd.set_angle_end_pitch >= -93.75f);
			core.parm_->armCmd.set_angle_Pitch1 = 59.f;
			core.parm_->armCmd.set_angle_Pitch2 = 58.298f;
			core.parm_->armCmd.set_angle_end_pitch = -98.750f;
      
      		goto proc_exit;
		}

		if (keyboard.mouse_R) {

			/*step 1*/
			core.pgimbal_->gimbalCmd.set_posit_lift = GIMBAL_LIFT_PHYSICAL_RANGE;    ///< Pull up the gimbal

			core.psubgantry_->subGantryCmd.setLiftPosit_L = SUB_GANTRY_LIFT_PHYSICAL_RANGE_L;
			core.psubgantry_->subGantryCmd.setLiftPosit_R = SUB_GANTRY_LIFT_PHYSICAL_RANGE_R;
			core.parm_->armCmd.set_angle_Yaw = 0.f;
			core.parm_->armCmd.set_angle_Pitch1 = 76.390f;
			core.parm_->armCmd.set_angle_Pitch2 = 43.207f;
			core.parm_->armCmd.set_angle_Roll = -1.480f;
			core.parm_->armCmd.set_angle_end_pitch = -60.815f;
			core.parm_->armCmd.set_angle_end_roll = 0.000f;

			core.psubgantry_->subGantryCmd.setPumpOn_Arm = true; ///< Set the arm pump on

			proc_waitMs(250);

			/* Wait for User Confirmation */
			cnt = timeout;
			core.parm_->armCmd.isAutoCtrl = false;
			core.psubgantry_->subGantryCmd.isAutoCtrl = false;
			core.pgimbal_->gimbalCmd.isAutoCtrl = false;      
			while (cnt--) {
				if (keyboard.key_Ctrl) {
					if (keyboard.mouse_L) break;
					if (keyboard.mouse_R) goto proc_exit;
				}
				proc_waitMs(5);
			}
			if (cnt == 0) goto proc_exit;

			/*Step 2 */
			core.parm_->armCmd.isAutoCtrl = true;
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.pgimbal_->gimbalCmd.isAutoCtrl = true;
			
			core.parm_->armCmd.set_angle_Yaw = 0.f;
			core.parm_->armCmd.set_angle_Pitch1 = 90.0f;
			core.parm_->armCmd.set_angle_Pitch2 = 45.0f;
			core.parm_->armCmd.set_angle_Roll = -1.480f;
			core.parm_->armCmd.set_angle_end_pitch = -60.180f;
			core.parm_->armCmd.set_angle_end_roll = 0.000f;
			proc_waitMs(300);

			do {
				proc_waitMs(1);
				if(core.parm_->armInfo.angle_Pitch1 >= 71.260f)
					core.parm_->armCmd.set_angle_Pitch1 -= 30.0f / core.freq;
				if(core.parm_->armInfo.angle_Pitch2 <= 55.405f)
					core.parm_->armCmd.set_angle_Pitch2 += 12.0f / core.freq;
				if(core.parm_->armInfo.angle_end_pitch >= -83.705f)
					core.parm_->armCmd.set_angle_end_pitch -= 30.0f / core.freq;
			} while (core.parm_->armCmd.set_angle_Pitch1 >= 71.260f 
				&& core.parm_->armCmd.set_angle_Pitch2 <= 55.405f
				&& core.parm_->armCmd.set_angle_end_pitch >= -83.705f);
			core.parm_->armCmd.set_angle_Pitch1 = 71.260f;
			core.parm_->armCmd.set_angle_Pitch2 = 55.405f;
			core.parm_->armCmd.set_angle_end_pitch = -83.705f;

			core.parm_->armCmd.set_angle_Roll = 79.139f;
			core.parm_->armCmd.set_angle_end_roll -= 28.800f;
			proc_waitMs(300);

			core.parm_->armCmd.set_angle_Pitch1 = 108.766f;
			core.parm_->armCmd.set_angle_Pitch2 = 47.564f;
			core.parm_->armCmd.set_angle_end_pitch = -121.835f;
			proc_waitMs(500);
			
			core.psubgantry_->subGantryCmd.setPumpOn_Arm = false; ///< Set the arm pump off
			core.parm_->armCmd.set_angle_Yaw -= 25.0f;
			proc_waitMs(500);

			/* Step 3 */
			core.psubgantry_->subGantryCmd.setLiftPosit_L = SUB_GANTRY_LIFT_PHYSICAL_RANGE_L;
			core.psubgantry_->subGantryCmd.setLiftPosit_R = SUB_GANTRY_LIFT_PHYSICAL_RANGE_R;
			core.parm_->armCmd.set_angle_Pitch1 = 76.390f;
			core.parm_->armCmd.set_angle_Pitch2 = 43.207f;
			core.parm_->armCmd.set_angle_Roll = -1.480f;
			core.parm_->armCmd.set_angle_end_pitch = -71.815f;
			core.parm_->armCmd.set_angle_end_roll = 0.000f;

			proc_waitMs(300);
			core.parm_->armCmd.set_angle_Yaw = 0.f;

			core.psubgantry_->subGantryCmd.setPumpOn_Arm = true; ///< Set the arm pump on

			proc_waitMs(250);

			/* Wait for User Confirmation */
			cnt = timeout;
			core.parm_->armCmd.isAutoCtrl = false;
			core.psubgantry_->subGantryCmd.isAutoCtrl = false;
			core.pgimbal_->gimbalCmd.isAutoCtrl = false;      
			while (cnt--) {
				if (keyboard.key_Ctrl) {
					if (keyboard.mouse_L) break;
					if (keyboard.mouse_R) goto proc_exit;
				}
				proc_waitMs(5);
			}
			if (cnt == 0) goto proc_exit;

			core.parm_->armCmd.isAutoCtrl = true;
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.pgimbal_->gimbalCmd.isAutoCtrl = true;
			
			core.parm_->armCmd.set_angle_Yaw = 0.f;
			core.parm_->armCmd.set_angle_Pitch1 = 90.526f;
			core.parm_->armCmd.set_angle_Pitch2 = 44.491f;
			core.parm_->armCmd.set_angle_Roll = -1.480f;
			core.parm_->armCmd.set_angle_end_pitch = -53.980f;
			core.parm_->armCmd.set_angle_end_roll = 0.000f;
			proc_waitMs(300);

			do {
				proc_waitMs(1);
				if(core.parm_->armInfo.angle_Pitch1 >= 59.f)
					core.parm_->armCmd.set_angle_Pitch1 -= 30.0f / core.freq;
				if(core.parm_->armInfo.angle_Pitch2 <= 58.298f)
					core.parm_->armCmd.set_angle_Pitch2 += 12.0f / core.freq;
				if(core.parm_->armInfo.angle_end_pitch >= -93.75f)
					core.parm_->armCmd.set_angle_end_pitch -= 30.0f / core.freq;
			} while (core.parm_->armCmd.set_angle_Pitch1 >= 59.f 
				&& core.parm_->armCmd.set_angle_Pitch2 <= 58.298f
				&& core.parm_->armCmd.set_angle_end_pitch >= -93.75f);
			core.parm_->armCmd.set_angle_Pitch1 = 59.f;
			core.parm_->armCmd.set_angle_Pitch2 = 58.298f;
			core.parm_->armCmd.set_angle_end_pitch = -98.750f;

			goto proc_exit;
		}

		proc_waitMs(5);

	}

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
