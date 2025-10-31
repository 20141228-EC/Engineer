/******************************************************************************
 * @brief        
 * 
 * @file         proc_gold_ore.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-03-23
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    金矿石任务
 ******************************************************************************/
void CSystemCore::StartGoldOreTask(void *arg) {

	if (arg == nullptr) proc_return();

	// 获取SystemCore句柄
	auto &core = *reinterpret_cast<CSystemCore *>(arg);
	auto &keyboard = SysRemote.remoteInfo.keyboard;
	auto cnt = 0;
	const auto timeout = 60000 / 5; // unit: ms

	core.parm_->armCmd.isAutoCtrl = true;
	core.psubgantry_->subGantryCmd.isAutoCtrl = true;
	core.pgimbal_->gimbalCmd.isAutoCtrl = true;

	static float_t subgantry_stretch_L = 266.0f;
	static float_t subgantry_stretch_R = 322.0f;
	static float_t subgantry_lift_L = 46.0f;
	static float_t subgantry_lift_R = 50.0f;

	core.pgimbal_->gimbalCmd.set_posit_lift = 100.0f;

	core.parm_->armCmd.set_angle_Yaw = -0.195f;
	core.parm_->armCmd.set_angle_Pitch1 = 24.976f;
	core.parm_->armCmd.set_angle_Pitch2 = 51.125f;
	core.parm_->armCmd.set_angle_Roll = 180.000f;
	core.parm_->armCmd.set_angle_end_pitch = -145.000f;
	core.parm_->armCmd.set_angle_end_roll = 0.000f;

	while (keyboard.key_Ctrl) {
		// 用刺雷
		if (keyboard.mouse_L) {

			/*Step 1*/
			core.psubgantry_->subGantryCmd.setLiftPosit_L = 0.0f;
			core.psubgantry_->subGantryCmd.setLiftPosit_R = 0.0f;
			core.psubgantry_->subGantryCmd.setStretchPosit_L = 100.0f;
			core.psubgantry_->subGantryCmd.setStretchPosit_R = 100.0f;
			core.psubgantry_->subGantryCmd.setPumpOn_Left = false;
			core.psubgantry_->subGantryCmd.setPumpOn_Right = false;

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

			/*Step 2*/
			core.parm_->armCmd.isAutoCtrl = true;
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.pgimbal_->gimbalCmd.isAutoCtrl = true;

			core.psubgantry_->subGantryCmd.setStretchPosit_L = subgantry_stretch_L;
			core.psubgantry_->subGantryCmd.setStretchPosit_R = subgantry_stretch_R;
			core.psubgantry_->subGantryCmd.setPumpOn_Left = true;
			core.psubgantry_->subGantryCmd.setPumpOn_Right = true;
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

			/*Step 3*/
			core.parm_->armCmd.isAutoCtrl = true;
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.pgimbal_->gimbalCmd.isAutoCtrl = true;

			core.psubgantry_->subGantryCmd.setLiftPosit_L = subgantry_lift_L;
			core.psubgantry_->subGantryCmd.setLiftPosit_R = subgantry_lift_R;
			proc_waitUntilWithTimeout(
				(core.psubgantry_->subGantryInfo.isPositArrived_Lift_L &&
				 core.psubgantry_->subGantryInfo.isPositArrived_Lift_R),
				3000
			);
			proc_waitMs(500);

			core.psubgantry_->subGantryCmd.setStretchPosit_L = 0.0f;
			core.psubgantry_->subGantryCmd.setStretchPosit_R = 0.0f;
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

			/*Step 4*/
			core.parm_->armCmd.set_angle_Yaw = -0.585f;
			core.parm_->armCmd.set_angle_Pitch1 = 20.644f;
			core.parm_->armCmd.set_angle_Pitch2 = 29.297f;
			core.parm_->armCmd.set_angle_Roll = -2.178f;
			core.parm_->armCmd.set_angle_end_pitch = -60.745f;
			core.parm_->armCmd.set_angle_end_roll = -0.720f;
			proc_waitMs(250);

			goto proc_exit;
		}
		
		// 用机械臂
		if (keyboard.mouse_R) {
			/*Step 1*/
			core.parm_->armCmd.set_angle_Yaw = -0.585f;
			core.parm_->armCmd.set_angle_Pitch1 = 88.868f;
			core.parm_->armCmd.set_angle_Pitch2 = 45.878f;
			core.parm_->armCmd.set_angle_Roll = 180.000f;
			core.parm_->armCmd.set_angle_end_pitch = -56.570f;
			core.parm_->armCmd.set_angle_end_roll = -1.440f;

			core.psubgantry_->subGantryCmd.setLiftPosit_L = SUB_GANTRY_LIFT_PHYSICAL_RANGE_L;
			core.psubgantry_->subGantryCmd.setLiftPosit_R = SUB_GANTRY_LIFT_PHYSICAL_RANGE_R;
			core.psubgantry_->subGantryCmd.setStretchPosit_L = 0.0f;
			core.psubgantry_->subGantryCmd.setStretchPosit_R = 0.0f;

			core.psubgantry_->subGantryCmd.setPumpOn_Arm = true;

			proc_waitMs(250);

			/* 选择 0 或 15度*/
			cnt = timeout;
			core.parm_->armCmd.isAutoCtrl = false;
			core.psubgantry_->subGantryCmd.isAutoCtrl = false;
			core.pgimbal_->gimbalCmd.isAutoCtrl = false;
			while (cnt--) {
				if (keyboard.key_Ctrl) {
					if (keyboard.mouse_L) {
						core.parm_->armCmd.set_angle_Yaw = 13.5f;
						break;
					}
					else if (keyboard.mouse_R) {
						core.parm_->armCmd.set_angle_Yaw = 0.0f;
						break;
					}
				}
				proc_waitMs(5);
			}
			if (cnt == 0) goto proc_exit;

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

			/*Step 2*/
			core.parm_->armCmd.isAutoCtrl = true;
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.pgimbal_->gimbalCmd.isAutoCtrl = true;

			core.psubgantry_->subGantryCmd.setPumpOn_Arm = true;
			proc_waitMs(750);

			do {
				proc_waitMs(1);
				if (core.parm_->armCmd.set_angle_Pitch1 > 73.214f) {
					core.parm_->armCmd.set_angle_Pitch1 -= 14.0 / core.freq; 
				}
				if (core.parm_->armCmd.set_angle_Pitch2 > 43.240f) {
					core.parm_->armCmd.set_angle_Pitch2 -= 14.0 / core.freq;
				}
				if (core.parm_->armCmd.set_angle_end_pitch > -62.0f) {
					core.parm_->armCmd.set_angle_end_pitch -= 14.0 / core.freq;
				}
			}while (core.parm_->armCmd.set_angle_Pitch1 > 73.214f ||
				core.parm_->armCmd.set_angle_Pitch2 > 43.240f ||
				core.parm_->armCmd.set_angle_end_pitch > -62.0f);

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

			do {
				proc_waitMs(1);
				core.parm_->armCmd.set_angle_Roll -= 360.0f / core.freq;
			} while (core.parm_->armCmd.set_angle_Roll > 0.f);

			/*Step 3*/
			core.parm_->armCmd.isAutoCtrl = true;
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.pgimbal_->gimbalCmd.isAutoCtrl = true;

			core.parm_->armCmd.set_angle_Yaw = -0.585f;
			core.parm_->armCmd.set_angle_Pitch1 = 40.644f;
			core.parm_->armCmd.set_angle_Pitch2 = 59.297f;
			core.parm_->armCmd.set_angle_Roll = 0.0f;
			core.parm_->armCmd.set_angle_end_pitch = -60.745f;
			core.parm_->armCmd.set_angle_end_roll = -0.720f;

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
