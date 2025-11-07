/******************************************************************************
 * @brief        	
 * 
 * @file         proc_pop.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-05-10
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/**
 * @brief 创建POP任务
 * 
 * @param argument 
 */
void CSystemCore::StartPopOreTask(void *arg) {

	if (arg == nullptr) proc_return();

	// 获取SystemCore句柄
	auto &core = *reinterpret_cast<CSystemCore *>(arg);
	auto &keyboard = SysRemote.remoteInfo.keyboard;
	auto cnt = 0;
	const auto timeout = 60000 / 5; // unit: ms

	while (keyboard.key_Ctrl) {

		if (keyboard.mouse_L) {

			/*step 1*/
			core.psubgantry_->subGantryCmd.setStretchPosit_L = 0.0f;
			core.psubgantry_->subGantryCmd.setLiftPosit_L = SUB_GANTRY_LIFT_PHYSICAL_RANGE_L;
			proc_waitUntilWithTimeout(
				core.psubgantry_->subGantryInfo.isPositArrived_Lift_L == true, 5000
			);
			/*Set RoboticArm*/
			core.parm_->armCmd.set_angle_Yaw = -15.585f;
			core.parm_->armCmd.set_angle_Pitch1 = 51.766f;
			core.parm_->armCmd.set_angle_Pitch2 = 40.425f;
			core.parm_->armCmd.set_angle_Roll = 85.770f;
			core.parm_->armCmd.set_angle_end_pitch = -108.865f;
			core.parm_->armCmd.set_angle_end_roll = 14.400f;
			proc_waitMs(250);
			core.psubgantry_->subGantryCmd.setPumpOn_Arm = false;
			core.psubgantry_->subGantryCmd.setPumpOn_Left = true;

			/*step 2*/
			proc_waitUntilWithTimeout(
				(core.parm_->armInfo.isAngleArrived_Pitch1 == true &&
				core.parm_->armInfo.isAngleArrived_Pitch2 == true), 800
			);
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.parm_->armCmd.isAutoCtrl = true;
			// core.pgimbal_->gimbalCmd.isAutoCtrl = true;      

			core.psubgantry_->subGantryCmd.setPumpOn_Arm = true;
			core.psubgantry_->subGantryCmd.setPumpOn_Left = true;
			proc_waitMs(250);
			
			/*Set RoboticArm*/
			core.parm_->armCmd.set_angle_Yaw   = 20.0f;
			proc_waitMs(250);

			/* Wait for User Confirmation */
			cnt = timeout;
			core.psubgantry_->subGantryCmd.isAutoCtrl = false;
			core.parm_->armCmd.isAutoCtrl = false;
			core.pgimbal_->gimbalCmd.isAutoCtrl = false;
			while (cnt--) {
				if (keyboard.key_Ctrl) {
					if (keyboard.mouse_L) break;
					if (keyboard.mouse_R) goto proc_exit;
				}
				proc_waitMs(5);
			}
			if (cnt == 0) goto proc_exit;

			/*step 3*/
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.parm_->armCmd.isAutoCtrl = true;
			// core.pgimbal_->gimbalCmd.isAutoCtrl = true;

			core.psubgantry_->subGantryCmd.setPumpOn_Left = false;
			proc_waitMs(250);

			core.parm_->armCmd.set_angle_Yaw = 0.0f;
			proc_waitUntilWithTimeout(
				core.parm_->armInfo.isAngleArrived_Yaw == true, 800
			);
			core.parm_->armCmd.set_angle_Pitch1 += 10.0f;
			core.parm_->armCmd.set_angle_Pitch2 += 10.0f;
			proc_waitUntilWithTimeout(
				(core.parm_->armInfo.isAngleArrived_Pitch1 == true &&
				core.parm_->armInfo.isAngleArrived_Pitch2 == true), 800
			);
			core.parm_->armCmd.set_angle_Roll = 0.0f;
			core.parm_->armCmd.set_angle_end_pitch = 0.0f;
			core.parm_->armCmd.set_angle_end_roll -= 9.0f;

			proc_waitMs(250);
			core.psubgantry_->subGantryCmd.setStretchPosit_L = 0.0f;
			proc_waitMs(500);

			// core.use_Controller_ = true;

			goto proc_exit;
		}

		if (keyboard.mouse_R) {

			/*step 1*/
			core.psubgantry_->subGantryCmd.setStretchPosit_R = 0.0f;
			core.psubgantry_->subGantryCmd.setLiftPosit_R = SUB_GANTRY_LIFT_PHYSICAL_RANGE_R;
			proc_waitUntilWithTimeout(
				core.psubgantry_->subGantryInfo.isPositArrived_Lift_R == true, 5000
			);
			core.parm_->armCmd.set_angle_Yaw = 14.235f;
			core.parm_->armCmd.set_angle_Pitch1 = 49.600f;
			core.parm_->armCmd.set_angle_Pitch2 = 36.754f;
			core.parm_->armCmd.set_angle_Roll = -90.824f;
			core.parm_->armCmd.set_angle_end_pitch = -105.970f;
			core.parm_->armCmd.set_angle_end_roll = -0.720f;
			proc_waitMs(250);
			core.psubgantry_->subGantryCmd.setPumpOn_Arm = false; // 关闭机械臂气泵
			core.psubgantry_->subGantryCmd.setPumpOn_Right = true; // 启动右气泵

			/*step 2*/
			proc_waitUntilWithTimeout(
				(core.parm_->armInfo.isAngleArrived_Pitch1 == true &&
				core.parm_->armInfo.isAngleArrived_Pitch2 == true), 800
			);
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.parm_->armCmd.isAutoCtrl = true;
			core.psubgantry_->subGantryCmd.setPumpOn_Arm = true; // 启动机械臂气泵
			core.psubgantry_->subGantryCmd.setPumpOn_Right = true; // 启动右气泵
			proc_waitMs(250);
			
			core.parm_->armCmd.set_angle_Yaw = -16.0f;
			// core.parm_->armCmd.set_angle_Pitch1 += 5.0f;
			proc_waitMs(250);

			/* Wait for User Confirmation */
			cnt = timeout;
			core.parm_->armCmd.isAutoCtrl = false;      // Unlock Arm
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

			/*step 3*/
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.parm_->armCmd.isAutoCtrl = true;

			core.psubgantry_->subGantryCmd.setPumpOn_Right = false; // 关闭右气泵
			proc_waitMs(250);
			
			// core.psubgantry_->subGantryCmd.setStretchPosit_R = 0.0f;
			// proc_waitUntilWithTimeout(
			// 	core.psubgantry_->subGantryInfo.isPositArrived_Stretch_R == true, 5000
			// );
			core.parm_->armCmd.set_angle_Yaw = -0.585f;
			proc_waitUntilWithTimeout(
				core.parm_->armInfo.isAngleArrived_Yaw == true, 800
			);
			core.parm_->armCmd.set_angle_Pitch1 += 10.0f;
			core.parm_->armCmd.set_angle_Pitch2 += 20.0f;
			proc_waitUntilWithTimeout(
				(core.parm_->armInfo.isAngleArrived_Pitch1 == true &&
				core.parm_->armInfo.isAngleArrived_Pitch2 == true), 800
			);
			core.parm_->armCmd.set_angle_Roll = 0.0f;
			core.parm_->armCmd.set_angle_end_pitch = 0.0f;
			core.parm_->armCmd.set_angle_end_roll += 10.8f;

			proc_waitMs(250);

			// core.use_Controller_ = true;

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
		