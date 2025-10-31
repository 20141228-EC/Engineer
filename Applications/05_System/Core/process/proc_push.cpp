/******************************************************************************
 * @brief        
 * 
 * @file         proc_push.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-05-10
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    储矿
 ******************************************************************************/
void CSystemCore::StartPushOreTask(void *arg) {

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
			core.psubgantry_->subGantryCmd.setLiftPosit_L = 0.0f;
			proc_waitMs(250);

			// core.parm_->armCmd.set_angle_Pitch1 = 67.902f;
			// core.parm_->armCmd.set_angle_Pitch2 = 64.212f;
			// core.parm_->armCmd.set_angle_Roll = -1.829f;
			// core.parm_->armCmd.set_angle_end_pitch = -115.515f;
			// core.parm_->armCmd.set_angle_end_roll -= 36.640f;

			// proc_waitMs(250);
			// core.parm_->armCmd.set_angle_Yaw = 51.870f;

			core.parm_->armCmd.set_angle_Pitch1 = 62.938f;
			core.parm_->armCmd.set_angle_Pitch2 = 59.471f;
			core.parm_->armCmd.set_angle_Roll = -3.923f;
			core.parm_->armCmd.set_angle_end_pitch = -98.260f;
			core.parm_->armCmd.set_angle_end_roll -= 33.120f;

			proc_waitMs(250);
			core.parm_->armCmd.set_angle_Yaw = 50.115f;

			/* Step 2 */
			proc_waitUntilWithTimeout(
				core.parm_->armInfo.isAngleArrived_Pitch1 == true &&
				core.parm_->armInfo.isAngleArrived_Pitch2 == true, 5000
			);
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.parm_->armCmd.isAutoCtrl = true;
			core.pgimbal_->gimbalCmd.isAutoCtrl = true;

			core.psubgantry_->subGantryCmd.setPumpOn_Arm = true; // 启动机械臂气泵
			core.psubgantry_->subGantryCmd.setPumpOn_Left = true; // 启动左气泵
			proc_waitMs(500);

			// core.parm_->armCmd.set_angle_Yaw = 21.35f;
			proc_waitMs(250);
			core.psubgantry_->subGantryCmd.setStretchPosit_L = 85.0f;

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

			/* Step 3 */
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.parm_->armCmd.isAutoCtrl = true;
			core.pgimbal_->gimbalCmd.isAutoCtrl = true;

			core.psubgantry_->subGantryCmd.setPumpOn_Arm = false; // 关闭机械臂气泵
			proc_waitMs(500);

			core.parm_->armCmd.set_angle_Pitch2 += 10.0f;
			core.parm_->armCmd.set_angle_Yaw = 0.0f;
			proc_waitMs(250);
			core.parm_->armCmd.set_angle_Pitch1 = 50.0f;
			core.parm_->armCmd.set_angle_Pitch2 = 40.0f;
			core.parm_->armCmd.set_angle_Roll = -2.178f;
			core.parm_->armCmd.set_angle_end_pitch = 0.0f;
			core.parm_->armCmd.set_angle_end_roll = 0.000f;
			proc_waitMs(250);
			core.psubgantry_->subGantryCmd.setStretchPosit_L = 0.0f;

			goto proc_exit;
		}

		if (keyboard.mouse_R) {

			/*step 1*/
			core.psubgantry_->subGantryCmd.setStretchPosit_R = 0.0f;
			
			proc_waitMs(250);

			core.parm_->armCmd.set_angle_Pitch1 = 62.254f;
			core.parm_->armCmd.set_angle_Pitch2 = 72.311f;
			core.parm_->armCmd.set_angle_Roll = -1.480f;
			core.parm_->armCmd.set_angle_end_pitch = -118.690f;
			core.parm_->armCmd.set_angle_end_roll += 35.280f;
			core.psubgantry_->subGantryCmd.setLiftPosit_R = SUB_GANTRY_LIFT_PHYSICAL_RANGE_R;
			proc_waitUntilWithTimeout(core.psubgantry_->subGantryInfo.isPositArrived_Lift_R == true, 5000);
			proc_waitMs(250);
			core.parm_->armCmd.set_angle_Yaw = -52.075f;

			/* Step 2 */
			proc_waitUntilWithTimeout(
				core.parm_->armInfo.isAngleArrived_Pitch1 == true &&
				core.parm_->armInfo.isAngleArrived_Pitch2 == true, 5000
			);
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.parm_->armCmd.isAutoCtrl = true;
			core.pgimbal_->gimbalCmd.isAutoCtrl = true;

			core.psubgantry_->subGantryCmd.setPumpOn_Arm = true; // 启动机械臂气泵
			core.psubgantry_->subGantryCmd.setPumpOn_Right = true; // 启动右气泵
			proc_waitMs(500);

			// core.parm_->armCmd.set_angle_Yaw = 21.35f;
			proc_waitMs(250);
			core.psubgantry_->subGantryCmd.setStretchPosit_R = 60.0f;

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

			/* Step 3 */
			core.psubgantry_->subGantryCmd.isAutoCtrl = true;
			core.parm_->armCmd.isAutoCtrl = true;
			core.pgimbal_->gimbalCmd.isAutoCtrl = true;

			core.psubgantry_->subGantryCmd.setPumpOn_Arm = false; // 关闭机械臂气泵
			proc_waitMs(500);

			core.parm_->armCmd.set_angle_Pitch1 -= 5.0f;
			core.parm_->armCmd.set_angle_Yaw = 0.0f;
			proc_waitMs(250);
			core.parm_->armCmd.set_angle_Pitch1 = 50.0f;
			core.parm_->armCmd.set_angle_Pitch2 = 40.0f;
			core.parm_->armCmd.set_angle_Roll = -2.178f;
			core.parm_->armCmd.set_angle_end_pitch = 0.0f;
			core.parm_->armCmd.set_angle_end_roll = 0.000f;
			proc_waitMs(250);
			core.psubgantry_->subGantryCmd.setStretchPosit_R = 0.0f;

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
