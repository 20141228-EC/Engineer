/**
 * @file proc_exchange.cpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 
 * @version 1.0
 * @date 2025-01-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "Core.hpp"

#define deg2rad (3.1415926f / 180.0f)

#define sucker_down 3.0f	// 吸盘吸矿下垂补偿


namespace my_engineer {

void CSystemCore::StartExchangeTask(void *arg) {
	
	if (arg == nullptr) proc_return();

	// 获取SystemCore句柄
	auto &core = *reinterpret_cast<CSystemCore *>(arg);
	auto &keyboard = SysRemote.remoteInfo.keyboard;
	auto cnt = 0;
	const auto timeout = 60000 / 5; // unit: ms

	/*step 1 */
	core.psubgantry_->subGantryCmd.setStretchPosit_L= 0.0f;
	core.psubgantry_->subGantryCmd.setLiftPosit_L = 52.5f;
	core.psubgantry_->subGantryCmd.setStretchPosit_R = 0.0f;
	core.psubgantry_->subGantryCmd.setLiftPosit_R = 52.5f;

	core.psubgantry_->subGantryCmd.setPumpOn_Arm = true; // 启动机械臂气泵

	cnt = 60000*3;
	core.parm_->armCmd.isAutoCtrl = false;
	proc_waitMs(500);
	while (cnt--) {
		if (keyboard.key_Ctrl) {
			if (keyboard.mouse_L) break;
			if (keyboard.mouse_R) goto proc_exit;              ///这里需要考虑是启动自定义控制兑矿还是使用视觉
		}
		proc_waitMs(1);
	}
	if (cnt == 0) goto proc_exit;

	/*step 2 */
	core.use_Controller_ = true; // 使用自定义控制器

	/* Wait for User Confirmation */
	cnt = 60000*10;
	core.parm_->armCmd.isAutoCtrl = false;
	core.psubgantry_->subGantryCmd.isAutoCtrl = false;
	core.pgimbal_->gimbalCmd.isAutoCtrl = false;
	proc_waitMs(500);
	while (cnt--){
		if (SysRemote.remoteInfo.keyboard.key_Ctrl) {
			if (SysRemote.remoteInfo.keyboard.mouse_L) break;
			if (SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
		}
		proc_waitMs(1);
	}
	if (cnt == 0) goto proc_exit;

	/*step 3*/
	core.use_Controller_ = false; // 关闭自定义控制器
	core.psubgantry_->subGantryCmd.setPumpOn_Arm = false; // 关闭机械臂气泵
	/*Set Arm*/
	core.parm_->armCmd.set_angle_Yaw = 0.0f;
	core.parm_->armCmd.set_angle_Pitch1 = 30.2f;
	core.parm_->armCmd.set_angle_Pitch2 = 22.0f;
	core.parm_->armCmd.set_angle_Roll = 0.0f;
	core.parm_->armCmd.set_angle_end_pitch = -90.0f;
	core.parm_->armCmd.set_angle_end_roll = 0.0f;

	/*Set Gimbal*/
	core.pgimbal_->gimbalCmd.set_posit_lift = GIMBAL_LIFT_PHYSICAL_RANGE;  ///< Pull up the gimbal

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


/*****************************************************************************

 * 				
 * 				    Z
 * 				 ↗
 * 				|————————————————————→ X
 * 				|          相机系（操作手视角）
 *   			|          
 * 				|          
 * 				|          
 * 				↓
 * 				Y
 */
