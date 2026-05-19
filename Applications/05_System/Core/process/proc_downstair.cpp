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

	/* Phase 1: 臂自动移到预设位姿，底盘WASD反转控制(图传180°掉头) */
	core.parm_->armCmd.isAutoCtrl = true;           ///< 臂自动控制
	core.gimbal_auto_ctrl = true;					///< 云台自动控制
	core.pchassis_->chassisCmd.isAutoCtrl = true;   ///< 阻止外部WASD，由本任务内部反转处理

	/*Set Arm*/
	core.parm_->armCmd.set_angle_Yaw = DOWNSTAIR_YAW_ANGLE;
	core.parm_->armCmd.set_angle_Pitch1 = DOWNSTAIR_PITCH1_ANGLE;
	core.parm_->armCmd.set_angle_Pitch2 = DOWNSTAIR_PITCH2_ANGLE - 5;
	core.parm_->armCmd.set_angle_Roll = DOWNSTAIR_ROLL_ANGLE;
	core.parm_->armCmd.set_angle_end_pitch = DOWNSTAIR_END_PITCH_ANGLE;
	//core.parm_->armCmd.set_angle_Roll = DOWNSTAIR_ROLL_ANGLE;
	core.parm_->armCmd.set_angle_end_roll = DOWNSTAIR_END_ROLL_ANGLE;
    core.parm_->armCmd.set_length_grip = DOWNSTAIR_GRIP_LENGTH;
	core.pgimbal_->gimbalCmd.set_visualyaw = DOWNSTAIR_GIMBAL_ANGLE;

	proc_waitMs(300);	// 等待臂到位

	/* Phase 1: 等待鼠标左键确认，WASD反转以适应图传180°掉头 */
	while (!keyboard.mouse_L) {

		// 阻尼衰减
		core.pchassis_->chassisCmd.speed_X *= 0.97f;
		core.pchassis_->chassisCmd.speed_Y *= 0.98f;
		if (abs(core.pchassis_->chassisCmd.speed_X) < 0.5f) core.pchassis_->chassisCmd.speed_X = 0.0f;
		if (abs(core.pchassis_->chassisCmd.speed_Y) < 0.5f) core.pchassis_->chassisCmd.speed_Y = 0.0f;

		// WASD反转: 图传180°掉头后，A/D和W/S方向都颠倒
		if (keyboard.key_Shift) {
			core.pchassis_->chassisCmd.speed_X += static_cast<float_t>(keyboard.key_A - keyboard.key_D) * 5.0f;
			core.pchassis_->chassisCmd.speed_Y += static_cast<float_t>(keyboard.key_S - keyboard.key_W) * 5.0f;
			core.pchassis_->chassisCmd.speed_X = std::clamp(core.pchassis_->chassisCmd.speed_X, -50.0f, 50.0f);
			core.pchassis_->chassisCmd.speed_Y = std::clamp(core.pchassis_->chassisCmd.speed_Y, -100.0f, 100.0f);
		} else {
			core.pchassis_->chassisCmd.speed_X += static_cast<float_t>(keyboard.key_A - keyboard.key_D) * 1.0f;
			core.pchassis_->chassisCmd.speed_Y += static_cast<float_t>(keyboard.key_S - keyboard.key_W) * 1.0f;
			core.pchassis_->chassisCmd.speed_X = std::clamp(core.pchassis_->chassisCmd.speed_X, -20.0f, 20.0f);
			core.pchassis_->chassisCmd.speed_Y = std::clamp(core.pchassis_->chassisCmd.speed_Y, -30.0f, 30.0f);
		}

		proc_waitMs(20);
	}
    core.movemode_ = EMoveMode::DOWNSTAIR;  // 更新系统层标志位
	core.pchassis_->MovMode = CModChassis::EmovMode::DOWNSTAIR;     // 更新模块层标志位

	/* Phase 2: 下台阶 */
    while (keyboard.key_Ctrl)
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
	core.pgimbal_->gimbalCmd.set_visualyaw = 0.f;//头部回正
	core.gimbal_auto_ctrl = false;
	core.autoCtrlTaskHandle_ = nullptr;
	core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
	core.movemode_ = EMoveMode::NONE;
	core.pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
	proc_return();

}

} // namespace my_engineer
