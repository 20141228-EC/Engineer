/******************************************************************************
 * @brief        
 * 
 * @file         proc_climbing.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-14
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    上台阶任务
 ******************************************************************************/
void CSystemCore::StartClimbingTask(void *arg) {

	if (arg == nullptr) proc_return();

	// 获取SystemCore句柄
	auto &core = *reinterpret_cast<CSystemCore *>(arg);
	auto &keyboard = SysRemote.remoteInfo.keyboard;
	auto cnt = 0;
	const auto timeout = 2250 / 5; // 9s

	core.parm_->armCmd.isAutoCtrl = true;           ///< 臂自动控制
	core.pchassis_->chassisCmd.isAutoCtrl = true;   ///< 底盘自动控制
	core.gimbal_auto_ctrl = true;					///< 云台自动控制
	
	/*Set Arm*/
	core.parm_->armCmd.set_angle_Yaw = CLIMBING_YAW_ANGLE;
	core.parm_->armCmd.set_angle_Pitch1 = CLIMBING_PITCH1_ANGLE;
	core.parm_->armCmd.set_angle_Pitch2 = CLIMBING_PITCH2_ANGLE;
	core.parm_->armCmd.set_angle_Roll = CLIMBING_ROLL_ANGLE;
	core.parm_->armCmd.set_angle_end_pitch = CLIMBING_END_PITCH_ANGLE;
	core.parm_->armCmd.set_angle_end_roll = CLIMBING_END_ROLL_ANGLE;
    core.parm_->armCmd.set_length_grip = CLIMBING_GRIP_LENGTH;
	// 后续看情况得改 在初始化位置可能会干涉

	/*Set Chassis*/
	core.pchassis_->chassisInfo.crawler_on = true;	// 开履带
	core.pchassis_->chassisCmd.L_length = CLIMBING_HIP_ANGLE;

	/*Set Gimbal*/
	// 在副板设置云台

	proc_waitMs(300);	// 等待履带和髋关节到位

	core.movemode_ = EMoveMode::CLIMBING;
	core.pchassis_->MovMode = CModChassis::EmovMode::CLIMBING;

	// 上台阶任务比较特殊，由操作手来决定何时退出任务
    while (keyboard.key_Ctrl)				///< 按住ctrl
    {
		core.pchassis_->chassisCmd.speed_Y = CLIMBING_SPEED;	// 保持底盘速度
		if(keyboard.mouse_R == false){		///< 未按下右键
			// core.pchassis_->chassisCmd.speed_X = 50.f;
			if(core.pchassis_->is_climbing){
				// proc_waitMs(100);	// 等待100ms
				// core.movemode_ = EMoveMode::CLIMBING;
				// core.pchassis_->MovMode = CModChassis::EmovMode::CLIMBING;
				core.pchassis_->chassisCmd.L_length += 90.f / 1000.f;
				// 此时开始自动抬腿
			}
			if(core.pchassis_->is_climbed){
				core.pchassis_->is_climbing = false;
				core.pchassis_->reset_hip = true;		// 检测到前轮爬上台阶之后就收腿 可能会需要一个延时
				core.pchassis_->is_climbed = false;
			}
		}
		else{		// 如果操作手判断卡住了导致腿没自动收，那就按下鼠标右键
			core.pchassis_->is_climbing = false;
			core.pchassis_->chassisCmd.L_length -= 90.f / 1000.f;
			// 此时开始慢慢收腿
		}
        proc_waitMs(20);
    }
	// 松开ctrl退出上台阶模式

// 退出
proc_exit:
	core.parm_->armCmd.isAutoCtrl = false;
	core.pchassis_->chassisCmd.isAutoCtrl = false;
	core.pchassis_->is_climbed = false;
	core.pchassis_->chassisInfo.crawler_on = false;	// 关履带
	core.gimbal_auto_ctrl = false;
	core.autoCtrlTaskHandle_ = nullptr;
	core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
	core.movemode_ = EMoveMode::NONE;
	core.pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
	proc_return();

}

} // namespace my_engineer
