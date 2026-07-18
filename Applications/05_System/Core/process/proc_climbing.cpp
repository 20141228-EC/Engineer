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
#include "proc_traj_data.hpp"


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

	float_t aimTarget[J::COUNT];// 构造空的数组
    aimTarget[J::J_YAW]  = CLIMBING_YAW_ANGLE;
    aimTarget[J::J_P1]   = CLIMBING_PITCH1_ANGLE;
    aimTarget[J::J_P2]   = CLIMBING_PITCH2_ANGLE;
    aimTarget[J::J_ROLL] = CLIMBING_ROLL_ANGLE;
    aimTarget[J::J_ENDP] = CLIMBING_END_PITCH_ANGLE;
    aimTarget[J::J_ENDR] = CLIMBING_END_ROLL_ANGLE;

    SPlayJointTargetOptions opt;
    opt.speedScale = 1.5f;
    opt.gripKeepCurrent = true;
    if(!PlayJointTarget(*core.parm_ ,aimTarget ,opt)) goto proc_exit;// 平滑过渡
	// 对臂的姿态不作限制，操作手根据情况调整

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
		if(!core.pchassis_->should_be_saved){	// 非自救情况下

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
				// if(core.pchassis_->is_climbed && core.pchassis_->time_to_reset_hip){
				if(core.pchassis_->is_climbed){
					core.pchassis_->is_climbing = false;
					proc_waitMs(40);
					core.pchassis_->reset_hip = true;		// 检测到前轮爬上台阶之后就收腿 可能会需要一个延时
					core.pchassis_->is_climbed = false;
				}
				// if(core.pchassis_->time_to_reset_hip){
				// 	core.pchassis_->is_climbing = false;
				// 	proc_waitMs(1000);	// 过一秒就收腿	
				// 	core.pchassis_->reset_hip = true;
				// 	core.pchassis_->time_to_reset_hip = false;		// 硬延时收腿 没办法了再用这个
				// }
			}
			else{		// 如果操作手判断卡住了导致腿没自动收，那就按住鼠标右键
				core.pchassis_->is_climbing = false;
				core.pchassis_->chassisCmd.L_length -= 90.f / 1000.f;
				// 此时开始慢慢收腿
			}
		}
		else{		// 需要自救
			core.pchassis_->chassisCmd.speed_Y = SAVING_SPEED;	// 退到台阶下
			core.pchassis_->chassisCmd.L_length = SAVING_HIP_ANGLE;	// 立刻抬腿
		}
		
        proc_waitMs(20);
    }
	// 松开ctrl退出上台阶模式

// 退出
proc_exit:
	core.parm_->armCmd.isAutoCtrl = false;
	core.pchassis_->chassisCmd.isAutoCtrl = false;
	core.pchassis_->is_climbing = false;
	core.pchassis_->is_climbed = false;
	core.pchassis_->chassisInfo.crawler_on = false;	// 关履带
	core.pchassis_->should_be_saved = false;	// 清自救标志位
	core.gimbal_auto_ctrl = false;
	core.autoCtrlTaskHandle_ = nullptr;
	core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
	core.movemode_ = EMoveMode::NONE;
	core.pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
	proc_return();

}

} // namespace my_engineer
