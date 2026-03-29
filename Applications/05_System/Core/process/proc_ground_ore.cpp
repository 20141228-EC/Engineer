/******************************************************************************
 * @brief        
 * 
 * @file         proc_ground_ore.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-15
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    捡地矿任务
 ******************************************************************************/
void CSystemCore::StartGroundOreTask(void *arg) {

    if (arg == nullptr) proc_return();

    // 获取SystemCore句柄
    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;
    auto cnt = 0;
    const auto timeout = 60000 / 5; // 12000ms

    while (keyboard.key_Ctrl) {         ///< 按住ctrl

		if (keyboard.mouse_L) {         ///< 捡一个地矿

			/*step 1*/

			core.movemode_ = EMoveMode::NORMAL;
    		core.pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
            
            /*Set Arm*/
            core.parm_->armCmd.set_angle_Yaw = GROUND_ORE_YAW_ANGLE;
            core.parm_->armCmd.set_angle_Pitch1 = GROUND_ORE_PITCH1_ANGLE;
            core.parm_->armCmd.set_angle_Pitch2 = GROUND_ORE_PITCH2_ANGLE;
            core.parm_->armCmd.set_angle_Roll = GROUND_ORE_ROLL_ANGLE;
            core.parm_->armCmd.set_angle_end_pitch = GROUND_ORE_END_PITCH_ANGLE;
            core.parm_->armCmd.set_angle_end_roll = GROUND_ORE_END_ROLL_ANGLE;
            core.parm_->armCmd.set_length_grip = GROUND_ORE_GRIP_LENGTH;

            /*Set Chassis*/
            core.pchassis_->chassisCmd.L_length = GROUND_ORE_HIP_LENGTH;
            // 抬腿

            /*Set Gimbal*/
    
            // 云台应当抬升到最高点 同时 pitch俯角下降
            // 以上逻辑是为了先确保机器人动到固定的起始位置，后面再根据矿具体位置进行微调

			proc_waitMs(250);		///< 等待各电机到位

			/* Wait for User Confirmation */
			cnt = timeout;
			core.parm_->armCmd.isAutoCtrl = false;
			core.gimbal_auto_ctrl = false;      
			while (cnt--) {
				if (keyboard.key_Ctrl) {
					if (keyboard.mouse_L) break;
					if (keyboard.mouse_R) goto proc_exit;
				}
				proc_waitMs(5);
			}
			if (cnt == 0) goto proc_exit;           ///< 12s 超时退出

			/*step 2*/
			core.parm_->armCmd.isAutoCtrl = true;			
			core.gimbal_auto_ctrl = true;           ///< 开启自动控制
			
			core.parm_->armCmd.set_angle_Yaw = 0.f;
			core.parm_->armCmd.set_angle_Pitch1 = 90.0f;
			core.parm_->armCmd.set_angle_Pitch2 = 45.0f;
			core.parm_->armCmd.set_angle_Roll = -1.480f;
			core.parm_->armCmd.set_angle_end_pitch = -58.180f;
			core.parm_->armCmd.set_angle_end_roll = 0.000f;
			proc_waitMs(300);
            // 等确认继续进行之后动到真正的准备捡地矿位置，这个step2主要是为了提供一个缓冲区
            // 这些角度等具体出车还得改

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
			// 缓慢下落逻辑
      
      		goto proc_exit;
		}

		if (keyboard.mouse_R) {                 // 捡完一个地矿之后进行存矿后再继续捡

			/*step 1*/
			//此处云台应当抬升

			core.parm_->armCmd.set_angle_Yaw = 0.f;
			core.parm_->armCmd.set_angle_Pitch1 = 76.390f;
			core.parm_->armCmd.set_angle_Pitch2 = 43.207f;
			core.parm_->armCmd.set_angle_Roll = -1.480f;
			core.parm_->armCmd.set_angle_end_pitch = -60.815f;
			core.parm_->armCmd.set_angle_end_roll = 0.000f;

			proc_waitMs(250);

			/* Wait for User Confirmation */
			cnt = timeout;
			core.parm_->armCmd.isAutoCtrl = false;      
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
			
			core.parm_->armCmd.set_angle_Yaw -= 25.0f;
			proc_waitMs(500);

			/* Step 3 */
			core.parm_->armCmd.set_angle_Pitch1 = 76.390f;
			core.parm_->armCmd.set_angle_Pitch2 = 43.207f;
			core.parm_->armCmd.set_angle_Roll = -1.480f;
			core.parm_->armCmd.set_angle_end_pitch = -71.815f;
			core.parm_->armCmd.set_angle_end_roll = 0.000f;

			proc_waitMs(300);
			core.parm_->armCmd.set_angle_Yaw = 0.f;

			proc_waitMs(250);

			/* Wait for User Confirmation */
			cnt = timeout;
			core.parm_->armCmd.isAutoCtrl = false;    
			while (cnt--) {
				if (keyboard.key_Ctrl) {
					if (keyboard.mouse_L) break;
					if (keyboard.mouse_R) goto proc_exit;
				}
				proc_waitMs(5);
			}
			if (cnt == 0) goto proc_exit;

			core.parm_->armCmd.isAutoCtrl = true;
			
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
    core.pchassis_->chassisCmd.isAutoCtrl = false;
    core.gimbal_auto_ctrl = false;
    core.autoCtrlTaskHandle_ = nullptr;
    core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
	core.movemode_ = EMoveMode::NORMAL;
    core.pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
    proc_return();

}

} // namespace my_engineer