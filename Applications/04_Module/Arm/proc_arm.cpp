/******************************************************************************
 * @brief
 *
 * @file         proc_arm.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-05-04
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#include "mod_arm.hpp"

namespace my_engineer {

using GravityMode = CAlgoGravityComp::CGravityCompMode;
/**
 * @brief 创建机械臂任务
 *
 * @param argument
 */
void CModArm::StartArmModuleTask(void *argument) {					///< 该任务在mod_arm.cpp被创建，然后在任务调度器调度

	// 要求参数为CModArm类的实例，如果传入为空则删除任务并返回
	if (argument == nullptr) proc_return();

	// 类型转换
	auto &arm = *static_cast<CModArm *>(argument);

	// 任务循环
	while (true) {

		// FSM
		switch (arm.Module_FSMFlag_) {

			case FSM_RESET: {

				arm.armInfo.isModuleAvailable = false;
				arm.comjoint_.StopComponent();
				arm.comRoll_.StopComponent();
				arm.comEndPitch_.StopComponent();
				arm.comEndRoll_.StopComponent();
				arm.comGrip_.StopComponent();
				arm.SetGravityCompMode(GravityMode::NONE);

				proc_waitMs(20);
				continue; // 跳过下面的代码，直接进入下一次循环
			}

			case FSM_INIT: {

				proc_waitMs(250); // 等待系统稳定


				/*step1 : 各自模块实现初始化*/
				arm.comEndPitch_.StartComponent();
				proc_waitUntil(arm.comEndPitch_.componentStatus == APP_OK);
				
				arm.comEndRoll_.StartComponent();
				proc_waitUntil(arm.comEndRoll_.componentStatus == APP_OK);

				arm.comRoll_.StartComponent();
				proc_waitUntil(arm.comRoll_.componentStatus == APP_OK);

				arm.comjoint_.StartComponent();
				proc_waitUntil(arm.comjoint_.componentStatus == APP_OK);



				arm.comGrip_.StartComponent();  ///< 等待末端初始化完成
				proc_waitUntil(arm.comGrip_.componentStatus == APP_OK);


				/*step2 : 任务层统一回到初始化的位置*/
				arm.armCmd = SArmCmd();
				arm.armCmd.set_angle_Yaw = ARM_YAW_INIT_ANGLE;
				arm.armCmd.set_angle_Pitch1 = ARM_PITCH1_INIT_ANGLE;
				arm.armCmd.set_angle_Pitch2 = ARM_PITCH2_INIT_ANGLE;
				arm.armCmd.set_angle_Roll = ARM_ROLL_INIT_ANGLE;
				arm.armCmd.set_angle_end_pitch = ARM_END_PITCH_INIT_ANGLE;
				arm.armCmd.set_angle_end_roll = ARM_END_ROLL_INIT_ANGLE;
				arm.armCmd.set_length_grip = ARM_GRIP_INIT_LENGTH;

				arm.comjoint_.jointCmd.setPosit_yaw =
					CComJoint::PhyPositToMtrPosit_yaw(arm.armCmd.set_angle_Yaw);
				arm.comjoint_.jointCmd.setPosit_pitch1 =
					CComJoint::PhyPositToMtrPosit_pitch1(arm.armCmd.set_angle_Pitch1);
				arm.comjoint_.jointCmd.setPosit_pitch2 =
					CComJoint::PhyPositToMtrPosit_pitch2(arm.armCmd.set_angle_Pitch2);
				arm.comRoll_.rollCmd.setAngle =
					CComRoll::PhyAngleToMtrAngle(arm.armCmd.set_angle_Roll);
				arm.comEndPitch_.endPitchCmd.setAngle = arm.armCmd.set_angle_end_pitch;
				arm.comEndRoll_.endRollCmd.setAngle = arm.armCmd.set_angle_end_roll;
				arm.comGrip_.gripCmd.setPosit_grip =
					CComGrip::PhyPositToMtrPosit(arm.armCmd.set_length_grip);
				proc_waitUntil(arm.comjoint_.jointInfo.isPositArrived_yaw &&
							arm.comjoint_.jointInfo.isPositArrived_pitch1 &&
							arm.comjoint_.jointInfo.isPositArrived_pitch2 &&
							arm.comRoll_.rollInfo.isAngleArrived);

				arm.armInfo.isModuleAvailable = true;
				arm.SetGravityCompMode(GravityMode::OBSERVE); // 只计算不输出
				arm.Module_FSMFlag_ = FSM_CTRL;
				arm.moduleStatus = APP_OK;

				break;
			}

			case FSM_CTRL: {

				arm.RestrictArmCommand_();

				arm.SetGravityCompMode(arm.armCmd.enableGravOnly ? GravityMode::GRAVITY_ONLY : GravityMode::ENABLE);

				arm.comjoint_.jointCmd.setPosit_yaw =
					CComJoint::PhyPositToMtrPosit_yaw(arm.armCmd.set_angle_Yaw);			///< 在这个文件中设置目标的位置，在com_joint.cpp中进行pid计算
				arm.comjoint_.jointCmd.setPosit_pitch1 =
					CComJoint::PhyPositToMtrPosit_pitch1(arm.armCmd.set_angle_Pitch1);
				arm.comjoint_.jointCmd.setPosit_pitch2 =
					CComJoint::PhyPositToMtrPosit_pitch2(arm.armCmd.set_angle_Pitch2);
				arm.comRoll_.rollCmd.setAngle =
					CComRoll::PhyAngleToMtrAngle(arm.armCmd.set_angle_Roll);
				arm.comEndPitch_.endPitchCmd.setAngle = arm.armCmd.set_angle_end_pitch;
				arm.comEndRoll_.endRollCmd.setAngle = arm.armCmd.set_angle_end_roll;

				// 夹爪命令传递：开合标志优先，否则透传速度
				const bool gripCommand = arm.armCmd.gripClose || arm.armCmd.gripOpen;

				arm.comGrip_.gripCmd.cmdClose = arm.armCmd.gripClose;
				arm.comGrip_.gripCmd.cmdOpen  = arm.armCmd.gripOpen;
				arm.comGrip_.gripCmd.setSpeed_grip =
					gripCommand ? 0.0f : arm.armCmd.set_speed_grip;

				if (gripCommand) {
					// 开合标志控制：不让 proc_arm 覆盖 setPosit_grip，同步当前位置到 armCmd
					arm.armCmd.set_length_grip = CComGrip::MtrPositToPhyPosit(
						static_cast<float_t>(arm.comGrip_.gripInfo.posit_grip));
				} else {
					// 位置控制：从 armCmd 设定目标位置
					arm.comGrip_.gripCmd.setPosit_grip =
						CComGrip::PhyPositToMtrPosit(arm.armCmd.set_length_grip);
				}

				// 二次夹紧指令传递到组件层
				if (arm.armCmd.reGripCmd) {
					arm.comGrip_.gripCmd.cmdReGrip = true;
					arm.armCmd.gripClose = false;
					arm.armCmd.gripOpen = false;
					arm.comGrip_.gripCmd.cmdClose = false;
					arm.comGrip_.gripCmd.cmdOpen = false;
					arm.armCmd.reGripCmd = false;
				}

				proc_waitMs(1); // 1000Hz
				break;
			}

			default: { arm.StopModule(); }
		}
	}

	// 任务退出
	arm.moduleTaskHandle = nullptr;
	proc_return();
}

} // namespace my_engineer
