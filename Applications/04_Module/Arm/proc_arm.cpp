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

/**
 * @brief 创建机械臂任务
 * 
 * @param argument 
 */
void CModArm::StartArmModuleTask(void *argument) {					///<该任务在mod_arm.cpp被创建，然后在任务调度器调度

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
				arm.comEnd_.StopComponent();
				arm.comGrip_.StopComponent();

				proc_waitMs(20);
				continue; // 跳过下面的代码，直接进入下一次循环
			}

			case FSM_INIT: {

				proc_waitMs(250); // 等待系统稳定

				arm.comEnd_.StartComponent();       ///<一定得先末端后夹爪，要不然补偿会失效
				proc_waitUntil(arm.comEnd_.componentStatus == APP_OK);
				
				//test
				arm.comGrip_.StartComponent();
				proc_waitUntil(arm.comGrip_.componentStatus == APP_OK);

				arm.comjoint_.StartComponent();												///<刚开始的时候设置为busy状态，当初始化以后就设置为ok状态
				proc_waitUntil(arm.comjoint_.componentStatus == APP_OK);					///<此处先挂起10ms之后，一直等待关节电机任务初始化结束否者就一直10ms的等
				
				

				
				arm.comRoll_.StartComponent();		///<当关节电机初始化完成之后，启动末端夹爪和夹爪roll电机任务
				proc_waitUntil(arm.comEnd_.componentStatus == APP_OK &&
							   arm.comRoll_.componentStatus == APP_OK);

				arm.armCmd = SArmCmd();
				arm.armCmd.set_angle_Yaw = ARM_YAW_INIT_ANGLE;
				arm.armCmd.set_angle_Pitch1 = ARM_PITCH1_INIT_ANGLE;
				arm.armCmd.set_angle_Pitch2 = ARM_PITCH2_INIT_ANGLE;
				arm.armCmd.set_angle_Roll = ARM_ROLL_INIT_ANGLE;
				arm.armCmd.set_angle_end_pitch = ARM_END_PITCH_INIT_ANGLE;
				arm.armCmd.set_length_grip = ARM_GRIP_INIT_LENGTH;
				arm.armInfo.isModuleAvailable = true;
				arm.Module_FSMFlag_ = FSM_CTRL;
				arm.moduleStatus = APP_OK;

				break;
			}

			case FSM_CTRL: {

				arm.RestrictArmCommand_();

				arm.comjoint_.jointCmd.setPosit_yaw = 
					CComJoint::PhyPositToMtrPosit_yaw(arm.armCmd.set_angle_Yaw);			///<在这个文件中设置目标的位置，在com_joint.cpp中进行pid计算
				arm.comjoint_.jointCmd.setPosit_pitch1 =
					CComJoint::PhyPositToMtrPosit_pitch1(arm.armCmd.set_angle_Pitch1);
				arm.comjoint_.jointCmd.setPosit_pitch2 =
					CComJoint::PhyPositToMtrPosit_pitch2(arm.armCmd.set_angle_Pitch2);
				arm.comjoint_.jointCmd.setPosit_pitch3 =
					CComJoint::PhyPositToMtrPosit_pitch3(arm.armCmd.set_angle_Pitch3);
				arm.comRoll_.rollCmd.setAngle =
					CComRoll::PhyAngleToMtrAngle(arm.armCmd.set_angle_Roll);
				arm.comEnd_.endCmd.setPosit_Pitch =
					CComEnd::PhyPositToMtrPosit_Pitch(arm.armCmd.set_angle_end_pitch);
				arm.comEnd_.endCmd.setPosit_Roll =
					CComEnd::PhyPositToMtrPosit_Roll(arm.armCmd.set_angle_end_roll);
				arm.comGrip_.gripCmd.setPosit_grip = 									///<夹爪的外部接口是距离，内部接口时编码器的数值
					CComGrip::PhyPositToMtrPosit(arm.armCmd.set_length_grip);

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
