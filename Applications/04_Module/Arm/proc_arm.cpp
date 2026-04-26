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
				arm.comEnd_.StopComponent();
				arm.comGrip_.StopComponent();

				proc_waitMs(20);
				continue; // 跳过下面的代码，直接进入下一次循环
			}

			case FSM_INIT: {

				proc_waitMs(250); // 等待系统稳定

				arm.comjoint_.StartComponent();												///< 刚开始的时候设置为busy状态，当初始化以后就设置为ok状态
				proc_waitUntil(arm.comjoint_.componentStatus == APP_OK);					///< 此处先挂起10ms之后，一直等待关节电机任务初始化结束否则就一直10ms的等
				
				arm.comRoll_.StartComponent();												///< 当关节电机初始化完成之后，启动末端夹爪和夹爪roll电机任务
				proc_waitUntil(arm.comRoll_.componentStatus == APP_OK);	
				
				arm.comEnd_.StartComponent();
				proc_waitUntil(arm.comEnd_.componentStatus == APP_OK);
					
				arm.comGrip_.StartComponent();  ///< 等待末端初始化完成
				proc_waitUntil(arm.comGrip_.componentStatus == APP_OK);

							   

				arm.armCmd = SArmCmd();
				arm.armCmd.set_angle_Yaw = ARM_YAW_INIT_ANGLE;
				arm.armCmd.set_angle_Pitch1 = ARM_PITCH1_INIT_ANGLE;
				arm.armCmd.set_angle_Pitch2 = ARM_PITCH2_INIT_ANGLE;
				arm.armCmd.set_angle_Pitch3 = ARM_PITCH3_INIT_ANGLE;
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
					CComJoint::PhyPositToMtrPosit_yaw(arm.armCmd.set_angle_Yaw);			///< 在这个文件中设置目标的位置，在com_joint.cpp中进行pid计算
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
					arm.comEnd_.PhyPositToMtrPosit_Roll(arm.armCmd.set_angle_end_roll);

				// 夹爪命令传递：手动模式传标志位，自动模式传目标位置
				const bool manualGrip =
					!arm.armCmd.isAutoCtrl && (arm.armCmd.gripClose || arm.armCmd.gripOpen);

				arm.comGrip_.gripCmd.cmdClose = manualGrip && arm.armCmd.gripClose;
				arm.comGrip_.gripCmd.cmdOpen  = manualGrip && arm.armCmd.gripOpen;

				if (manualGrip) {
					// 手动模式：不让 proc_arm 覆盖 setPosit_grip，同步当前位置到 armCmd
					arm.armCmd.set_length_grip = CComGrip::MtrPositToPhyPosit(
						static_cast<float_t>(arm.comGrip_.gripInfo.posit_grip));
				} else {
					// 自动任务：从 armCmd 设定目标位置，确保手动标志清除
					arm.comGrip_.gripCmd.setPosit_grip =
						CComGrip::PhyPositToMtrPosit(arm.armCmd.set_length_grip);
				}

				// 二次夹紧指令传递到组件层
				if (arm.armCmd.reGripCmd) {
					arm.comGrip_.gripCmd.cmdReGrip = true;
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
