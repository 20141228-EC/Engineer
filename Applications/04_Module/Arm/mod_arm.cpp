/******************************************************************************
 * @brief
 *
 * @file         mod_arm.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2026-01-28
 *
 * @copyright    Copyright (c) 2026
 *
 ******************************************************************************/

#include "mod_arm.hpp"



namespace my_engineer {

CModArm *pArm_test = nullptr;

// ///< 全局变量
bool Need_Grav_Compensation = false; ///< 是否启用重力补偿
bool Is_Recording_ArmTorque = true; ///<是否正在记录数据
DataBuffer<float_t> arm_Info[3][10]; ///<用于记录臂的力矩，三个关节，1000个数据点
uint16_t index = 0; ///< 数组索引
bool is_record = false; ///< 是否要记录数据

/**
 * @brief 初始化机械臂模块
 *
 * @param param
 * @return EAppStatus
 */
EAppStatus CModArm::InitModule(SModInitParam_Base &param) {
	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto armParam = static_cast<SModInitParam_Arm &>(param);
	moduleID = armParam.moduleID;

	comjoint_.InitComponent(param);
	comRoll_.InitComponent(param);
	comEndPitch_.InitComponent(param);
	comEndRoll_.InitComponent(param);
	comGrip_.InitComponent(param);
    gravComp_.Init(armParam.gravParam);

	// 创建任务并注册模块
	CreateModuleTask_();
	RegisterModule_();

	// test
	pArm_test = this;

	Module_FSMFlag_ = FSM_RESET;
	moduleStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新处理
 *
 */
void CModArm::UpdateHandler_() {
	// 检查模块状态
	static uint8_t HalfTickRate = 0;
	HalfTickRate = 1 - HalfTickRate;
	if (moduleStatus == APP_RESET) return;

	if(HalfTickRate) {
		comRoll_.UpdateComponent();
		comEndPitch_.UpdateComponent();	///<更新末端Pitch电机数据
		comEndRoll_.UpdateComponent();	///<更新末端Roll电机数据
	} ///< 降为500Hz
	comGrip_.UpdateComponent();
	comjoint_.UpdateComponent();

	// 更新模块信息
	armInfo.angle_Yaw = comjoint_.MtrPositToPhyPosit_yaw(comjoint_.jointInfo.posit_yaw);
	armInfo.angle_Pitch1 = comjoint_.MtrPositToPhyPosit_pitch1(comjoint_.jointInfo.posit_pitch1);
	armInfo.angle_Pitch2 = comjoint_.MtrPositToPhyPosit_pitch2(comjoint_.jointInfo.posit_pitch2); ///<这里是将底层的关节信息转换为用户层的arm信息
	armInfo.angle_Roll = comRoll_.MtrAngleToPhyAngle(comRoll_.rollInfo.angle);
	armInfo.angle_end_pitch = comEndPitch_.endPitchInfo.angle;
	armInfo.angle_end_roll = comEndRoll_.endRollInfo.angle;								///<MIT电机直接读取角度
	armInfo.length_grip = CComGrip::MtrPositToPhyPosit(static_cast<float_t>(comGrip_.gripInfo.posit_grip));
	armInfo.isAngleArrived_Yaw = comjoint_.jointInfo.isPositArrived_yaw;
	armInfo.isAngleArrived_Pitch1 = comjoint_.jointInfo.isPositArrived_pitch1;
	armInfo.isAngleArrived_Pitch2 = comjoint_.jointInfo.isPositArrived_pitch2;
	armInfo.isAngleArrived_Roll = comRoll_.rollInfo.isAngleArrived;
	armInfo.isAngleArrived_End_Pitch = comEndPitch_.endPitchInfo.isAngleArrived;
	armInfo.isAngleArrived_End_Roll = comEndRoll_.endRollInfo.isAngleArrived;
	armInfo.isAngleArrived_Grip = comGrip_.gripInfo.isGripped;
	armInfo.isGripped = comGrip_.gripInfo.isGripped;
	armInfo.gripState = (comGrip_.gripInfo.state == CComGrip::SGripInfo::EGripState::HOLD) ? SArmInfo::EGripState::HOLD
																						    : SArmInfo::EGripState::RELEASE;//模块层传递夹爪的状态
	armInfo.holdLength_grip = CComGrip::MtrPositToPhyPosit( static_cast<float_t>(comGrip_.gripInfo.holdPosit_Grip));
	gravState_ = { armInfo.angle_Pitch1, armInfo.angle_Pitch2,armInfo.angle_Roll, armInfo.angle_end_roll, armInfo.angle_end_pitch};
	gravOut_ = gravComp_.Calc(gravState_);
	if(gravityOnlyMode_ || !gravComp_.IsObserveMode()){  // 重补的调试模式和非观察观察模式
		comjoint_.grav_ff_pitch1 = gravOut_.pitch1_current_ff;
		comjoint_.grav_ff_pitch2 = gravOut_.pitch2_current_ff;
		comRoll_.grav_ff_roll = gravOut_.roll_tau_ff;
		comEndRoll_.Grav_End_Roll_Out = gravOut_.end_roll_tau_ff;
		comEndPitch_.Grav_End_Pitch_Out = gravOut_.end_pitch_tau_ff;
	}
	else{
		comjoint_.grav_ff_pitch1 = 0.0f;// 观察模式下不直接加重补到输出
		comjoint_.grav_ff_pitch2 = 0.0f;
		comRoll_.grav_ff_roll = 0.0f;
		comEndRoll_.Grav_End_Roll_Out = 0.0f;
		comEndPitch_.Grav_End_Pitch_Out = 0.0f;
	}
	bool gravActive = !gravComp_.IsObserveMode();// 观察模式的变量传递


	// 填充电机发送缓冲区
	CDevMtrKT::FillCanTxBuffer(comjoint_.motor[CComJoint::P1],							///<用的是关节底层信息的发送
								comjoint_.mtrCanTxNode[CComJoint::P1]->dataBuffer,
								comjoint_.mtrOutputBuffer[CComJoint::P1]);		///< 1.电机 2.can节点信息 3. 输出缓冲区
	CDevMtrKT::FillCanTxBuffer(comjoint_.motor[CComJoint::P2],
								comjoint_.mtrCanTxNode[CComJoint::P2]->dataBuffer,
								comjoint_.mtrOutputBuffer[CComJoint::P2]);
	CDevMtrKT::FillCanTxBuffer(comjoint_.motor[CComJoint::Y],
								comjoint_.mtrCanTxNode[CComJoint::Y]->dataBuffer,
								comjoint_.mtrOutputBuffer[CComJoint::Y]);
	CDevMtrKT::FillCanTxBuffer(comGrip_.motor,
								comGrip_.mtrCanTxNode->dataBuffer,
								comGrip_.mtrOutputBuffer);

}

/**
 * @brief 心跳处理
 *
 */
void CModArm::HeartbeatHandler_() {
	// 心跳处理逻辑
}

/**
 * @brief 创建模块任务
 *
 */
EAppStatus CModArm::CreateModuleTask_() {
	// 任务已存在，删除任务
	if (moduleTaskHandle != nullptr) vTaskDelete(moduleTaskHandle);

	// 创建任务
	xTaskCreate(StartArmModuleTask, "Arm Task",
				512, this, proc_ModuleTaskPriority,
				&moduleTaskHandle);

	return APP_OK;
}

/**
 * @brief 限制机械臂模块的控制命令大小
 *
 */
EAppStatus CModArm::RestrictArmCommand_() {
	// 检查模块状态
	if (moduleStatus == APP_RESET) {
		armCmd = SArmCmd();
		return APP_ERROR;
	}
	// 自动控制模式下跳过后续更复杂的动态限位
	// static float_t prevPitch1 = ARM_PITCH1_INIT_ANGLE;
	// if (armCmd.isAutoCtrl) {
	// 	prevPitch1 = armCmd.set_angle_Pitch1;  ///< 保持追踪，防止退出自动模式时P1-P2耦合跳变
    //     return APP_OK;
    // }
    // // 物理限位
    // armCmd.set_angle_Yaw =
    //     std::clamp(armCmd.set_angle_Yaw, ARM_YAW_PHYSICAL_RANGE_MIN, ARM_YAW_PHYSICAL_RANGE_MAX);

	// // if(armCmd.set_angle_Pitch2 < 13.f && armCmd.set_angle_Roll < 90.f ) {
	// // 	armCmd.set_angle_end_pitch = std::clamp(armCmd.set_angle_end_pitch, 0.f, 90.f);
	// // }

    // armCmd.set_angle_Pitch1 =
    //     std::clamp(armCmd.set_angle_Pitch1,
    //                ARM_PITCH1_PHYSICAL_RANGE_MIN, ARM_PITCH1_PHYSICAL_RANGE_MAX);

	// if(!armCmd.isCustomCtrl){  //非自定义控制器模式
	// 	armCmd.set_angle_Pitch2 += (armCmd.set_angle_Pitch1 - prevPitch1) * 1;
	// 	prevPitch1 = armCmd.set_angle_Pitch1;

	// 	float_t p2UpperLimit = std::min(ARM_P2_MAX_WHEN_P1_MIN + armCmd.set_angle_Pitch1,
	// 													ARM_PITCH2_PHYSICAL_RANGE_MAX);//p2的动态限位受到p1的影响

	// 	armCmd.set_angle_Pitch2 =
	// 		std::clamp(armCmd.set_angle_Pitch2, ARM_PITCH2_PHYSICAL_RANGE_MIN, p2UpperLimit);

	// }

	// else{ //自定义控制器模式下
	// 	armCmd.set_angle_Pitch2 =
	// 		std::clamp(armCmd.set_angle_Pitch2,
	// 					ARM_PITCH2_PHYSICAL_RANGE_MIN, ARM_PITCH2_PHYSICAL_RANGE_MAX);
	// }
	// prevPitch1 = armCmd.set_angle_Pitch1; //避免切换的跳变

    // armCmd.set_angle_Roll =
    //     std::clamp(armCmd.set_angle_Roll,
    //                ARM_ROLL_PHYSICAL_RANGE_MIN, ARM_ROLL_PHYSICAL_RANGE_MAX);
    // armCmd.set_angle_end_pitch =
    //     std::clamp(armCmd.set_angle_end_pitch,
    //                ARM_END_PITCH_PHYSICAL_RANGE_MIN, ARM_END_PITCH_PHYSICAL_RANGE_MAX);
    // armCmd.set_length_grip =
    //     std::clamp(armCmd.set_length_grip,
    //         ARM_END_GRIP_PHYSICAL_RANGE_MIN, ARM_END_GRIP_PHYSICAL_RANGE_MAX);
	// armCmd.set_angle_end_roll =
    //     std::clamp(armCmd.set_angle_end_roll,
    //         ARM_END_ROLL_PHYSICAL_RANGE_MIN, ARM_END_ROLL_PHYSICAL_RANGE_MAX);//限制末端roll

    // // 最高优先级的限位
    // if (should_limit_yaw) {
    //     armCmd.set_angle_Yaw = std::clamp(armCmd.set_angle_Yaw,
    //         -12.0f, 6.0f);
    // }
		return APP_OK;
}

/**
 * @brief 设置纯重补模式，用于验证重补效果
 */
void CModArm::SetGravityOnlyMode(bool enable) {
	gravityOnlyMode_ = enable;
	comjoint_.SetOnlyGravity(enable);
	comRoll_.SetOnlyGravity(enable);
	comEndRoll_.SetOnlyGravity(enable);
	comEndPitch_.SetOnlyGravity(enable);
}

} // namespace my_engineer
