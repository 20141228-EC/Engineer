/******************************************************************************
 * @brief
 *
 * @file         mod_controller.cpp
 * @author       Fish_Joe (2328339747@qq.com), Ciallo
 * @version      V1.1
 * @date         2025-04-01
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-22
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#include "mod_controller.hpp"

namespace my_engineer {


/******************************************************************************
 * @brief    初始化控制器模块
 ******************************************************************************/
EAppStatus CModController::InitModule(SModInitParam_Base &param) {

	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto controllerParam = static_cast<SModInitParam_Controller &>(param);
	moduleID = controllerParam.moduleID;

	// 初始化控制器组件
	comPitchEnd_.InitComponent(param);
	comYaw_.InitComponent(param);
	comPitch1_.InitComponent(param);
	comPitch2_.InitComponent(param);
	comRoll_.InitComponent(param);
	comRocker_.InitComponent(param);
	comBuzzer_.InitComponent(param);

	// 初始化重力补偿算法
	gravityComp_.Init();
	gravityCompEnabled_ = true;   // 启用重力补偿
	gravityComp_.SetScale(1.0f); // 全局补偿比例保持1.0（理论值），各轴单独调节


	// 创建任务并注册模块
	CreateModuleTask_();
	RegisterModule_();

	Module_FSMFlag_ = FSM_RESET;
	moduleStatus = APP_OK;

	return APP_OK;
}

/******************************************************************************
 * @brief    更新处理
 ******************************************************************************/
void CModController::UpdateHandler_() {

	// 检查模块状态
	if (moduleStatus == APP_RESET) return ;

	// 计算并应用重力补偿（在组件更新前执行）
	UpdateGravityComp_();

	// 更新组件
	comPitch1_.UpdateComponent();
	comPitch2_.UpdateComponent();
	comYaw_.UpdateComponent();
	comRocker_.UpdateComponent();  // 先更新摇杆，再更新Roll
	comRoll_.UpdateComponent();
	comPitchEnd_.UpdateComponent();
	comBuzzer_.UpdateComponent();


	// 摇杆X轴缩放到 -100 ~ +100（有效范围 = 总范围 - 死区，与死区重映射配合）
	{
		int32_t raw_x = comRocker_.rockerInfo.X;
		float normalized_x;
		if (raw_x > 0) {
			normalized_x = raw_x / static_cast<float>(comRocker_.x_range_pos - CONTROLLER_ROCKER_DEAD_ZONE) * 100.0f;
		} else {
			normalized_x = raw_x / static_cast<float>(comRocker_.x_range_neg - CONTROLLER_ROCKER_DEAD_ZONE) * 100.0f;
		}
		ControllerInfo.rocker_X = static_cast<int8_t>(std::clamp(
			comRocker_.x_dir * normalized_x, -100.0f, 100.0f));
	}
	// Y轴：非对称范围处理（有效范围 = 总范围 - 死区）
	{
		int32_t raw_y = comRocker_.rockerInfo.Y;
		float normalized_y;
		if (raw_y > 0) {
			normalized_y = raw_y / static_cast<float>(comRocker_.y_range_pos - CONTROLLER_ROCKER_DEAD_ZONE) * 100.0f;
		} else {
			normalized_y = raw_y / static_cast<float>(comRocker_.y_range_neg - CONTROLLER_ROCKER_DEAD_ZONE) * 100.0f;
		}
		ControllerInfo.rocker_Y = static_cast<int8_t>(std::clamp(
			comRocker_.y_dir * normalized_y, -100.0f, 100.0f));
	}
	ControllerInfo.rocker_Key = comRocker_.rockerInfo.Key_status;
	ControllerInfo.posit_yaw = CModController::CComYaw::MtrPositToPhyPosit(comYaw_.yawInfo.posit);
	ControllerInfo.posit_pitch1 = comPitch1_.pitch1Info.posit;
	ControllerInfo.posit_pitch2 = comPitch2_.pitch2Info.posit;
	ControllerInfo.posit_roll = comRoll_.rollInfo.posit;         // MIT模式直接使用float位置
	ControllerInfo.posit_pitch_end = comPitchEnd_.pitchEndInfo.posit;  // 已在组件层转换为物理方向

	/*----------- Yaw电机（DJI M6020）CAN发送 -----------*/
	// Roll/PitchEnd改为MIT模式后在各自组件内发送，Pitch1/Pitch2也在组件内发送
	CDevMtrDJI::FillCanTxBuffer(comYaw_.motor[0],
							   comYaw_.mtrCanTxNode_[0]->dataBuffer,
							   comYaw_.mtrOutputBuffer[0]);

}

/******************************************************************************
 * @brief    心跳处理
 ******************************************************************************/
void CModController::HeartbeatHandler_() {

}

/******************************************************************************
 * @brief    创建控制器模块任务
 ******************************************************************************/
EAppStatus CModController::CreateModuleTask_() {

	// 任务已存在，删除任务
	if (moduleTaskHandle != nullptr) vTaskDelete(moduleTaskHandle);

	// 创建任务
	xTaskCreate(StartControllerModuleTask, "Controller Module Task",
						 512, this, proc_ModuleTaskPriority,
						  &moduleTaskHandle);

	return APP_OK;
}

/******************************************************************************
 * @brief    限制控制器模块的控制命令大小
 ******************************************************************************/
EAppStatus CModController::RestrictControllerCommand_() {
	
	// 检查模块状态
	if (moduleStatus == APP_RESET) {
		ControllerCmd = SControllerCmd();
		return APP_ERROR;
	}

	// 限制控制器各模块的控制命令大小
	ControllerCmd.cmd_yaw =
		std::clamp(ControllerCmd.cmd_yaw, CONTROLLER_YAW_PHYSICAL_RANGE_MIN, CONTROLLER_YAW_PHYSICAL_RANGE_MAX);
	ControllerCmd.cmd_pitch1 =
		std::clamp(ControllerCmd.cmd_pitch1, CONTROLLER_PITCH1_PHYSICAL_RANGE_MIN, CONTROLLER_PITCH1_PHYSICAL_RANGE_MAX);
	ControllerCmd.cmd_pitch2 =
		std::clamp(ControllerCmd.cmd_pitch2, CONTROLLER_PITCH2_PHYSICAL_RANGE_MIN, CONTROLLER_PITCH2_PHYSICAL_RANGE_MAX);
	ControllerCmd.cmd_roll =
		std::clamp(ControllerCmd.cmd_roll, CONTROLLER_ROLL_PHYSICAL_RANGE_MIN, CONTROLLER_ROLL_PHYSICAL_RANGE_MAX);
	ControllerCmd.cmd_pitch_end =
		std::clamp(ControllerCmd.cmd_pitch_end, CONTROLLER_PITCH_END_PHYSICAL_RANGE_MIN, CONTROLLER_PITCH_END_PHYSICAL_RANGE_MAX);

	return APP_OK;

}

/******************************************************************************
 * @brief    计算并应用重力补偿 + 虚拟阻尼
 ******************************************************************************/
void CModController::UpdateGravityComp_() {
	// 清零TF lambada函数
	auto clearTorques = [this]() {
		comPitch1_.pitch1Cmd.setParam[EMotorParam::TF] = 0.0f;
		comPitch2_.pitch2Cmd.setParam[EMotorParam::TF] = 0.0f;
		comRoll_.rollCmd.setParam[EMotorParam::TF] = 0.0f;
		comPitchEnd_.pitchEndCmd.setParam[EMotorParam::TF] = 0.0f;
	};

	// 未使能或组件未就绪时清零并退出
	if (!gravityCompEnabled_) { clearTorques(); return; }
	if (comPitch1_.componentStatus != APP_OK) { clearTorques(); return; }
	if (comPitch2_.componentStatus != APP_OK) { clearTorques(); return; }
	if (comRoll_.componentStatus != APP_OK) { clearTorques(); return; }
	if (comPitchEnd_.componentStatus != APP_OK) { clearTorques(); return; }

	constexpr float DEG2RAD = PI / 180.0f;

	// 物理角度(deg) -> 编码器角度(rad)
	// 注意：enc_pitch1 取反，因为实际Link2绝对角度 = P2 - P1（不是 P1 + P2）
	// DH模型中 q23 = q2 + q3，取反后变成 q23 = -q2 + q3 = q3 - q2，符合物理关系
	float enc_pitch1 = -(comPitch1_.pitch1Info.posit - CONTROLLER_GRAV_COMP_PITCH1_OFFSET) * DEG2RAD;
	float enc_pitch2 = (comPitch2_.pitch2Info.posit - CONTROLLER_GRAV_COMP_PITCH2_OFFSET) * DEG2RAD;
	float enc_roll = (comRoll_.rollInfo.posit - CONTROLLER_GRAV_COMP_ROLL_OFFSET) * DEG2RAD;
	// PitchEnd: posit 已在组件层转换为物理方向（向上为正）
	float enc_pitchEnd = (comPitchEnd_.pitchEndInfo.posit - CONTROLLER_GRAV_COMP_PITCHEND_OFFSET) * DEG2RAD;

	// 计算重力补偿力矩
	auto torques = gravityComp_.Calculate(enc_pitch1, enc_pitch2, enc_roll, enc_pitchEnd);

	/*---------------------虚拟阻尼计算---------------------*/
	float dampingTorque_pitchEnd = 0.0f;
	float dampingTorque_roll = 0.0f;

	// PitchEnd 虚拟阻尼
	if (comPitchEnd_.pitchEndCmd.isFree) {
		// 从电机反馈读取速度 (MIT模式12bit编码 → rad/s)
		float rawVelocity = CDevMtrDM::uint_to_float(
			comPitchEnd_.motor[0]->motorData[CDevMtr::DATA_SPEED],
			-comPitchEnd_.motor[0]->mitLimit_.DQ_MAX,
			comPitchEnd_.motor[0]->mitLimit_.DQ_MAX, 12);
		dampingTorque_pitchEnd = gravityComp_.CalculateDamping_PitchEnd(rawVelocity);
	}

	// Roll 虚拟阻尼
	if (comRoll_.rollCmd.isFree) {
		float rawVelocity = CDevMtrDM::uint_to_float(
			comRoll_.motor[0]->motorData[CDevMtr::DATA_SPEED],
			-comRoll_.motor[0]->mitLimit_.DQ_MAX,
			comRoll_.motor[0]->mitLimit_.DQ_MAX, 12);
		dampingTorque_roll = gravityComp_.CalculateDamping_Roll(rawVelocity);
	}

	// 非示教模式时重置滤波器
	if (!comPitchEnd_.pitchEndCmd.isFree && !comRoll_.rollCmd.isFree) {
		gravityComp_.ResetDampingFilters();
	}

	// 应用补偿力矩
	// DM4310 (Pitch1)
	comPitch1_.pitch1Cmd.setParam[EMotorParam::TF] =
		CONTROLLER_PITCH1_MOTOR_DIR * std::clamp(
			torques.tau_pitch1 * CONTROLLER_PITCH1_EFFICIENCY_COMP / CONTROLLER_GEAR_RATIO_DM4310,
			-CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310, CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310);
	// DM4310 (Pitch2): 独立效率补偿
	comPitch2_.pitch2Cmd.setParam[EMotorParam::TF] =
		CONTROLLER_PITCH2_MOTOR_DIR * std::clamp(
			torques.tau_pitch2 * CONTROLLER_PITCH2_EFFICIENCY_COMP / CONTROLLER_GEAR_RATIO_DM4310,
			-CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310, CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310);
	// DM3510 (Roll): 无重力补偿，仅虚拟阻尼
	if (comRoll_.rollCmd.isFree) {
		comRoll_.rollCmd.setParam[EMotorParam::TF] = dampingTorque_roll;
	} else {
		comRoll_.rollCmd.setParam[EMotorParam::TF] = 0.0f;
	}

	// DM3510 (PitchEnd):
	// - 示教模式(isFree): 重力补偿 + 虚拟阻尼
	// - 位控模式(!isFree): 仅重力补偿
	// 注意：重力补偿是物理坐标系，需要乘MOTOR_DIR；阻尼已是电机坐标系，不需要转换

	// [暂时禁用末端PitchEnd重力补偿]
	// float rollPhysical = comRoll_.rollInfo.posit;
	// bool inGimbalLockZone = (rollPhysical >= 80.0f && rollPhysical <= 100.0f) ||
	//                         (rollPhysical >= -100.0f && rollPhysical <= -80.0f);
	//
	// float gravTorque_pitchEnd = 0.0f;
	// if (!inGimbalLockZone) {
	// 	gravTorque_pitchEnd = CONTROLLER_PITCH_END_MOTOR_DIR * CONTROLLER_PITCHEND_EFFICIENCY_COMP *
	// 		std::clamp(torques.tau_pitchEnd, -CONTROLLER_GRAV_COMP_TAU_LIMIT_DM3510, CONTROLLER_GRAV_COMP_TAU_LIMIT_DM3510);
	// }
	//
	// float totalTorque_pitchEnd = gravTorque_pitchEnd;
	// if (comPitchEnd_.pitchEndCmd.isFree) {
	// 	totalTorque_pitchEnd += dampingTorque_pitchEnd;
	// }
	// comPitchEnd_.pitchEndCmd.setParam[EMotorParam::TF] = totalTorque_pitchEnd;
	comPitchEnd_.pitchEndCmd.setParam[EMotorParam::TF] = 0.0f;
}


} // namespace my_engineer
