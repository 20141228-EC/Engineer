/******************************************************************************
 * @brief
 *
 * @file         mod_controller.cpp
 * @author       Fish_Joe (2328339747@qq.com), Ciallo
 * @version      V1.1
 * @date         2025-04-01
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-05-30
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#include "mod_controller.hpp"
#include "algo_other.hpp"

extern "C" {
volatile float dbg_gc_enc_pitch1_rad = 0.0f;
volatile float dbg_gc_enc_pitch2_rad = 0.0f;
volatile float dbg_gc_enc_roll_rad = 0.0f;
volatile float dbg_gc_enc_pitchEnd_rad = 0.0f;

volatile float dbg_gc_tau_pitch1 = 0.0f;
volatile float dbg_gc_tau_pitch2 = 0.0f;
volatile float dbg_gc_tau_roll = 0.0f;
volatile float dbg_gc_tau_pitchEnd = 0.0f;

volatile float dbg_gc_motor_tf_pitch1 = 0.0f;
volatile float dbg_gc_motor_tf_pitch2 = 0.0f;
volatile float dbg_gc_motor_tf_roll = 0.0f;
volatile float dbg_gc_motor_tf_pitchEnd = 0.0f;

volatile float dbg_motor_tau_pitch1 = 0.0f;
volatile float dbg_motor_tau_pitch2 = 0.0f;

volatile float dbg_motor_speed_pitch1 = 0.0f;
volatile float dbg_motor_speed_pitch2 = 0.0f;

// Gravity identification mode (J-Scope writable).
// 0=normal compensation, 1=hold target angle with all TF injection disabled.
volatile int dbg_gravity_ident_mode = 0;
volatile float dbg_ident_tau_pitch1 = 0.0f;
volatile float dbg_ident_tau_pitch2 = 0.0f;

// Position hold mode (J-Scope writable)
volatile int dbg_position_hold_mode = 0;     // 0=off, 1=position hold
volatile float dbg_hold_kp = 15.0f;           // MIT kp for position hold
volatile float dbg_hold_kd = 2.f;           // MIT kd for position hold
volatile float dbg_target_pitch1 = 0.0f;     // Target physical angle (deg)
volatile float dbg_target_pitch2 = 0.0f;
volatile float dbg_target_roll = 0.0f;
volatile float dbg_target_pitchEnd = 0.0f;
}

namespace my_engineer {

/*----------- Debug: 力反馈实际叠加力矩 (N·m) -----------*/
volatile float dbg_fbTF_pitch1 = 0;
volatile float dbg_fbTF_pitch2 = 0;
volatile float dbg_fbTF_roll   = 0;


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

	// 叠加力反馈（在重力补偿之后、组件更新之前）
	UpdateForceFeedback_();

	// 更新组件
	comPitch1_.UpdateComponent();
	comPitch2_.UpdateComponent();
	comYaw_.UpdateComponent();
	comRocker_.UpdateComponent();  // 先更新摇杆，再更新Roll
	comRoll_.UpdateComponent();
	comPitchEnd_.UpdateComponent();
	comBuzzer_.UpdateComponent();


	// 摇杆X轴缩放到 -100 ~ +100（有效范围 = 总范围 - 死区，与死区重映射配合）
	// Y轴：非对称范围处理（有效范围 = 总范围 - 死区）
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
 * @brief    计算并应用重力补偿与虚拟阻尼
 ******************************************************************************/
void CModController::UpdateGravityComp_() {
	// 清零TF lambda函数
	auto clearTorques = [this]() {
		comPitch1_.pitch1Cmd.setParam[EMotorParam::TF] = 0.0f;
		comPitch2_.pitch2Cmd.setParam[EMotorParam::TF] = 0.0f;
		comRoll_.rollCmd.setParam[EMotorParam::TF] = 0.0f;
		comPitchEnd_.pitchEndCmd.setParam[EMotorParam::TF] = 0.0f;
		dbg_gc_tau_pitch1 = 0.0f;
		dbg_gc_tau_pitch2 = 0.0f;
		dbg_gc_tau_roll = 0.0f;
		dbg_gc_tau_pitchEnd = 0.0f;
		dbg_gc_motor_tf_pitch1 = 0.0f;
		dbg_gc_motor_tf_pitch2 = 0.0f;
		dbg_gc_motor_tf_roll = 0.0f;
		dbg_gc_motor_tf_pitchEnd = 0.0f;
		dbg_ident_tau_pitch1 = 0.0f;
		dbg_ident_tau_pitch2 = 0.0f;
	};

	// 未使能或组件未就绪时清零并退出
	const bool identMode = (dbg_gravity_ident_mode != 0);
	if (!gravityCompEnabled_ && !identMode) { clearTorques(); return; }
	if (comPitch1_.componentStatus != APP_OK) { clearTorques(); return; }
	if (comPitch2_.componentStatus != APP_OK) { clearTorques(); return; }
	if (comRoll_.componentStatus != APP_OK) { clearTorques(); return; }
	if (comPitchEnd_.componentStatus != APP_OK) { clearTorques(); return; }

	constexpr float DEG2RAD = PI / 180.0f;

	// 物理角度(deg) -> DH角度(rad)
	// DH: q2=Pitch1, q3=Pitch2, q4=P3(removed), q5=Roll, q6=PitchEnd
	// 注意：enc_pitch1 取反，因为实际Link2绝对角度 = P2 - P1
	float enc_pitch1 = -(comPitch1_.pitch1Info.posit - CONTROLLER_GRAV_COMP_PITCH1_OFFSET) * DEG2RAD;
	float enc_pitch2 = (comPitch2_.pitch2Info.posit - CONTROLLER_GRAV_COMP_PITCH2_OFFSET) * DEG2RAD;
	float enc_roll = (comRoll_.rollInfo.posit - CONTROLLER_GRAV_COMP_ROLL_OFFSET) * DEG2RAD;
	// PitchEnd: posit 已在组件层转换为物理方向（向上为正）
	float enc_pitchEnd = (comPitchEnd_.pitchEndInfo.posit - CONTROLLER_GRAV_COMP_PITCHEND_OFFSET) * DEG2RAD;

	dbg_gc_enc_pitch1_rad = enc_pitch1;
	dbg_gc_enc_pitch2_rad = enc_pitch2;
	dbg_gc_enc_roll_rad = enc_roll;
	dbg_gc_enc_pitchEnd_rad = enc_pitchEnd;

	// 计算重力补偿力矩 (P3已删除，传0.0f)
 	auto torques = gravityComp_.Calculate(enc_pitch1, enc_pitch2, 0.0f, enc_roll, enc_pitchEnd);

	dbg_gc_tau_pitch1 = torques.tau_pitch1;
	dbg_gc_tau_pitch2 = torques.tau_pitch2;
	dbg_gc_tau_roll = torques.tau_roll;
	dbg_gc_tau_pitchEnd = torques.tau_pitchEnd;

	/*---------------------虚拟阻尼计算---------------------*/
	float dampingTorque_roll = 0.0f;

	// Roll 虚拟阻尼
	if (!identMode && comRoll_.rollCmd.isFree) {
		float rawVelocity = CDevMtrDM::uint_to_float(
			comRoll_.motor[0]->motorData[CDevMtr::DATA_SPEED],
			-comRoll_.motor[0]->mitLimit_.DQ_MAX,
			comRoll_.motor[0]->mitLimit_.DQ_MAX, 12);
		dampingTorque_roll = gravityComp_.CalculateDamping_Roll(rawVelocity);
	}

	// 非示教模式时重置滤波器
	if (identMode || (!comPitchEnd_.pitchEndCmd.isFree && !comRoll_.rollCmd.isFree)) {
		gravityComp_.ResetDampingFilters();
	}

	// 应用补偿力矩
	// DM4310 (Pitch1): 仅示教模式下施加重力补偿，锁定时清零
	if (!identMode && comPitch1_.pitch1Cmd.isFree) {
		comPitch1_.pitch1Cmd.setParam[EMotorParam::TF] =
			CONTROLLER_PITCH1_MOTOR_DIR * std::clamp(
				torques.tau_pitch1 * CONTROLLER_PITCH1_EFFICIENCY_COMP / CONTROLLER_GEAR_RATIO_DM4310,
				-CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310, CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310);
	} else {
		comPitch1_.pitch1Cmd.setParam[EMotorParam::TF] = 0.0f;
	}
	// DM4310 (Pitch2): 仅示教模式下施加重力补偿，锁定时清零
	if (!identMode && comPitch2_.pitch2Cmd.isFree) {
		comPitch2_.pitch2Cmd.setParam[EMotorParam::TF] =
			CONTROLLER_PITCH2_MOTOR_DIR * std::clamp(
				torques.tau_pitch2 * CONTROLLER_PITCH2_EFFICIENCY_COMP / CONTROLLER_GEAR_RATIO_DM4310,
				-CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310, CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310);
	} else {
		comPitch2_.pitch2Cmd.setParam[EMotorParam::TF] = 0.0f;
	}

	// DM3510 (Roll): 无重力补偿，仅虚拟阻尼
	float totalTorque_roll = 0.0f;
	if (!identMode && comRoll_.rollCmd.isFree) {
		totalTorque_roll = dampingTorque_roll;
	}
	comRoll_.rollCmd.setParam[EMotorParam::TF] = totalTorque_roll;

	// DM3510 (PitchEnd): 无重力补偿，虚拟阻尼已关闭
	float totalTorque_pitchEnd = 0.0f;
	comPitchEnd_.pitchEndCmd.setParam[EMotorParam::TF] = totalTorque_pitchEnd;

	dbg_gc_motor_tf_pitch1 = comPitch1_.pitch1Cmd.setParam[EMotorParam::TF];
	dbg_gc_motor_tf_pitch2 = comPitch2_.pitch2Cmd.setParam[EMotorParam::TF];
	dbg_gc_motor_tf_roll = comRoll_.rollCmd.setParam[EMotorParam::TF];
	dbg_gc_motor_tf_pitchEnd = comPitchEnd_.pitchEndCmd.setParam[EMotorParam::TF];

	auto *mP1 = comPitch1_.motor[0];
	auto *mP2 = comPitch2_.motor[0];

	// ----- 关节侧重力矩 -----
      float tau_m_p1 = CDevMtrDM::uint_to_float(
          mP1->motorData[CDevMtr::DATA_TORQUE],
          -mP1->mitLimit_.TAU_MAX, mP1->mitLimit_.TAU_MAX, 12);
      float tau_m_p2 = CDevMtrDM::uint_to_float(
          mP2->motorData[CDevMtr::DATA_TORQUE],
          -mP2->mitLimit_.TAU_MAX, mP2->mitLimit_.TAU_MAX, 12);

      // DM4310 减速 10:1, DM3510 直驱
      dbg_motor_tau_pitch1 = -tau_m_p1 * CONTROLLER_GEAR_RATIO_DM4310 * CONTROLLER_PITCH1_MOTOR_DIR;
      dbg_motor_tau_pitch2 = -tau_m_p2 * CONTROLLER_GEAR_RATIO_DM4310 * CONTROLLER_PITCH2_MOTOR_DIR;

      // ----- 关节侧速度 (rad/s) -----
      // motorData[DATA_SPEED] 是 12-bit raw, 中位 2047 = 0
      dbg_motor_speed_pitch1 = CDevMtrDM::uint_to_float(
          mP1->motorData[CDevMtr::DATA_SPEED],
          -mP1->mitLimit_.DQ_MAX, mP1->mitLimit_.DQ_MAX, 12)
          * CONTROLLER_PITCH1_MOTOR_DIR / CONTROLLER_GEAR_RATIO_DM4310;
      dbg_motor_speed_pitch2 = CDevMtrDM::uint_to_float(
          mP2->motorData[CDevMtr::DATA_SPEED],
          -mP2->mitLimit_.DQ_MAX, mP2->mitLimit_.DQ_MAX, 12)
          * CONTROLLER_PITCH2_MOTOR_DIR / CONTROLLER_GEAR_RATIO_DM4310;

      if (identMode) {
          dbg_ident_tau_pitch1 = dbg_motor_tau_pitch1;
          dbg_ident_tau_pitch2 = dbg_motor_tau_pitch2;
      } else {
          dbg_ident_tau_pitch1 = 0.0f;
          dbg_ident_tau_pitch2 = 0.0f;
      }

}

/******************************************************************************
 * @brief    力反馈：将机器人回传的力矩叠加到示教模式的 TF 前馈上
 *
 * 仅在示教模式(isFree=true)下生效，操作者能感受到机器人端的阻力。
 * 力反馈增益 fbGain_ 各轴独立控制反馈强度，需根据实际的情况调整参数。
 ******************************************************************************/
void CModController::UpdateForceFeedback_() {
	if (dbg_gravity_ident_mode != 0) {
		dbg_fbTF_pitch1 = 0.0f;
		dbg_fbTF_pitch2 = 0.0f;
		dbg_fbTF_roll = 0.0f;
		return;
	}

	if (!forceFeedbackEnabled_ || !ControllerCmd.isFree) return; //联动模式并且力反馈没有初始化

	if(ControllerInfo.isRobotInit == true){
	// 死区：过滤静态重力补偿力矩，只反馈碰撞外力
	constexpr float DEADZONE_P1 = 5.0f;   // Pitch1 死区 (N·m)
	constexpr float DEADZONE_P2 = 15.0f;   // Pitch2 死区 (N·m)
	constexpr float DEADZONE_R  = 0.1f;   // Roll 死区 (N·m)
	auto applyDeadzone = [](float val, float dz) -> float {
		if (val > dz) return val - dz;
		if (val < -dz) return val + dz;
		return 0.0f;
	};

	// 滑动窗口滤波（窗口100，1kHz下覆盖一个10Hz通信周期）
	static CMovingAvgFilter<100> filter_pitch1;
	static CMovingAvgFilter<100> filter_pitch2;
	static CMovingAvgFilter<100> filter_roll;

	float avg_p1 = filter_pitch1.Update(applyDeadzone(ControllerCmd.fb_torque_pitch1, DEADZONE_P1));
	float avg_p2 = filter_pitch2.Update(applyDeadzone(ControllerCmd.fb_torque_pitch2, DEADZONE_P2));
	float avg_r  = filter_roll.Update(applyDeadzone(ControllerCmd.fb_torque_roll, DEADZONE_R));

	// 各轴独立增益
	dbg_fbTF_pitch1 = fbGain_.pitch1 * avg_p1;
	dbg_fbTF_pitch2 = fbGain_.pitch2 * avg_p2;
	dbg_fbTF_roll   = fbGain_.roll   * avg_r;

	comPitch1_.pitch1Cmd.setParam[EMotorParam::TF] += dbg_fbTF_pitch1;
	comPitch2_.pitch2Cmd.setParam[EMotorParam::TF] += dbg_fbTF_pitch2;
	comRoll_.rollCmd.setParam[EMotorParam::TF]     += dbg_fbTF_roll;

	dbg_gc_motor_tf_pitch1 = comPitch1_.pitch1Cmd.setParam[EMotorParam::TF];
	dbg_gc_motor_tf_pitch2 = comPitch2_.pitch2Cmd.setParam[EMotorParam::TF];
	dbg_gc_motor_tf_roll = comRoll_.rollCmd.setParam[EMotorParam::TF];
	}

	// PitchEnd: 暂不处理（末端差速）
}


} // namespace my_engineer
