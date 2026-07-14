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
// 重力辨识模式 (J-Scope 可写): 0=正常补偿, 1=保持目标角, 关闭所有前馈注入
volatile int dbg_gravity_ident_mode = 0;

// 位置保持模式 (J-Scope 可写, 辨识用)
volatile int dbg_position_hold_mode = 0;     // 0=off, 1=position hold
volatile float dbg_hold_kp = 15.0f;           // MIT kp for position hold
volatile float dbg_hold_kd = 2.f;           // MIT kd for position hold
volatile float dbg_target_pitch1 = 0.0f;     // 目标物理角度 (deg)
volatile float dbg_target_pitch2 = 0.0f;
volatile float dbg_target_roll = 0.0f;
volatile float dbg_target_pitchEnd = 0.0f;
}

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
	gravityComp_.InitComponent(controllerParam.gravParam);
	gravityComp_.SetMode(CAlgoGravityComp::CGravityCompMode::ENABLE);  // 启用重力补偿


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

	// 计算重力补偿并分发 grav_ff 到各组件（在组件更新前执行）
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
	// 清零 grav_ff lambda
	auto clearGravFf = [this]() {
		comPitch1_.grav_ff = 0.0f;
		comPitch2_.grav_ff = 0.0f;
		comRoll_.grav_ff = 0.0f;
		comPitchEnd_.grav_ff = 0.0f;
	};

	const bool identMode = (dbg_gravity_ident_mode != 0);

	// 辨识模式: 关闭重力补偿输出
	if (identMode) {
		clearGravFf();
		return;
	}

	// 未使能或组件未就绪时清零并退出
	if (gravityComp_.GetMode() == CAlgoGravityComp::CGravityCompMode::NONE) { clearGravFf(); return; }
	if (comPitch1_.componentStatus != APP_OK) { clearGravFf(); return; }
	if (comPitch2_.componentStatus != APP_OK) { clearGravFf(); return; }
	if (comRoll_.componentStatus != APP_OK) { clearGravFf(); return; }
	if (comPitchEnd_.componentStatus != APP_OK) { clearGravFf(); return; }

	// 组装输入状态
	gravState_.pitch1_deg = comPitch1_.pitch1Info.posit;
	gravState_.pitch2_deg = comPitch2_.pitch2Info.posit;
	gravState_.roll_deg = comRoll_.rollInfo.posit;
	gravState_.pitch_end_deg = comPitchEnd_.pitchEndInfo.posit;

	// 计算重力补偿
	gravOut_ = gravityComp_.Calc(gravState_);

	// 分发 grav_ff 到各组件 (组件在 _UpdateOutput 时叠加到 TF)
	// 非示教模式(isFree=false)锁定时候清零
	comPitch1_.grav_ff = comPitch1_.pitch1Cmd.isFree ? gravOut_.pitch1_ff : 0.0f;
	comPitch2_.grav_ff = comPitch2_.pitch2Cmd.isFree ? gravOut_.pitch2_ff : 0.0f;
	comRoll_.grav_ff = comRoll_.rollCmd.isFree ? gravOut_.roll_ff : 0.0f;
	comPitchEnd_.grav_ff = comPitchEnd_.pitchEndCmd.isFree ? gravOut_.pitch_end_ff : 0.0f;
}

} // namespace my_engineer
