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

CModController *pController_test;

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
	gravityCompEnabled_ = true;  // 默认开启


	// 创建任务并注册模块
	CreateModuleTask_();
	RegisterModule_();

	// test
	pController_test = this;

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


	// 在模块层将各各设备的编码器信息汇总
	// 摇杆值缩放到 -100 ~ +100
	ControllerInfo.rocker_X = -static_cast<int8_t>(comRocker_.rockerInfo.X / static_cast<float>(CONTROLLER_ROCKER_X_HALF_RANGE) * 100.0f);
	ControllerInfo.rocker_Y = -static_cast<int8_t>(comRocker_.rockerInfo.Y / static_cast<float>(CONTROLLER_ROCKER_Y_HALF_RANGE) * 100.0f);
	ControllerInfo.rocker_Key = comRocker_.rockerInfo.Key_status;
	ControllerInfo.posit_yaw = CModController::CComYaw::MtrPositToPhyPosit(comYaw_.yawInfo.posit);
	ControllerInfo.posit_pitch1 = comPitch1_.pitch1Info.posit;
	ControllerInfo.posit_pitch2 = comPitch2_.pitch2Info.posit;
	ControllerInfo.posit_roll = comRoll_.rollInfo.posit;         // MIT模式直接使用float位置
	ControllerInfo.posit_pitch_end = comPitchEnd_.pitchEndInfo.posit;  // MIT模式直接使用float位置

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
 * @brief    计算并应用重力补偿
 ******************************************************************************/
void CModController::UpdateGravityComp_() {
	// 清零TF辅助函数
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
	float enc_pitch1 = (comPitch1_.pitch1Info.posit - CONTROLLER_GRAV_COMP_PITCH1_OFFSET) * DEG2RAD;
	float enc_pitch2 = (comPitch2_.pitch2Info.posit - CONTROLLER_GRAV_COMP_PITCH2_OFFSET) * DEG2RAD;
	float enc_roll = (comRoll_.rollInfo.posit - CONTROLLER_GRAV_COMP_ROLL_OFFSET) * DEG2RAD;
	float enc_pitchEnd = (comPitchEnd_.pitchEndInfo.posit - CONTROLLER_GRAV_COMP_PITCHEND_OFFSET) * DEG2RAD;

	// 计算补偿力矩
	auto torques = gravityComp_.Calculate(enc_pitch1, enc_pitch2, enc_roll, enc_pitchEnd);

	// 应用补偿力矩（带限幅保护）
	comPitch1_.pitch1Cmd.setParam[EMotorParam::TF] =
		std::clamp(torques.tau_pitch1, -CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310, CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310);
	comPitch2_.pitch2Cmd.setParam[EMotorParam::TF] =
		std::clamp(torques.tau_pitch2, -CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310, CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310);
	comRoll_.rollCmd.setParam[EMotorParam::TF] =
		std::clamp(torques.tau_roll, -CONTROLLER_GRAV_COMP_TAU_LIMIT_DM3510, CONTROLLER_GRAV_COMP_TAU_LIMIT_DM3510);
	comPitchEnd_.pitchEndCmd.setParam[EMotorParam::TF] =
		std::clamp(torques.tau_pitchEnd, -CONTROLLER_GRAV_COMP_TAU_LIMIT_DM3510, CONTROLLER_GRAV_COMP_TAU_LIMIT_DM3510);
}


} // namespace my_engineer
