/******************************************************************************
 * @brief        
 * 
 * @file         mod_arm.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-05-04
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "mod_arm.hpp"



namespace my_engineer {

CModArm *pArm_test = nullptr;

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
	comEnd_.InitComponent(param);

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
	// static uint8_t HalfTickRate = 0;
	// HalfTickRate = 1 - HalfTickRate;
	if (moduleStatus == APP_RESET) return;
	
	// if(HalfTickRate) {	}
	// 更新组件
	comjoint_.UpdateComponent();
	comEnd_.UpdateComponent();
	comRoll_.UpdateComponent();

	// 更新模块信息
	armInfo.angle_Yaw = comjoint_.MtrPositToPhyPosit_yaw(comjoint_.jointInfo.posit_yaw);
	armInfo.angle_Pitch1 = comjoint_.MtrPositToPhyPosit_pitch1(comjoint_.jointInfo.posit_pitch1);
	armInfo.angle_Pitch2 = comjoint_.MtrPositToPhyPosit_pitch2(comjoint_.jointInfo.posit_pitch2);
	armInfo.angle_Roll = comRoll_.MtrAngleToPhyAngle(comRoll_.rollInfo.angle);
	armInfo.angle_end_pitch =
		comEnd_.MtrPositToPhyPosit_Pitch(comEnd_.endInfo.posit_Pitch);
	armInfo.angle_end_roll =
		comEnd_.MtrPositToPhyPosit_Roll(comEnd_.endInfo.posit_Roll);
	armInfo.isAngleArrived_Yaw = comjoint_.jointInfo.isPositArrived_yaw;
	armInfo.isAngleArrived_Pitch1 = comjoint_.jointInfo.isPositArrived_pitch1;
	armInfo.isAngleArrived_Pitch2 = comjoint_.jointInfo.isPositArrived_pitch2;
	armInfo.isAngleArrived_Roll = comRoll_.rollInfo.isAngleArrived;
	armInfo.isAngleArrived_End_Pitch = comEnd_.endInfo.isPositArrived_Pitch;
	armInfo.isAngleArrived_End_Roll = comEnd_.endInfo.isPositArrived_Roll;

	// 填充电机发送缓冲区
	CDevMtrKT::FillCanTxBuffer(comjoint_.motor[CComJoint::P1],
								comjoint_.mtrCanTxNode[CComJoint::P1]->dataBuffer,
								comjoint_.mtrOutputBuffer[CComJoint::P1]);
	CDevMtrKT::FillCanTxBuffer(comjoint_.motor[CComJoint::P2],
								comjoint_.mtrCanTxNode[CComJoint::P2]->dataBuffer,
								comjoint_.mtrOutputBuffer[CComJoint::P2]);
	CDevMtrKT::FillCanTxBuffer(comjoint_.motor[CComJoint::Y],
								comjoint_.mtrCanTxNode[CComJoint::Y]->dataBuffer,
								comjoint_.mtrOutputBuffer[CComJoint::Y]);
	CDevMtrDJI::FillCanTxBuffer(comEnd_.motor[CComEnd::L],
								comEnd_.mtrCanTxNode[CComEnd::L]->dataBuffer,
								comEnd_.mtrOutputBuffer[CComEnd::L]);
	CDevMtrDJI::FillCanTxBuffer(comEnd_.motor[CComEnd::R],
								comEnd_.mtrCanTxNode[CComEnd::R]->dataBuffer,
								comEnd_.mtrOutputBuffer[CComEnd::R]);

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

	// 限制控制命令大小
	armCmd.set_angle_Yaw =
		std::clamp(armCmd.set_angle_Yaw, ARM_YAW_PHYSICAL_RANGE_MIN, ARM_YAW_PHYSICAL_RANGE_MAX);
	armCmd.set_angle_Pitch1 =
		std::clamp(armCmd.set_angle_Pitch1,
				   ARM_PITCH1_PHYSICAL_RANGE_MIN, ARM_PITCH1_PHYSICAL_RANGE_MAX);
	if (armCmd.set_angle_Pitch2 < ARM_PITCH2_PHYSICAL_RANGE_MIN) {
		armCmd.set_angle_Pitch2 = ARM_PITCH2_PHYSICAL_RANGE_MIN;
	} else {
		armCmd.set_angle_Pitch2 =
			std::clamp(armCmd.set_angle_Pitch2,
				ARM_PITCH2_PHYSICAL_RANGE_MIN, 1.25f * armCmd.set_angle_Pitch1);
	}
	if (armCmd.set_angle_Pitch2 > ARM_PITCH2_PHYSICAL_RANGE_MAX) {
		armCmd.set_angle_Pitch2 = ARM_PITCH2_PHYSICAL_RANGE_MAX;
	}
	armCmd.set_angle_Roll =
		std::clamp(armCmd.set_angle_Roll,
				   ARM_ROLL_PHYSICAL_RANGE_MIN, ARM_ROLL_PHYSICAL_RANGE_MAX);
	armCmd.set_angle_end_pitch =
		std::clamp(armCmd.set_angle_end_pitch,
				   ARM_END_PITCH_PHYSICAL_RANGE_MIN, ARM_END_PITCH_PHYSICAL_RANGE_MAX);

	if(armCmd.isCustomCtrl)
	armCmd.set_angle_Pitch1 =
		std::clamp(armCmd.set_angle_Pitch1,
				   18.0f, ARM_PITCH1_PHYSICAL_RANGE_MAX);

	// 自动控制启用，则不继续做限制
	if (armCmd.isAutoCtrl) return APP_OK;

	if (armCmd.set_angle_Pitch1 >= 65.0f) {
		armCmd.set_angle_Pitch2 = std::clamp(armCmd.set_angle_Pitch2,
			32.0f, 32.0f + armCmd.set_angle_Pitch1);
	}
	
	if (armCmd.set_angle_Pitch1 < 45.0f) {
		armCmd.set_angle_Yaw = std::clamp(armCmd.set_angle_Yaw,
			-12.0f, 5.0f);
	}

	if (armCmd.set_angle_Pitch2 < 20.0f) {
		armCmd.set_angle_Roll = std::clamp(armCmd.set_angle_Roll,
			-15.0f, 15.0f);
	}

	if (should_limit_yaw) {
		armCmd.set_angle_Yaw = std::clamp(armCmd.set_angle_Yaw,
			-12.0f, 6.0f);
	}

	return APP_OK;
}

} // namespace my_engineer
