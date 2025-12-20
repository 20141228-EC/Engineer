/**
 * @file mod_gimbal.cpp
 * @author Ciallo～(∠·ω< )⌒☆
 * @brief 云台模块
 * @version 2.0
 * @date 2025-12-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "mod_gimbal.hpp"

namespace my_engineer {

CModGimbal *pGimbal_test = nullptr;

/**
 * @brief 初始化云台模块
 *
 * @param param
 * @return EAppStatus
 */
EAppStatus CModGimbal::InitModule(SModInitParam_Base &param){
	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto gimbalParam = static_cast<SModInitParam_Gimbal &>(param);
	moduleID = gimbalParam.moduleID;

	// 初始化云台升降组件 (双电机同步)
	comLift_.InitComponent(param);

	// 初始化云台俯仰组件 (单电机)
	comPitch_.InitComponent(param);

	// 创建任务并注册模块
	CreateModuleTask_();
	RegisterModule_();

	// test
	pGimbal_test = this;

	Module_FSMFlag_ = FSM_RESET;
	moduleStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新处理
 *
 */
void CModGimbal::UpdateHandler_(){
	// 检查模块状态
	if (moduleStatus == APP_RESET) return;

	// 更新所有组件
	comLift_.UpdateComponent();
	comPitch_.UpdateComponent();

	// 更新模块信息 - 升降
	gimbalInfo.posit_lift =
		comLift_.MtrPositToPhyPosit(comLift_.liftInfo.posit);
	gimbalInfo.isPositArrived_Lift = comLift_.liftInfo.isPositArrived;

	// 更新模块信息 - 俯仰
	gimbalInfo.posit_pitch =
		comPitch_.MtrPositToPhyPosit(comPitch_.pitchInfo.posit);
	gimbalInfo.isPositArrived_Pitch = comPitch_.pitchInfo.isPositArrived;

	// 填充数据发送缓冲区 - 升降左电机
	CDevMtrDJI::FillCanTxBuffer(comLift_.motor[CComLift::L],
								comLift_.mtrCanTxNode[CComLift::L]->dataBuffer,
								comLift_.mtrOutputBuffer[CComLift::L]);

	// 填充数据发送缓冲区 - 升降右电机
	CDevMtrDJI::FillCanTxBuffer(comLift_.motor[CComLift::R],
								comLift_.mtrCanTxNode[CComLift::R]->dataBuffer,
								comLift_.mtrOutputBuffer[CComLift::R]);

	// 填充数据发送缓冲区 - 俯仰电机
	CDevMtrDJI::FillCanTxBuffer(comPitch_.motor,
								comPitch_.mtrCanTxNode->dataBuffer,
								comPitch_.mtrOutputBuffer[0]);

}

/**
 * @brief 心跳处理
 *
 */
void CModGimbal::HeartbeatHandler_(){

}

/**
 * @brief 创建云台任务
 *
 * @param argument
 */
EAppStatus CModGimbal::CreateModuleTask_(){
	// 任务已存在，删除任务
	if (moduleTaskHandle != nullptr) vTaskDelete(moduleTaskHandle);

	// 创建任务
	xTaskCreate(StartGimbalModuleTask, "Gimbal Module Task",
						 512, this, proc_ModuleTaskPriority,
						  &moduleTaskHandle);

	return APP_OK;
}

/**
 * @brief 限制云台模块的控制命令大小
 *
 */
EAppStatus CModGimbal::RestrictGimbalCommand_(){

	// 检查模块状态
	if (moduleStatus == APP_RESET) {
		gimbalCmd = SGimbalCmd();
		return APP_ERROR;
	}

	// 限制升降控制命令
	gimbalCmd.set_posit_lift =
		std::clamp(gimbalCmd.set_posit_lift, 0.0f, GIMBAL_LIFT_PHYSICAL_RANGE);

	// 限制俯仰控制命令
	gimbalCmd.set_posit_pitch =
		std::clamp(gimbalCmd.set_posit_pitch, 0.0f, GIMBAL_PITCH_PHYSICAL_RANGE);

	// 自动控制启用，则不继续做限制
	if (gimbalCmd.isAutoCtrl) return APP_OK;


	return APP_OK;

}

} // namespace my_engineer

