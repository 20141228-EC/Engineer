/**
 * @file mod_gimbal.cpp
 * @author sllllr
 * @brief 云台模块
 * @version 1.0
 * @date 2026-03-06
 *
 * @copyright Copyright (c) 2026
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

	// 初始化云台
	comYaw_.InitComponent(param);

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
	comYaw_.UpdateComponent();

	// 更新模块信息
	gimbalInfo.posit_yaw = comYaw_.yawInfo.posit;
	gimbalInfo.encoder_yaw = comYaw_.yawInfo.encoder;
	gimbalInfo.isPositArrived = comYaw_.yawInfo.isPositArrived;

	// 填充数据发送缓冲区
	CDevMtrKT::FillCanTxBuffer(comYaw_.motor,
								comYaw_.mtrCanTxNode->dataBuffer,
								comYaw_.mtrOutputBuffer);
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

	// 限制控制命令
	// gimbalCmd.set_posit_yaw =
	// 	std::clamp(gimbalCmd.set_posit_yaw, 0.0f, GIMBAL_YAW_PHYSICAL_RANGE);

	// 自动控制启用，则不继续做限制
	if (gimbalCmd.isAutoCtrl) return APP_OK;


	return APP_OK;

}

} // namespace my_engineer

