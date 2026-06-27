/**
 * @file mod_gimbal.cpp
 * @author ciallo
 * @brief 云台模块
 * @version 3.0
 * @date 2026-06-07
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mod_gimbal.hpp"

namespace my_engineer {

CModGimbal *pGimbal_test = nullptr; ///< 调试用全局指针

/**
 * @brief 初始化云台模块
 *
 */
EAppStatus CModGimbal::InitModule(SModInitParam_Base &param){
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	auto gimbalParam = static_cast<SModInitParam_Gimbal &>(param);
	moduleID = gimbalParam.moduleID;

	comVisualyaw_.InitComponent(param);
	comGimbalPitch_.InitComponent(param);

	CreateModuleTask_();
	RegisterModule_();

	pGimbal_test = this;

	Module_FSMFlag_ = FSM_RESET;
	moduleStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新处理 (1000Hz, 由主循环调用)
 *
 */
void CModGimbal::UpdateHandler_(){
	static uint8_t TickRate = 0;
	if (++TickRate >= 4) TickRate = 0;
	if (moduleStatus == APP_RESET) return;

	// 云台 visualYaw 与pitch 都以 250Hz 更新
	if(TickRate == 0) {
		comVisualyaw_.UpdateComponent();   // 250Hz DM_MIT
		comGimbalPitch_.UpdateComponent(); // 250Hz DJI M2006

		CDevMtrDJI::FillCanTxBuffer(comGimbalPitch_.motor,
		                            comGimbalPitch_.mtrCanTxNode->dataBuffer,
		                            comGimbalPitch_.mtrOutputBuffer);
	}

	// 将组件信息同步到模块级信息结构体, 供 SystemCore 
	gimbalInfo.angle_visualyaw = comVisualyaw_.VisuallyawInfo.angle;
	gimbalInfo.isPositArrived_Visualyaw = comVisualyaw_.VisuallyawInfo.isAngleArrived;
	gimbalInfo.angle_pitch = comGimbalPitch_.MtrPositToPhyPosit(comGimbalPitch_.pitchInfo.posit);
	gimbalInfo.isPositArrived_Pitch = comGimbalPitch_.pitchInfo.isPositArrived;
}

/**
 * @brief 心跳处理 (100Hz)
 */
void CModGimbal::HeartbeatHandler_(){

}

/**
 * @brief 创建云台模块FreeRTOS任务
 *
 * @return EAppStatus APP_OK
 */
EAppStatus CModGimbal::CreateModuleTask_(){
	if (moduleTaskHandle != nullptr) vTaskDelete(moduleTaskHandle);

	xTaskCreate(StartGimbalModuleTask, "Gimbal Module Task",
                         512, this, proc_ModuleTaskPriority,
						 &moduleTaskHandle);

	return APP_OK;
}

/**
 * @brief 限制云台模块的控制命令范围
 *
 * @note 图传yaw限制在 0~360 度范围
 */
EAppStatus CModGimbal::RestrictGimbalCommand_(){
	if (moduleStatus == APP_RESET) {
		gimbalCmd = SGimbalCmd();
		return APP_ERROR;
	}

	gimbalCmd.set_visualyaw = std::clamp(gimbalCmd.set_visualyaw, 0.f, 179.f);

	gimbalCmd.set_pitch = std::clamp(gimbalCmd.set_pitch, -55.0f, 55.0f);

	if (gimbalCmd.isAutoCtrl) return APP_OK;

	return APP_OK;
}

} // namespace my_engineer
