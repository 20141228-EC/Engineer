/**
 * @file mod_gimbal.cpp
 * @author ciallo
 * @brief 云台模块
 * @version 2.0
 * @date 2026-03-06
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
	comstorage_.InitComponent(param);

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
	static uint8_t HalfTickRate = 0;
	HalfTickRate = 1 - HalfTickRate;
	if (moduleStatus == APP_RESET) return;

	if(HalfTickRate) {
		comVisualyaw_.UpdateComponent();
		comstorage_.UpdateComponent();
	}//500hz

	// 将组件信息同步到模块级信息结构体, 供 SystemCore 读取
	gimbalInfo.angle_visualyaw = comVisualyaw_.VisuallyawInfo.angle;
	gimbalInfo.isPositArrived_Visualyaw = comVisualyaw_.VisuallyawInfo.isAngleArrived;
	gimbalInfo.posit_storage_L = comstorage_.storageInfo.posit_L_storage;
	gimbalInfo.posit_storage_R = comstorage_.storageInfo.posit_R_storage;
	gimbalInfo.isPositArrived_Storage_L = comstorage_.storageInfo.isPositArrived_L_storage;
	gimbalInfo.isPositArrived_Storage_R = comstorage_.storageInfo.isPositArrived_R_storage;
	gimbalInfo.isStorageAvailable = comstorage_.storageInfo.isAvailable;
	CDevMtrKT::FillCanTxBuffer(comstorage_.motor[CComStorage::L],
								   comstorage_.mtrCanTxNode[CComStorage::L]->dataBuffer,
								   comstorage_.mtrOutputBuffer[CComStorage::L]);
	CDevMtrKT::FillCanTxBuffer(comstorage_.motor[CComStorage::R],
								   comstorage_.mtrCanTxNode[CComStorage::R]->dataBuffer,
								   comstorage_.mtrOutputBuffer[CComStorage::R]);
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
 * @return EAppStatus APP_OK=正常, APP_ERROR=模块复位状态
 *
 * @note 图传yaw限制在 0~360 度范围
 */
EAppStatus CModGimbal::RestrictGimbalCommand_(){
	if (moduleStatus == APP_RESET) {
		gimbalCmd = SGimbalCmd();
		return APP_ERROR;
	}

	gimbalCmd.set_visualyaw =
		std::clamp(gimbalCmd.set_visualyaw, -185.0f, 1.f);
	gimbalCmd.set_posit_storage_L = std::clamp(gimbalCmd.set_posit_storage_L,
			static_cast<int32_t>(-STORAGE_L_MOTOR_RANGE / 2),
			static_cast<int32_t>(STORAGE_L_MOTOR_RANGE / 2));
	gimbalCmd.set_posit_storage_R = std::clamp(gimbalCmd.set_posit_storage_R,
			static_cast<int32_t>(-STORAGE_R_MOTOR_RANGE / 2),
			static_cast<int32_t>(STORAGE_R_MOTOR_RANGE / 2));

	if (gimbalCmd.isAutoCtrl) return APP_OK;

	return APP_OK;
}
/**
 * @brief 对自动任务暴露的接口
 */
void CModGimbal::ChooseStoreOre(EStorageSlot ore){
	comstorage_.ChooseStoreOre(static_cast<CComStorage::Eore_station>(ore));
}

} // namespace my_engineer
