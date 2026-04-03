/******************************************************************************
 * @brief        
 * 
 * @file         com_rocker.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-03-30
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "mod_controller.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    初始化自定义控制器摇杆模块
 ******************************************************************************/
EAppStatus CModController::CComRocker::InitComponent(SModInitParam_Base &param) {

	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto controllerParam = static_cast<SModInitParam_Controller &>(param);

	// 保存摇杆指针
	rocker = static_cast<CDevRocker *>(DeviceIDMap.at(controllerParam.rocker_id));

	// 保存X轴校准参数（每个摇杆实例独立）
	x_center    = controllerParam.rocker_x_center;
	x_range_pos = controllerParam.rocker_x_range_pos;
	x_range_neg = controllerParam.rocker_x_range_neg;
	x_dir       = controllerParam.rocker_x_dir;

	// 保存Y轴校准参数（每个摇杆实例独立）
	y_center    = controllerParam.rocker_y_center;
	y_range_pos = controllerParam.rocker_y_range_pos;
	y_range_neg = controllerParam.rocker_y_range_neg;
	y_dir       = controllerParam.rocker_y_dir;

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/******************************************************************************
 * @brief    更新组件
 ******************************************************************************/
EAppStatus CModController::CComRocker::UpdateComponent() {
	// 检查组件状态
	if (componentStatus == APP_RESET) return APP_ERROR;

	componentStatus = APP_OK;

	// 更新X轴信息（死区重映射：出死区后从0开始平滑过渡，消除跳变）
	{
		int32_t centered_x = rocker->rockerValues.X - x_center;
		if (abs(centered_x) < CONTROLLER_ROCKER_DEAD_ZONE) {
			rockerInfo.X = 0;
		} else {
			// 减去死区偏移，使出死区后从0开始
			rockerInfo.X = (centered_x > 0)
				? (centered_x - CONTROLLER_ROCKER_DEAD_ZONE)
				: (centered_x + CONTROLLER_ROCKER_DEAD_ZONE);
		}
	}

	// 更新Y轴信息（同样应用死区重映射）
	if (!rocker->IsYEnabled()) {
		rockerInfo.Y = 0;
	} else {
		int32_t centered_y = rocker->rockerValues.Y - y_center;
		if (abs(centered_y) < CONTROLLER_ROCKER_DEAD_ZONE) {
			rockerInfo.Y = 0;
		} else {
			rockerInfo.Y = (centered_y > 0)
				? (centered_y - CONTROLLER_ROCKER_DEAD_ZONE)
				: (centered_y + CONTROLLER_ROCKER_DEAD_ZONE);
		}
	}

	if (rocker->rockerValues.key == 1) key_duration += HAL_GetTick() - last_time_stamp;//这一部分是摇杆有按键功能的处理，但是现在没有用到
	else key_duration = 0;

	last_time_stamp = HAL_GetTick();

	if (key_duration == 0) rockerInfo.Key_status = KEY_STATUS::RELEASE;
	else if (key_duration < CONTROLLER_ROCKER_KEY_LONG_PRESS_DURATION) rockerInfo.Key_status = KEY_STATUS::PRESS;
	else rockerInfo.Key_status = KEY_STATUS::LONG_PRESS;

	return APP_OK;
}

} // namespace my_engineer
