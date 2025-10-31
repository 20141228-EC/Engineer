/******************************************************************************
 * @brief        
 * 
 * @file         com_roll.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-07-09
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "mod_arm.hpp"

namespace my_engineer {

/**
 * @brief 初始化机械臂关节Roll组件
 * 
 * @param param 
 * @return EAppStatus 
 */
EAppStatus CModArm::CComRoll::InitComponent(SModInitParam_Base &param) {
	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto armParam = static_cast<SModInitParam_Arm &>(param);

	// 保存电机指针
	motor = MotorIDMap.at(armParam.MotorID_Roll);

	// 初始化PID控制器
	mitCtrl.kp = armParam.MIT_Roll_kp;
	mitCtrl.kd = armParam.MIT_Roll_kd;

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新Roll关节组件
 * 
 * @return EAppStatus 
 */
EAppStatus CModArm::CComRoll::UpdateComponent() {
	// 检查组件状态
	if (componentStatus == APP_RESET) return APP_ERROR;

	CDevMtrDM_MIT *pMtr = static_cast<CDevMtrDM_MIT *>(motor);

	// 更新组件信息
	rollInfo.angle = rad2deg(pMtr->motorPhyAngle);
	rollInfo.isAngleArrived = (fabs(rollInfo.angle - rollCmd.setAngle) < 3.0f);

	uint8_t test1 = 0;
	if(test1 == 1) {
		pMtr->SetZero();
	}

	// 缓慢移动控制逻辑
	static float_t next_angle = 0.0f;
	static float_t gradual_kp = 0.005f;
	static float_t gradual_min = 0.03f;

	next_angle += (rollCmd.setAngle - next_angle) * gradual_kp;
	if (fabs(next_angle - rollCmd.setAngle) < gradual_min) {
		next_angle = rollCmd.setAngle;
	}

	switch (Component_FSMFlag_) {
		case FSM_RESET: {
			pMtr->Control_MIT(0.0f, 0.0f, deg2rad(0.0f) * ARM_ROLL_MOTOR_DIR, 0.0f, 0.0f);
			rollCmd = SRollCmd();
			return APP_OK;
		}

		case FSM_PREINIT: {
			rollCmd.setAngle = ARM_ROLL_INIT_ANGLE;
			Component_FSMFlag_ = FSM_INIT;
			return APP_OK;
		}

		case FSM_INIT: {
			if (fabs(rollInfo.angle - rollCmd.setAngle) < 10.0) {
				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;
			}
			pMtr->Control_MIT(mitCtrl.kp, mitCtrl.kd, deg2rad(rollCmd.setAngle), 0.0f, 0.0f);
			return APP_OK;
		}

		case FSM_CTRL: {
			pMtr->Control_MIT(mitCtrl.kp, mitCtrl.kd, deg2rad(next_angle), 0.0f, 0.0f);
			return APP_OK;
		}

		default: {
			componentStatus = APP_ERROR;
			return APP_ERROR;
		}
	}

	return APP_OK;
}

} // namespace my_engineer

