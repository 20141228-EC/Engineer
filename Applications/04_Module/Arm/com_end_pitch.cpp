/******************************************************************************
 * @brief        末端Pitch关节组件
 *
 * @file         com_end_pitch.cpp
 * @author       ciallo (1002046597@qq.com)
 * @version      V1.0
 * @date         2026-05-27
 *
 * @copyright    Copyright (c) 2026
 *
 ******************************************************************************/

#include "mod_arm.hpp"

namespace my_engineer {

/**
 * @brief 初始化末端Pitch关节组件
 *
 * @param param
 * @return EAppStatus
 */
EAppStatus CModArm::CComEndPitch::InitComponent(SModInitParam_Base &param) {
	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto armParam = static_cast<SModInitParam_Arm &>(param);

	// 保存电机指针
	motor = MotorIDMap.at(armParam.MotorID_End_Pitch);				///<通过ID映射找到初始注册的电机

	// 初始化MIT控制器参数
	mitCtrl.kp = armParam.MIT_End_Pitch_kp;
	mitCtrl.kd = armParam.MIT_End_Pitch_kd;

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新末端Pitch关节组件
 *
 * @return EAppStatus
 */
EAppStatus CModArm::CComEndPitch::UpdateComponent() {
	// 检查组件状态
	if (componentStatus == APP_RESET) {
		static_cast<CDevMtrDM_MIT *>(motor)->Control_MIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		return APP_ERROR;
	}

	CDevMtrDM_MIT *pMtr = static_cast<CDevMtrDM_MIT *>(motor);

	// 更新组件信息
	endPitchInfo.angle = rad2deg(pMtr->motorPhyAngle);
	endPitchInfo.torque = CDevMtrDM_MIT::uint_to_float(pMtr->motorData[CDevMtr::DATA_TORQUE], -pMtr->get_tau_max(), pMtr->get_tau_max(), 12); ///< 12位无符号转实际力矩
	endPitchInfo.isAngleArrived = (fabs(endPitchInfo.angle - endPitchCmd.setAngle) < 3.0f);

	// 缓慢移动控制逻辑
	static float_t next_angle = 0.0f;
	static float_t gradual_kp = 0.05f;
	static float_t gradual_min = 0.03f;

	next_angle += (endPitchCmd.setAngle - next_angle) * gradual_kp;		///<一阶低通滤波，避免角度突变
	if (fabs(next_angle - endPitchCmd.setAngle) < gradual_min) {
		next_angle = endPitchCmd.setAngle;									///<设定最小的分辨率
	}
	
	static uint8_t test1 = 0;
	if(test1 == 1) {
		pMtr->SetZero();			///<测试用，将当前角度设为零点
		test1 =0;
	}
	switch (Component_FSMFlag_) {
		case FSM_RESET: {
			pMtr->Control_MIT(0.0f, 0.0f, deg2rad(0.0f) * ARM_END_PITCH_MOTOR_DIR, 0.0f, 0.0f);
			endPitchCmd = SEndPitchCmd();									///<调用默认构造函数初始化
			return APP_OK;
		}

		case FSM_PREINIT: {
			endPitchCmd.setAngle = ARM_END_PITCH_INIT_ANGLE;
			Component_FSMFlag_ = FSM_INIT;
			return APP_OK;
		}

		case FSM_INIT: {
			if (fabs(endPitchInfo.angle - endPitchCmd.setAngle) < 10.0) {
				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;
			}
			pMtr->Control_MIT(mitCtrl.kp, mitCtrl.kd, deg2rad(endPitchCmd.setAngle), 0.0f, 0.0f);
			return APP_OK;
		}

		case FSM_CTRL: {
			float_t kp = onlyGravity_ ? 0.0f : mitCtrl.kp;
			float_t kd = onlyGravity_ ? 0.0f : mitCtrl.kd;
			pMtr->Control_MIT(kp, kd, deg2rad(next_angle), 0.0f, this->Grav_End_Pitch_Out);
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
