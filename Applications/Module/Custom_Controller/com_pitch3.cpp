/******************************************************************************
 * @brief
 *
 * @file         com_pitch3.cpp
 * @author       Ciallo (1002046597@qq.com)
 * @version      V1.0
 * @date         2025-03-30
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-22
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#include "mod_controller.hpp"

extern "C" {
extern volatile int dbg_position_hold_mode;
extern volatile float dbg_hold_kp;
extern volatile float dbg_hold_kd;
extern volatile float dbg_target_pitch3;
}


namespace my_engineer {

/******************************************************************************
 * @brief    初始化自定义控制器Pitch3电机模块
 ******************************************************************************/
EAppStatus CModController::CComPitch3::InitComponent(SModInitParam_Base &param){
	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto controllerParam = static_cast<SModInitParam_Controller &>(param);

	// 保存电机指针
	motor[0] = static_cast<CDevMtrDM*>(MotorIDMap.at(controllerParam.pitch3_id));

	// 设置发送节点
	mtrCanTxNode_[0] = controllerParam.pitch3TxNode;

	// 初始化控制参数
	pitch3Cmd.setParam[EMotorParam::KP] = motor[0]->Kp;   ///< set Kp
	pitch3Cmd.setParam[EMotorParam::KD] = motor[0]->Kd;  ///< set Kd
	pitch3Cmd.setParam[EMotorParam::SPEED] = 0.0f;      ///< Position control means the speed is zero.

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/******************************************************************************
 * @brief    更新组件
 ******************************************************************************/
EAppStatus CModController::CComPitch3::UpdateComponent() {
	// 检查组件状态
	if (componentStatus == APP_RESET) return APP_ERROR;

	// 更新电机信息
	pitch3Info.posit = MotortruePositToOffsetPosit_test(     ///<注意是在这里更新的示教器控制信息传给机器人，下面的状态机是用来控制示教器的重力补偿的
			CDevMtrDM::uint_to_float(motor[0]->motorData[CDevMtr::DATA_ANGLE], -motor[0]->mitLimit_.Q_MAX, motor[0]->mitLimit_.Q_MAX, 16));
	pitch3Info.isPositArrived = (fabs(pitch3Cmd.setParam[EMotorParam::POSIT] - pitch3Info.posit) < 8.0f);

	switch (Component_FSMFlag_){
		case FSM_RESET: {
			std::fill(std::begin(pitch3Cmd.setParam), std::end(pitch3Cmd.setParam), 0.0f);
			return APP_OK;
		}

		case FSM_PREINIT: {
			float_t params[] = {0.0, 0.0f, motor[0]->Kp, motor[0]->Kd, 0.0f};
			std::copy(std::begin(params), std::end(params), pitch3Cmd.setParam);
			Component_FSMFlag_ = FSM_INIT;
			return APP_OK;
		}

		case FSM_INIT: {
			if (pitch3Info.isPositArrived) {
				pitch3Cmd.setParam[static_cast<int>(EMotorParam::POSIT)] = 0.0f;
				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;
				return APP_OK;
			}
			float_t params[] = {0.0, 0.0f, motor[0]->Kp, motor[0]->Kd, 0.0f};
			std::copy(std::begin(params), std::end(params), pitch3Cmd.setParam);
			return _UpdateOutput(pitch3Cmd.setParam);
		}

		case FSM_CTRL: {
			if (pitch3Cmd.isFree) {
				float_t savedTF = pitch3Cmd.setParam[EMotorParam::TF];
				std::fill(std::begin(pitch3Cmd.setParam), std::end(pitch3Cmd.setParam), 0.0f);
				if (dbg_position_hold_mode) {
					pitch3Cmd.setParam[EMotorParam::KP] = dbg_hold_kp;
					pitch3Cmd.setParam[EMotorParam::KD] = dbg_hold_kd;
					pitch3Cmd.setParam[EMotorParam::POSIT] = dbg_target_pitch3;
				} else {
					pitch3Cmd.setParam[EMotorParam::KP] = 0.0f;
					pitch3Cmd.setParam[EMotorParam::KD] = 0.03f;
				}
				pitch3Cmd.setParam[EMotorParam::TF] = savedTF;
				if(!dbg_position_hold_mode && pitch3Info.posit < 0){
					pitch3Cmd.setParam[EMotorParam::POSIT] = pitch3Info.posit;//这个是装配的问题就是实际上p3的角度只有向下的
				}
				return _UpdateOutput(pitch3Cmd.setParam);
			}
			// 联动模式：恢复位控参数，跟随机器人回传位置
			pitch3Cmd.setParam[EMotorParam::KP] = motor[0]->Kp;
			pitch3Cmd.setParam[EMotorParam::KD] = motor[0]->Kd;
			return _UpdateOutput(pitch3Cmd.setParam);
		}

		default: {
			StopComponent();
			std::fill(std::begin(pitch3Cmd.setParam), std::end(pitch3Cmd.setParam), 0.0f);
			componentStatus = APP_ERROR;
			return APP_ERROR;
		}
	}
}

/******************************************************************************
 * @brief    物理位置转换为电机位置
 *
 * @param    offsetPosit
 * @return   float_t
 ******************************************************************************/
float_t CModController::CComPitch3::OffsetPositToMotortruePosit_test(float_t offsetPosit) {
	const float_t scale = 180.0f / PI;
	return CONTROLLER_PITCH3_MOTOR_DIR * (static_cast<float_t>(offsetPosit) / scale);
}

/******************************************************************************
 * @brief    电机位置转换为物理位置
 *
 * @param    motortruePosit
 * @return   float_t
 ******************************************************************************/
float_t CModController::CComPitch3::MotortruePositToOffsetPosit_test(float_t motortruePosit) {
	const float_t scale = 180.0f / PI;
	return CONTROLLER_PITCH3_MOTOR_DIR *(static_cast<float_t>(motortruePosit * scale));
}

/******************************************************************************
 * @brief    输出更新函数
 ******************************************************************************/
EAppStatus CModController::CComPitch3::_UpdateOutput(float_t* Setparam){
  float_t posit = OffsetPositToMotortruePosit_test(Setparam[static_cast<int>(EMotorParam::POSIT)]);
  float_t torq = Setparam[static_cast<int>(EMotorParam::TF)];

  /* 使用电机内部 TxNode 发送 (CAN重分配后Pitch1独占CAN3，无需降频) */
  motor[0]->Control_MIT(
      Setparam[static_cast<int>(EMotorParam::KP)],
      Setparam[static_cast<int>(EMotorParam::KD)],
      posit,
      Setparam[static_cast<int>(EMotorParam::SPEED)],
      torq);

  return APP_OK;
}

} // namespace my_engineer
