/******************************************************************************
 * @brief        
 * 
 * @file         com_pitch1.cpp
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
 * @brief    初始化自定义控制器Pitch1电机模块
 ******************************************************************************/
EAppStatus CModController::CComPitch1::InitComponent(SModInitParam_Base &param){
	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto controllerParam = static_cast<SModInitParam_Controller &>(param);

	// 保存电机指针
	motor[0] = static_cast<CDevMtrDM*>(MotorIDMap.at(controllerParam.pitch1_id));

	// 设置发送节点
	mtrCanTxNode_[0] = controllerParam.pitch1TxNode;

	// 初始化控制参数
	pitch1Cmd.setParam[EMotorParam::KP] = motor[0]->Kp;   ///< set Kp
	pitch1Cmd.setParam[EMotorParam::KD] = motor[0]->Kd;  ///< set Kd
	pitch1Cmd.setParam[EMotorParam::SPEED] = 0.0f;      ///< Position control means the speed is zero.

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/******************************************************************************
 * @brief    更新组件
 ******************************************************************************/
EAppStatus CModController::CComPitch1::UpdateComponent() {
	// 检查组件状态
	if (componentStatus == APP_RESET) return APP_ERROR;

	// 更新电机信息
	pitch1Info.posit = MotortruePositToOffsetPosit_test(     ///<注意是在这里更新的示教器控制信息传给机器人，下面的状态机是用来控制自定义控制器的重力补偿的
			CDevMtrDM::uint_to_float(motor[0]->motorData[CDevMtr::DATA_ANGLE], Pos_MIN, Pos_MAX, 16));
	pitch1Info.isPositArrived = (fabs(pitch1Cmd.setParam[EMotorParam::POSIT] - pitch1Info.posit) < 5.0f);

	switch (Component_FSMFlag_){
		case FSM_RESET: {
			std::fill(std::begin(pitch1Cmd.setParam), std::end(pitch1Cmd.setParam), 0.0f);
			return APP_OK;
		}

		case FSM_PREINIT: {
			float_t params[] = {20.0, 0.0f, motor[0]->Kp, motor[0]->Kd, 0.0f};
			std::copy(std::begin(params), std::end(params), pitch1Cmd.setParam);
			Component_FSMFlag_ = FSM_INIT;
			return APP_OK;
		}

		case FSM_INIT: {
			if (pitch1Info.isPositArrived) {
				pitch1Cmd.setParam[static_cast<int>(EMotorParam::POSIT)] = 20.0f;
				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;
				return APP_OK;
			}
			float_t params[] = {20.0, 0.0f, motor[0]->Kp, motor[0]->Kd, 0.0f};
			std::copy(std::begin(params), std::end(params), pitch1Cmd.setParam);
			return _UpdateOutput(pitch1Cmd.setParam);
		}

		case FSM_CTRL: {
			if (pitch1Cmd.isFree) {
				std::fill(std::begin(pitch1Cmd.setParam), std::end(pitch1Cmd.setParam), 0.0f);
				return _UpdateOutput(pitch1Cmd.setParam);
			}
			return _UpdateOutput(pitch1Cmd.setParam);
		}

		default: {
			StopComponent();
			std::fill(std::begin(pitch1Cmd.setParam), std::end(pitch1Cmd.setParam), 0.0f);
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
float_t CModController::CComPitch1::OffsetPositToMotortruePosit_test(float_t offsetPosit) {
	const float_t zeroOffset = 5.0f; 
	const float_t scale = 180.0f / PI; 
	return (static_cast<float_t>(offsetPosit - zeroOffset) / scale);
}

/******************************************************************************
 * @brief    电机位置转换为物理位置
 * 
 * @param    motortruePosit 
 * @return   float_t 
 ******************************************************************************/
float_t CModController::CComPitch1::MotortruePositToOffsetPosit_test(float_t motortruePosit) {
	const float_t zeroOffset = 5.0f;
	const float_t scale = 180.0f / PI;
	
	return (static_cast<float_t>(motortruePosit * scale) + zeroOffset);
}

/******************************************************************************
 * @brief    输出更新函数
 ******************************************************************************/
EAppStatus CModController::CComPitch1::_UpdateOutput(float_t* Setparam){
  float_t posit = OffsetPositToMotortruePosit_test(Setparam[static_cast<int>(EMotorParam::POSIT)]);    ///< turn offset posit to motor true posit
  float_t torq = Setparam[static_cast<int>(EMotorParam::TF)];       

  CDevMtrDM::FillCanTxBuffer_MIT(motor[0], mtrCanTxNode_[0]->dataBuffer, 
                              posit, Setparam[static_cast<int>(EMotorParam::SPEED)], 
                              torq, Setparam[static_cast<int>(EMotorParam::KP)], 
                              Setparam[static_cast<int>(EMotorParam::KD)]);   
                              
  /*!!!!!!!!!!!Here Can had been sent!!!!!!!!!!!!!!!!!!!*/
  mtrCanTxNode_[0]->Transmit();
  return APP_OK;
}

} // namespace my_engineer
