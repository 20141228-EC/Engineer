/******************************************************************************
 * @brief        
 * 
 * @file         com_yaw.cpp
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
 * @brief    初始化滚轴电机模块
 ******************************************************************************/
EAppStatus CModController::CComYaw::InitComponent(SModInitParam_Base &param) {
	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto controllerParam = static_cast<SModInitParam_Controller &>(param);

	// 保存电机指针
	motor[0] = MotorIDMap.at(controllerParam.yaw_id);

	// 设置发送节点
	mtrCanTxNode_[0] = controllerParam.yawTxNode;

	// 初始化PID控制器
	controllerParam.yawPosPidParam.threadNum = 1;
	pidPosCtrl.InitPID(&controllerParam.yawPosPidParam);

	controllerParam.yawSpdPidParam.threadNum = 1;
	pidSpdCtrl.InitPID(&controllerParam.yawSpdPidParam);

	// 初始化电机数据输出缓冲区
	mtrOutputBuffer.fill(0);

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/******************************************************************************
 * @brief    更新组件
 ******************************************************************************/
EAppStatus CModController::CComYaw::UpdateComponent() {
	// 检查组件状态
	if (componentStatus == APP_RESET) return APP_ERROR;

	// 更新组件信息
	yawInfo.posit = motor[0]->motorData[CDevMtr::DATA_POSIT];
	yawInfo.isPositArrived = (abs(yawCmd.setPosit - yawInfo.posit) < 8192 * 0.05);

	switch (Component_FSMFlag_) {    ///<这个轴不需要重补
		case FSM_RESET: {
			StopComponent();
			mtrOutputBuffer.fill(0);
			pidPosCtrl.ResetPidController();
			pidSpdCtrl.ResetPidController();
			componentStatus = APP_OK;
			return APP_OK;
		}

		case FSM_PREINIT: {
			motor[0]->motorData[CDevMtr::DATA_POSIT] = motor[0]->motorData[CDevMtr::DATA_ANGLE] - CONTROLLER_YAW_MOTOR_MACH;
			while (motor[0]->motorData[CDevMtr::DATA_POSIT] > 4096)
				motor[0]->motorData[CDevMtr::DATA_POSIT] -= 8192;
			while (motor[0]->motorData[CDevMtr::DATA_POSIT] < -4096)
				motor[0]->motorData[CDevMtr::DATA_POSIT] += 8192;  //yaw轴就近归位
			mtrOutputBuffer.fill(0);
			pidPosCtrl.ResetPidController();
			pidSpdCtrl.ResetPidController();
			Component_FSMFlag_ = FSM_INIT;
			return APP_OK;
		}

		case FSM_INIT: {
			if(abs(yawInfo.posit) < 100){
				yawCmd.setPosit = 0;
				pidPosCtrl.ResetPidController();
				pidSpdCtrl.ResetPidController();
				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;
				return APP_OK;
			}
			return _UpdateOutput(0.0f);
		}

		case FSM_CTRL: {
			yawCmd.setPosit = std::clamp(yawCmd.setPosit, -(rangeLimit / 2), rangeLimit / 2);
			if (yawCmd.isFree) {
				mtrOutputBuffer.fill(0);
				return APP_OK;
			}
			return _UpdateOutput(static_cast<float_t>(yawCmd.setPosit));
		}

		default: {
			StopComponent();
			mtrOutputBuffer.fill(0);
			pidPosCtrl.ResetPidController();
			pidSpdCtrl.ResetPidController();
			componentStatus = APP_ERROR;
			return APP_ERROR;
		}
	}
}

/******************************************************************************
 * @brief    物理位置转换为电机位置
 * 
 * @param    phyPosit 
 * @return   int32_t 
 ******************************************************************************/
int32_t CModController::CComYaw::PhyPositToMtrPosit(float_t phyPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = 22.76;

	return static_cast<int32_t>(phyPosit * scale + zeroOffset);
}

/******************************************************************************
 * @brief    电机位置转换为物理位置
 * 
 * @param    mtrPosit 
 * @return   float_t 
 ******************************************************************************/
float_t CModController::CComYaw::MtrPositToPhyPosit(int32_t mtrPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = 22.76;

	return static_cast<float_t>(mtrPosit - zeroOffset) / scale;
}

/******************************************************************************
 * @brief    输出更新函数
 ******************************************************************************/
EAppStatus CModController::CComYaw::_UpdateOutput(float_t posit) {
	// 位置环
	DataBuffer<float_t> yawPos = {
		static_cast<float_t>(posit),
	};

	DataBuffer<float_t> yawPosMeasure = {
		static_cast<float_t>(motor[0]->motorData[CDevMtr::DATA_POSIT]),
	};

	auto yawSpd =
		pidPosCtrl.UpdatePidController(yawPos, yawPosMeasure);

	// 速度环
	DataBuffer<float_t> yawSpdMeasure = {
		static_cast<float_t>(motor[0]->motorData[CDevMtr::DATA_SPEED]),
	};

	auto output =
		pidSpdCtrl.UpdatePidController(yawSpd, yawSpdMeasure);

	mtrOutputBuffer = {
		static_cast<int16_t>(output[0]),
	};

	return APP_OK;
}

}
