/**
 * @file com_pitch_servo_gimbal.cpp
 * @author Ciallo～(∠·ω< )⌒☆
 * @brief 云台俯仰组件 舵机版本
 * @version 1.0
 * @date 2025-01-19
 *
 * @note 由于机械进度问题，暂时使用舵机代替M2006电机
 *       后期切换回电机时，注释 USE_PITCH_SERVO 宏即可切换到 com_pitch_gimbal.cpp
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "mod_gimbal.hpp"

// 仅在舵机模式下编译此文件
#ifdef USE_PITCH_SERVO

#include "dev_servo.hpp"
#include "dev_common.hpp"  // DeviceIDMap

namespace my_engineer {

/**
 * @brief 初始化云台俯仰组件（舵机版本）
 *
 * @param param
 * @return EAppStatus
 */
EAppStatus CModGimbal::CComPitchServo::InitComponent(SModInitParam_Base &param){
	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto gimbalParam = static_cast<SModInitParam_Gimbal &>(param);

	// 检查舵机ID是否有效
	if (gimbalParam.pitchServoID == EDeviceID::DEV_NULL) return APP_ERROR;

	// 从通用设备映射表获取舵机指针
	if (DeviceIDMap.find(gimbalParam.pitchServoID) == DeviceIDMap.end()) {
		return APP_ERROR;
	}
	servo = static_cast<CDevServo*>(DeviceIDMap.at(gimbalParam.pitchServoID));

	// 保存角度参数
	servoAngleMin = gimbalParam.servoAngleMin;
	servoAngleMax = gimbalParam.servoAngleMax;
	servoAngleOffset = gimbalParam.servoAngleOffset;

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新俯仰组件（舵机版本）
 *
 * @note 舵机控制相比电机简单很多，无需PID，直接角度映射
 */
EAppStatus CModGimbal::CComPitchServo::UpdateComponent() {
	// 检查组件状态
	if (componentStatus == APP_RESET) return APP_ERROR;
	if (servo == nullptr) return APP_ERROR;

	// 舵机没有反馈，直接使用设定值
	pitchInfo.posit = pitchCmd.setPosit;
	pitchInfo.isPositArrived = true; // 舵机响应快，认为立即到位

	switch (Component_FSMFlag_){
		case FSM_RESET: {
			// 重置状态下，禁用舵机
			servo->servoCmd.enable = false;
			return APP_OK;
		}

		case FSM_PREINIT:
		case FSM_INIT: {
			// 初始化状态，使能舵机并设置到中间位置
			servo->servoCmd.enable = true;
			pitchCmd.setPosit = 0; // 默认0度
			Component_FSMFlag_ = FSM_CTRL;
			componentStatus = APP_OK;
			return APP_OK;
		}

		case FSM_CTRL: {
			// 正常控制状态
			return _UpdateServoOutput(pitchCmd.setPosit);
		}

		default: {
			StopComponent();
			servo->servoCmd.enable = false;
			componentStatus = APP_ERROR;
			return APP_ERROR;
		}
	}

	return APP_OK;
}

/**
 * @brief 舵机输出更新函数
 *
 * @param posit 目标物理角度（度）
 * @return EAppStatus
 */
EAppStatus CModGimbal::CComPitchServo::_UpdateServoOutput(float_t posit) {
	if (servo == nullptr) return APP_ERROR;

	// 物理角度范围限制 (0~90度)
	posit = std::clamp(posit, 0.0f, GIMBAL_PITCH_PHYSICAL_RANGE);

	// 假设物理角度0度对应舵机的offset角度
	float_t servoAngle = posit + servoAngleOffset;

	// 舵机角度范围限制 (0~180度)
	servoAngle = std::clamp(servoAngle, servoAngleMin, servoAngleMax);

	// 设置舵机
	servo->servoCmd.enable = true;
	servo->servoCmd.set_angle = servoAngle;

	return APP_OK;
}

/**
 * @brief 物理位置转换为设定位置
 *
 * @param phyPosit 物理角度（度）
 * @return float_t 设定角度（度）
 */
float_t CModGimbal::CComPitchServo::PhyPositToSetPosit(float_t phyPosit){
	return std::clamp(phyPosit, 0.0f, GIMBAL_PITCH_PHYSICAL_RANGE);
}

/**
 * @brief 设定位置转换为物理位置
 *
 * @param setPosit 设定角度（度）
 * @return float_t 物理角度（度）
 */
float_t CModGimbal::CComPitchServo::SetPositToPhyPosit(float_t setPosit){
	return std::clamp(setPosit, 0.0f, GIMBAL_PITCH_PHYSICAL_RANGE);
}

} // namespace my_engineer

#endif // USE_PITCH_SERVO
