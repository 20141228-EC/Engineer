/******************************************************************************
 * @brief        
 * 
 * @file         com_yaw.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-07-10
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include <cstddef>  // for offsetof
#include "mod_arm.hpp"

namespace my_engineer {

/**
 * @brief 初始化机械臂Yaw组件
 * 
 * @param param 
 * @return EAppStatus 
 */
EAppStatus CModArm::CComJoint::InitComponent(SModInitParam_Base &param) {
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	auto armParam = static_cast<SModInitParam_Arm &>(param);

	motor[Y] = MotorIDMap.at(armParam.MotorID_Yaw);
	motor[P1] = MotorIDMap.at(armParam.MotorID_Pitch1);
	motor[P2] = MotorIDMap.at(armParam.MotorID_Pitch2);

	mtrCanTxNode[Y] = armParam.MotorTxNode_Yaw;
	mtrCanTxNode[P1] = armParam.MotorTxNode_Pitch1;
	mtrCanTxNode[P2] = armParam.MotorTxNode_Pitch2;

	armParam.YawPosPidParam.threadNum = 1;
	pidPosCtrl_yaw.InitPID(&armParam.YawPosPidParam);
	armParam.YawSpdPidParam.threadNum = 1;
	pidSpdCtrl_yaw.InitPID(&armParam.YawSpdPidParam);

	armParam.Pitch1PosPidParam.threadNum = 1;
	pidPosCtrl_pitch1.InitPID(&armParam.Pitch1PosPidParam);
	armParam.Pitch1SpdPidParam.threadNum = 1;
	pidSpdCtrl_pitch1.InitPID(&armParam.Pitch1SpdPidParam);

	armParam.Pitch2PosPidParam.threadNum = 1;
	pidPosCtrl_pitch2.InitPID(&armParam.Pitch2PosPidParam);
	armParam.Pitch2SpdPidParam.threadNum = 1;
	pidSpdCtrl_pitch2.InitPID(&armParam.Pitch2SpdPidParam);

	mtrOutputBuffer.fill(0);

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新Yaw组件
 * 
 * @return EAppStatus 
 */
EAppStatus CModArm::CComJoint::UpdateComponent() {
	static bool isreset_flag = false;
	static bool alreadySetYaw = false;
	if (componentStatus == APP_RESET) {
		return APP_ERROR;
	}

	jointInfo.posit_yaw = motor[Y]->motorData[CDevMtr::DATA_POSIT] * ARM_YAW_MOTOR_DIR;
	jointInfo.posit_pitch1 = motor[P1]->motorData[CDevMtr::DATA_POSIT] * ARM_PITCH1_MOTOR_DIR;
	jointInfo.posit_pitch2 = motor[P2]->motorData[CDevMtr::DATA_POSIT] * ARM_PITCH2_MOTOR_DIR;
	
	jointInfo.isPositArrived_yaw = abs(jointInfo.posit_yaw - jointCmd.setPosit_yaw) < 700;
	jointInfo.isPositArrived_pitch1 = abs(jointInfo.posit_pitch1 - jointCmd.setPosit_pitch1) < 700;
	jointInfo.isPositArrived_pitch2 = abs(jointInfo.posit_pitch2 - jointCmd.setPosit_pitch2) < 700;

	switch (Component_FSMFlag_) {

		case FSM_RESET: {
			mtrOutputBuffer.fill(0);
			pidPosCtrl_yaw.ResetPidController();
			pidSpdCtrl_yaw.ResetPidController();
			pidPosCtrl_pitch1.ResetPidController();
			pidSpdCtrl_pitch1.ResetPidController();
			pidPosCtrl_pitch2.ResetPidController();
			pidSpdCtrl_pitch2.ResetPidController();
			isreset_flag = false;
			return APP_OK;
		}

		case FSM_PREINIT: {
			if (isreset_flag == false){
				mtrOutputBuffer.fill(0);
				pidPosCtrl_yaw.ResetPidController();
				pidSpdCtrl_yaw.ResetPidController();
				pidPosCtrl_pitch1.ResetPidController();
				pidSpdCtrl_pitch1.ResetPidController();
				pidPosCtrl_pitch2.ResetPidController();
				pidSpdCtrl_pitch2.ResetPidController();
				/*设置每个关节的绝对角度*/
				motor[Y]->motorData[CDevMtr::DATA_POSIT]  = motor[Y]->motorData[CDevMtr::DATA_ANGLE] * ARM_YAW_MOTOR_DIR;
				motor[P1]->motorData[CDevMtr::DATA_POSIT] = motor[P1]->motorData[CDevMtr::DATA_ANGLE] - POSIT_JOINT2_PITCH1_MACH;
				while(motor[P1]->motorData[CDevMtr::DATA_POSIT] < -32767)
					motor[P1]->motorData[CDevMtr::DATA_POSIT] += 65535;
				motor[P1]->motorData[CDevMtr::DATA_POSIT] +=	POSIT_JOINT2_PITCH1_MACH_PHY * 182.04f * ARM_PITCH1_MOTOR_DIR;			///<这个是等效连杆和水平面的夹角
				jointCmd.setPosit_pitch1 = static_cast<int32_t>(70.0f * 182.04f);																		///<这个初始角度

				motor[P2]->motorData[CDevMtr::DATA_POSIT] = motor[P2]->motorData[CDevMtr::DATA_ANGLE] - POSIT_JOINT3_PITCH2_MACH;
				while(motor[P2]->motorData[CDevMtr::DATA_POSIT] < -32767)
					motor[P2]->motorData[CDevMtr::DATA_POSIT] += 65535;
				motor[P2]->motorData[CDevMtr::DATA_POSIT]  += ARM_PITCH2_MOTOR_DIR * POSIT_JOINT3_PITCH2_MACH_PHY * 182.04f;			///<这个是等效连杆和水平面的夹角
				jointCmd.setPosit_pitch2 = static_cast<int32_t>(70.0f * 182.04f);																		///<这个复位角度是90度
				alreadySetYaw = false;
				isreset_flag = true;  // 重置标志
			}	
			else{
				/*全部到位后才进入初始化 - 优先级最高*/
				if(jointInfo.isPositArrived_pitch2 && jointInfo.isPositArrived_pitch1 && alreadySetYaw == true){
					if(abs(jointInfo.posit_yaw - jointCmd.setPosit_yaw)<300){
						motor[Y]->motorData[CDevMtr::DATA_POSIT] = 0.0f;
						jointCmd.setPosit_yaw = 0;
						jointCmd.setPosit_pitch1 = POSIT_JOINT2_PITCH1_INIT_PHY * 182.04f;
						jointCmd.setPosit_pitch2 = POSIT_JOINT3_PITCH2_INIT_PHY * 182.04f;
						Component_FSMFlag_ = FSM_INIT;
						return APP_OK;
					}
					else if(motor[Y]->motorStatus == CDevMtr::EMotorStatus::STALL){
						motor[Y]->motorData[CDevMtr::DATA_POSIT] = 36484 * ARM_YAW_MOTOR_DIR; ///< 36484是Yaw电机的初始位置
						jointCmd.setPosit_yaw = 0;
					}
					return _UpdateOutput(static_cast<float_t>(jointCmd.setPosit_yaw),
						static_cast<float_t>(jointCmd.setPosit_pitch1),
						static_cast<float_t>(jointCmd.setPosit_pitch2));
				}
				/*全部到位后才进入初始化*/
				else if(jointInfo.isPositArrived_pitch2 && jointInfo.isPositArrived_pitch1 && alreadySetYaw == false){
					jointCmd.setPosit_yaw = POSIT_JOINT1_YAW_MACH ;
					alreadySetYaw = true;
					return _UpdateOutput(static_cast<float_t>(jointCmd.setPosit_yaw),
						static_cast<float_t>(jointCmd.setPosit_pitch1),
						static_cast<float_t>(jointCmd.setPosit_pitch2));
				}
				/*先抬起两个臂yaw才能动 - 至少有一个pitch没到位*/
				else {
					_UpdateOutput_Pitch2(jointCmd.setPosit_pitch2);
					_UpdateOutput_Pitch1(jointCmd.setPosit_pitch1);
					// Yaw保持不动
					mtrOutputBuffer[Y] = 0;
					return APP_OK;
				}
			}

			return APP_OK;
		}

		case FSM_INIT: {
			if (jointInfo.isPositArrived_yaw && jointInfo.isPositArrived_pitch1 && jointInfo.isPositArrived_pitch2) {
				jointCmd.setPosit_yaw = 0;
				jointCmd.setPosit_pitch1 = POSIT_JOINT2_PITCH1_INIT_PHY * 182.04f;
				jointCmd.setPosit_pitch2 = POSIT_JOINT3_PITCH2_INIT_PHY * 182.04f;
				pidPosCtrl_yaw.ResetPidController();
				pidSpdCtrl_yaw.ResetPidController();
				pidPosCtrl_pitch1.ResetPidController();
				pidSpdCtrl_pitch1.ResetPidController();
				pidPosCtrl_pitch2.ResetPidController();
				pidSpdCtrl_pitch2.ResetPidController();
				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;
				return APP_OK;
			}
			return _UpdateOutput(static_cast<float_t>(0), static_cast<float_t>(POSIT_JOINT2_PITCH1_INIT_PHY * 182.04f), static_cast<float_t>(POSIT_JOINT3_PITCH2_INIT_PHY * 182.04f));
		}

		case FSM_CTRL: {
			jointCmd.setPosit_yaw = std::clamp(jointCmd.setPosit_yaw, static_cast<int32_t>(-rangeLimit_yaw/2),  static_cast<int32_t>(rangeLimit_yaw/2));

			return _UpdateOutput(static_cast<float_t>(jointCmd.setPosit_yaw),
				static_cast<float_t>(jointCmd.setPosit_pitch1),
				static_cast<float_t>(jointCmd.setPosit_pitch2));
			}

		default: {
			StopComponent();
			mtrOutputBuffer.fill(0);
			pidPosCtrl_yaw.ResetPidController();
			pidSpdCtrl_yaw.ResetPidController();
			pidPosCtrl_pitch1.ResetPidController();
			pidSpdCtrl_pitch1.ResetPidController();
			pidPosCtrl_pitch2.ResetPidController();
			pidSpdCtrl_pitch2.ResetPidController();
			componentStatus = APP_ERROR;
			return APP_ERROR;
		}
	}

	return APP_OK;
}



/*------------------------------------------------------------------------------------*/
// 物理位置转换为电机位置
int32_t CModArm::CComJoint::PhyPositToMtrPosit_yaw(float_t phyPosit) {
	const int32_t zeroOffset = 0.0f;//ARM_YAW_MOTOR_OFFSET;
	const float_t scale = -2*65535/360.0f;//ARM_YAW_MOTOR_RATIO;

	return static_cast<int32_t>((phyPosit * scale) + zeroOffset);
}

// 电机位置转换为物理位置
float_t CModArm::CComJoint::MtrPositToPhyPosit_yaw(int32_t mtrPosit) {
	const int32_t zeroOffset = 0.0f;//ARM_YAW_MOTOR_OFFSET;
	const float_t scale = -2*65535/360.0f;//ARM_YAW_MOTOR_RATIO;

	return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}
/*------------------------------------------------------------------------------------*/
// 物理位置转换为电机位置
int32_t CModArm::CComJoint::PhyPositToMtrPosit_pitch1(float_t phyPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = 182.04f;

	return static_cast<int32_t>((phyPosit * scale) + zeroOffset);
}

// 电机位置转换为物理位置
float_t CModArm::CComJoint::MtrPositToPhyPosit_pitch1(int32_t mtrPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = 182.04f;

	return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}
/*------------------------------------------------------------------------------------*/
int32_t CModArm::CComJoint::PhyPositToMtrPosit_pitch2(float_t phyPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = 182.04f;

	return static_cast<int32_t>((phyPosit * scale) + zeroOffset);
}

// 电机位置转换为物理位置
float_t CModArm::CComJoint::MtrPositToPhyPosit_pitch2(int32_t mtrPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = 182.04f;

	return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}
/*------------------------------------------------------------------------------------*/
// 输出更新函数
EAppStatus CModArm::CComJoint::_UpdateOutput(float_t posit_yaw, float_t posit_pitch1, float_t posit_pitch2) {

	DataBuffer<float_t> Pos_yaw = { static_cast<float_t>(posit_yaw * ARM_YAW_MOTOR_DIR) };
	DataBuffer<float_t> Pos_pitch1 = { static_cast<float_t>(posit_pitch1 * ARM_PITCH1_MOTOR_DIR) };
	DataBuffer<float_t> Pos_pitch2 = { static_cast<float_t>(posit_pitch2 * ARM_PITCH2_MOTOR_DIR) };

	DataBuffer<float_t> PosMeasure_yaw = {static_cast<float_t>(motor[Y]->motorData[CDevMtr::DATA_POSIT])};
	DataBuffer<float_t> PosMeasure_pitch1 = {static_cast<float_t>(motor[P1]->motorData[CDevMtr::DATA_POSIT])};
	DataBuffer<float_t> PosMeasure_pitch2 = {static_cast<float_t>(motor[P2]->motorData[CDevMtr::DATA_POSIT])};

	auto Spd_yaw = pidPosCtrl_yaw.UpdatePidController(Pos_yaw, PosMeasure_yaw);
	auto Spd_pitch1 = pidPosCtrl_pitch1.UpdatePidController(Pos_pitch1, PosMeasure_pitch1);
	auto Spd_pitch2 = pidPosCtrl_pitch2.UpdatePidController(Pos_pitch2, PosMeasure_pitch2);

	DataBuffer<float_t> SpdMeasure_yaw = {static_cast<float_t>(motor[Y]->motorData[CDevMtr::DATA_SPEED])};
	DataBuffer<float_t> SpdMeasure_pitch1 = {static_cast<float_t>(motor[P1]->motorData[CDevMtr::DATA_SPEED])};
	DataBuffer<float_t> SpdMeasure_pitch2 = {static_cast<float_t>(motor[P2]->motorData[CDevMtr::DATA_SPEED])};

	auto output_yaw = pidSpdCtrl_yaw.UpdatePidController(Spd_yaw, SpdMeasure_yaw);
	auto output_pitch1 = pidSpdCtrl_pitch1.UpdatePidController(Spd_pitch1, SpdMeasure_pitch1);
	auto output_pitch2 = pidSpdCtrl_pitch2.UpdatePidController(Spd_pitch2, SpdMeasure_pitch2);

	mtrOutputBuffer = { 
		static_cast<int16_t>(output_yaw[0]),
		static_cast<int16_t>(output_pitch1[0]),
		static_cast<int16_t>(output_pitch2[0]) };

	return APP_OK;
}

/**
 * @brief Yaw电机输出更新函数
 * 
 * @param posit_yaw Yaw目标位置
 * @return int16_t 电机输出值
 */
EAppStatus CModArm::CComJoint::_UpdateOutput_Yaw(float_t posit_yaw) {
	DataBuffer<float_t> Pos_yaw = { static_cast<float_t>(posit_yaw * ARM_YAW_MOTOR_DIR) };
	DataBuffer<float_t> PosMeasure_yaw = {static_cast<float_t>(motor[Y]->motorData[CDevMtr::DATA_POSIT])};

	auto Spd_yaw = pidPosCtrl_yaw.UpdatePidController(Pos_yaw, PosMeasure_yaw);

	DataBuffer<float_t> SpdMeasure_yaw = {static_cast<float_t>(motor[Y]->motorData[CDevMtr::DATA_SPEED])};

	auto output_yaw = pidSpdCtrl_yaw.UpdatePidController(Spd_yaw, SpdMeasure_yaw);

	mtrOutputBuffer[CComJoint::Y] = static_cast<int16_t>(output_yaw[0]);

	return APP_OK;
}

/**
 * @brief Pitch1电机输出更新函数
 * 
 * @param posit_pitch1 Pitch1目标位置
 * @return int16_t 电机输出值
 */
EAppStatus CModArm::CComJoint::_UpdateOutput_Pitch1(float_t posit_pitch1) {
	DataBuffer<float_t> Pos_pitch1 = { static_cast<float_t>(posit_pitch1 * ARM_PITCH1_MOTOR_DIR) };
	DataBuffer<float_t> PosMeasure_pitch1 = {static_cast<float_t>(motor[P1]->motorData[CDevMtr::DATA_POSIT])};

	auto Spd_pitch1 = pidPosCtrl_pitch1.UpdatePidController(Pos_pitch1, PosMeasure_pitch1);

	DataBuffer<float_t> SpdMeasure_pitch1 = {static_cast<float_t>(motor[P1]->motorData[CDevMtr::DATA_SPEED])};

	auto output_pitch1 = pidSpdCtrl_pitch1.UpdatePidController(Spd_pitch1, SpdMeasure_pitch1);

	mtrOutputBuffer[CComJoint::P1] = static_cast<int16_t>(output_pitch1[0]);
	return APP_OK;
}

/**
 * @brief Pitch2电机输出更新函数
 * 
 * @param posit_pitch2 Pitch2目标位置
 * @return int16_t 电机输出值
 */
EAppStatus CModArm::CComJoint::_UpdateOutput_Pitch2(float_t posit_pitch2) {
	DataBuffer<float_t> Pos_pitch2 = { static_cast<float_t>(posit_pitch2 * ARM_PITCH2_MOTOR_DIR) };
	DataBuffer<float_t> PosMeasure_pitch2 = {static_cast<float_t>(motor[P2]->motorData[CDevMtr::DATA_POSIT])};

	auto Spd_pitch2 = pidPosCtrl_pitch2.UpdatePidController(Pos_pitch2, PosMeasure_pitch2);

	DataBuffer<float_t> SpdMeasure_pitch2 = {static_cast<float_t>(motor[P2]->motorData[CDevMtr::DATA_SPEED])};

	auto output_pitch2 = pidSpdCtrl_pitch2.UpdatePidController(Spd_pitch2, SpdMeasure_pitch2);

	mtrOutputBuffer[CComJoint::P2] = static_cast<int16_t>(output_pitch2[0]);
	return APP_OK;
}

} // namespace my_engineer
