/******************************************************************************
 * @brief        
 * 
 * @file         com_joint.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2026-01-27
 * 
 * @copyright    Copyright (c) 2026
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
	motor[P3] = MotorIDMap.at(armParam.MotorID_Pitch3);

	mtrCanTxNode[Y] = armParam.MotorTxNode_Yaw;
	mtrCanTxNode[P1] = armParam.MotorTxNode_Pitch1;
	mtrCanTxNode[P2] = armParam.MotorTxNode_Pitch2;
	mtrCanTxNode[P3] = armParam.MotorTxNode_Pitch3;

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

	armParam.Pitch3PosPidParam.threadNum = 1;
	pidPosCtrl_pitch3.InitPID(&armParam.Pitch3PosPidParam);
	armParam.Pitch3SpdPidParam.threadNum = 1;
	pidSpdCtrl_pitch3.InitPID(&armParam.Pitch3SpdPidParam);

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
	static bool isreset_flag = false;	///<总初始化标志位
	static bool alreadySetYaw = false;	///<yaw设置标志位
	if (componentStatus == APP_RESET) {
		mtrOutputBuffer.fill(0);
		return APP_ERROR;
	}

	jointInfo.posit_yaw = motor[Y]->motorData[CDevMtr::DATA_POSIT] * ARM_YAW_MOTOR_DIR;
	jointInfo.posit_pitch1 = motor[P1]->motorData[CDevMtr::DATA_POSIT] * ARM_PITCH1_MOTOR_DIR;
	jointInfo.posit_pitch2 = motor[P2]->motorData[CDevMtr::DATA_POSIT] * ARM_PITCH2_MOTOR_DIR;	  ///<将电机的机械角度更新到关节类中
	jointInfo.posit_pitch3 = motor[P3]->motorData[CDevMtr::DATA_POSIT] * ARM_PITCH3_MOTOR_DIR;
	
	jointInfo.isPositArrived_yaw = abs(jointInfo.posit_yaw - jointCmd.setPosit_yaw) < 500;
	jointInfo.isPositArrived_pitch1 = abs(jointInfo.posit_pitch1 - jointCmd.setPosit_pitch1) < 700;
	jointInfo.isPositArrived_pitch2 = abs(jointInfo.posit_pitch2 - jointCmd.setPosit_pitch2) < 700;
	jointInfo.isPositArrived_pitch3 = abs(jointInfo.posit_pitch3 - jointCmd.setPosit_pitch3) < 700;

	switch (Component_FSMFlag_) {

		case FSM_RESET: {
			mtrOutputBuffer.fill(0);
			pidPosCtrl_yaw.ResetPidController();
			pidSpdCtrl_yaw.ResetPidController();
			pidPosCtrl_pitch1.ResetPidController();
			pidSpdCtrl_pitch1.ResetPidController();
			pidPosCtrl_pitch2.ResetPidController();
			pidSpdCtrl_pitch2.ResetPidController();
			pidPosCtrl_pitch3.ResetPidController();
			pidSpdCtrl_pitch3.ResetPidController();
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
				pidPosCtrl_pitch3.ResetPidController();
				pidSpdCtrl_pitch3.ResetPidController();
				/*设置每个关节的绝对角度*/
				motor[Y]->motorData[CDevMtr::DATA_POSIT]  = motor[Y]->motorData[CDevMtr::DATA_ANGLE] - POSIT_JOINT1_YAW_MACH;
				while(motor[Y]->motorData[CDevMtr::DATA_POSIT] < -32767)
					motor[Y]->motorData[CDevMtr::DATA_POSIT] += 65535;
				while(motor[Y]->motorData[CDevMtr::DATA_POSIT] > 32767)
					motor[Y]->motorData[CDevMtr::DATA_POSIT] -= 65535;
				motor[Y]->motorData[CDevMtr::DATA_POSIT]  += POSIT_JOINT1_YAW_MACH_PHY * 182.04f * POSIT_JOINT1_YAW_MACH;	
				jointCmd.setPosit_yaw =  PhyPositToMtrPosit_yaw(ARM_INIT_SAFE_YAW_ANGLE);	
				
				// motor[Y]->motorData[CDevMtr::DATA_POSIT]  = motor[Y]->motorData[CDevMtr::DATA_ANGLE] * ARM_YAW_MOTOR_DIR;

				motor[P1]->motorData[CDevMtr::DATA_POSIT] = motor[P1]->motorData[CDevMtr::DATA_ANGLE] - POSIT_JOINT2_PITCH1_MACH;		///<刚上电的时候获取初始值.距离机械中值的偏差
				while(motor[P1]->motorData[CDevMtr::DATA_POSIT] < -32767)
					motor[P1]->motorData[CDevMtr::DATA_POSIT] += 65535;//归位到-32767~32768范围内
				motor[P1]->motorData[CDevMtr::DATA_POSIT] +=	POSIT_JOINT2_PITCH1_MACH_PHY * 182.04f * ARM_PITCH1_MOTOR_DIR;			///<这个是等效连杆和水平面的夹角
				jointCmd.setPosit_pitch1 = PhyPositToMtrPosit_pitch1(ARM_INIT_SAFE_PITCH1_ANGLE);

				motor[P2]->motorData[CDevMtr::DATA_POSIT] = motor[P2]->motorData[CDevMtr::DATA_ANGLE] - POSIT_JOINT3_PITCH2_MACH;
				while(motor[P2]->motorData[CDevMtr::DATA_POSIT] < -32767)
					motor[P2]->motorData[CDevMtr::DATA_POSIT] += 65535;
				motor[P2]->motorData[CDevMtr::DATA_POSIT]  += ARM_PITCH2_MOTOR_DIR * POSIT_JOINT3_PITCH2_MACH_PHY * 182.04f;			///<这个是等效连杆和水平面的夹角
				jointCmd.setPosit_pitch2 = PhyPositToMtrPosit_pitch2(ARM_INIT_SAFE_PITCH2_ANGLE);
				
				motor[P3]->motorData[CDevMtr::DATA_POSIT] = motor[P3]->motorData[CDevMtr::DATA_ANGLE] - POSIT_JOINT4_PITCH3_MACH;
				while(motor[P3]->motorData[CDevMtr::DATA_POSIT] < -32767)
					motor[P3]->motorData[CDevMtr::DATA_POSIT] += 65535;
				motor[P3]->motorData[CDevMtr::DATA_POSIT]  += ARM_PITCH3_MOTOR_DIR * POSIT_JOINT4_PITCH3_MACH_PHY * 182.04f;
				jointCmd.setPosit_pitch3 = PhyPositToMtrPosit_pitch3(ARM_INIT_SAFE_PITCH3_ANGLE);
				alreadySetYaw = false;
				isreset_flag = true;  // 重置标志
			}	
			else{
				/*全部到位后才进入初始化 - 优先级最高*/
				if(jointInfo.isPositArrived_pitch3 &&jointInfo.isPositArrived_pitch2 && jointInfo.isPositArrived_pitch1 && alreadySetYaw == true){
					if(abs(jointInfo.posit_yaw - jointCmd.setPosit_yaw)<500){									///<包含了堵转和未堵转两种标定yaw零点的情况
						motor[Y]->motorData[CDevMtr::DATA_POSIT] = 0.0f;
						jointCmd.setPosit_yaw = 0;
						jointCmd.setPosit_pitch1 = PhyPositToMtrPosit_pitch1(ARM_INIT_SAFE_PITCH1_ANGLE);
						jointCmd.setPosit_pitch2 = PhyPositToMtrPosit_pitch2(ARM_INIT_SAFE_PITCH2_ANGLE);
						jointCmd.setPosit_pitch3 = PhyPositToMtrPosit_pitch3(ARM_INIT_SAFE_PITCH3_ANGLE);
						Component_FSMFlag_ = FSM_INIT;
						return APP_OK;
					}
					// else if(motor[Y]->motorStatus == CDevMtr::EMotorStatus::STALL){				///<通过堵转来重新标定零点				
					// 	motor[Y]->motorData[CDevMtr::DATA_POSIT] = 30768 * ARM_YAW_MOTOR_DIR; ///< 30768是Yaw电机的初始位置
					// 	jointCmd.setPosit_yaw = 0;
					// }
					_UpdateOutput(static_cast<float_t>(jointCmd.setPosit_yaw),
						static_cast<float_t>(jointCmd.setPosit_pitch1),
						static_cast<float_t>(jointCmd.setPosit_pitch2),
						static_cast<float_t>(jointCmd.setPosit_pitch3));
					for (auto &out : mtrOutputBuffer) out = std::clamp(out, static_cast<int16_t>(-3000), static_cast<int16_t>(3000));//初始化的时候限制输出防止撞到灯条，遍历所有输出数组
					return APP_OK;
				}
				/*全部到位后才进入初始化*/
				else if(jointInfo.isPositArrived_pitch3 && jointInfo.isPositArrived_pitch2 && jointInfo.isPositArrived_pitch1 && alreadySetYaw == false){
					jointCmd.setPosit_yaw = 0;								///<yaw轴在p1,p2抬升到安全位置之后才动
					alreadySetYaw = true;
					_UpdateOutput(static_cast<float_t>(jointCmd.setPosit_yaw),
						static_cast<float_t>(jointCmd.setPosit_pitch1),
						static_cast<float_t>(jointCmd.setPosit_pitch2),
						static_cast<float_t>(jointCmd.setPosit_pitch3));
					for (auto &out : mtrOutputBuffer) out = std::clamp(out, static_cast<int16_t>(-3000), static_cast<int16_t>(3000));
					return APP_OK;
				}
				else if(motor[Y]->motorStatus == CDevMtr::EMotorStatus::STALL){				///<通过堵转来重新标定零点,防止编码器值回绕
						motor[Y]->motorData[CDevMtr::DATA_POSIT] = motor[Y]->motorData[CDevMtr::DATA_ANGLE] - POSIT_JOINT1_YAW_MACH; ///< 是Yaw电机的初始位置
						jointCmd.setPosit_yaw = 0;
				}
				/*先抬起两个臂后，yaw才能动 - 至少有一个pitch没到位*/
				else {
					_UpdateOutput_Pitch2(jointCmd.setPosit_pitch2);
					_UpdateOutput_Pitch1(jointCmd.setPosit_pitch1);
					_UpdateOutput_Pitch3(jointCmd.setPosit_pitch3);
					// Yaw保持不动
					mtrOutputBuffer[Y] = 0;
					for (auto &out : mtrOutputBuffer) out = std::clamp(out, static_cast<int16_t>(-3000), static_cast<int16_t>(3000));
					return APP_OK;
				}
			}

			return APP_OK;
		}

		case FSM_INIT: {
			if (jointInfo.isPositArrived_yaw && jointInfo.isPositArrived_pitch1 && jointInfo.isPositArrived_pitch2 && jointInfo.isPositArrived_pitch3) {			///<如果到了目标的位置
				jointCmd.setPosit_yaw = 0;
				jointCmd.setPosit_pitch1 = PhyPositToMtrPosit_pitch1(ARM_INIT_SAFE_PITCH1_ANGLE);
				jointCmd.setPosit_pitch2 = PhyPositToMtrPosit_pitch2(ARM_INIT_SAFE_PITCH2_ANGLE);
				jointCmd.setPosit_pitch3 = PhyPositToMtrPosit_pitch3(ARM_INIT_SAFE_PITCH3_ANGLE);
				pidPosCtrl_yaw.ResetPidController();																			
				pidSpdCtrl_yaw.ResetPidController();
				pidPosCtrl_pitch1.ResetPidController();
				pidSpdCtrl_pitch1.ResetPidController();
				pidPosCtrl_pitch2.ResetPidController();
				pidSpdCtrl_pitch2.ResetPidController();
				pidPosCtrl_pitch3.ResetPidController();
				pidSpdCtrl_pitch3.ResetPidController();
				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;
				return APP_OK;
			}
			_UpdateOutput(static_cast<float_t>(0),
				static_cast<float_t>(PhyPositToMtrPosit_pitch1(ARM_INIT_SAFE_PITCH1_ANGLE)),
				static_cast<float_t>(PhyPositToMtrPosit_pitch2(ARM_INIT_SAFE_PITCH2_ANGLE)),
				static_cast<float_t>(PhyPositToMtrPosit_pitch3(ARM_INIT_SAFE_PITCH3_ANGLE)));
				for (auto &out : mtrOutputBuffer) out = std::clamp(out, static_cast<int16_t>(-3000), static_cast<int16_t>(3000));
			return APP_OK;
		}

		case FSM_CTRL: {
			//jointCmd.setPosit_yaw = std::clamp(jointCmd.setPosit_yaw, static_cast<int32_t>(-rangeLimit_yaw/2),  static_cast<int32_t>(rangeLimit_yaw/2));///<对Yaw进行机械限位
			
			//is_record = true; ///< 臂初始化完之后开始记录数据
			return _UpdateOutput(static_cast<float_t>(jointCmd.setPosit_yaw),
				static_cast<float_t>(jointCmd.setPosit_pitch1),
				static_cast<float_t>(jointCmd.setPosit_pitch2),
				static_cast<float_t>(jointCmd.setPosit_pitch3));
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
			pidPosCtrl_pitch3.ResetPidController();
			pidSpdCtrl_pitch3.ResetPidController();
			componentStatus = APP_ERROR;
			return APP_ERROR;
		}
	}

	return APP_OK;
}



/*------------------------------------------------------------------------------------*/
// 物理位置转换为电机位置
int32_t CModArm::CComJoint::PhyPositToMtrPosit_yaw(float_t phyPosit) {
	const float_t scale = -1.15*65535/360.f;

	return static_cast<int32_t>(phyPosit * scale);
}

// 电机位置转换为物理位置
float_t CModArm::CComJoint::MtrPositToPhyPosit_yaw(int32_t mtrPosit) {
	const float_t scale = -1.15*65535/360.f;

	return static_cast<float_t>(mtrPosit) / scale;
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
// 物理位置转换为电机位置
int32_t CModArm::CComJoint::PhyPositToMtrPosit_pitch3(float_t phyPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = 182.04f;

	return static_cast<int32_t>((phyPosit * scale) + zeroOffset);
}

// 电机位置转换为物理位置
float_t CModArm::CComJoint::MtrPositToPhyPosit_pitch3(int32_t mtrPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = 182.04f;

	return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}
/*------------------------------------------------------------------------------------*/
// 输出更新函数
EAppStatus CModArm::CComJoint::_UpdateOutput(float_t posit_yaw, float_t posit_pitch1, float_t posit_pitch2, float_t posit_pitch3) {

	DataBuffer<float_t> Pos_yaw = { static_cast<float_t>(posit_yaw * ARM_YAW_MOTOR_DIR) };
	DataBuffer<float_t> Pos_pitch1 = { static_cast<float_t>(posit_pitch1 * ARM_PITCH1_MOTOR_DIR) };
	DataBuffer<float_t> Pos_pitch2 = { static_cast<float_t>(posit_pitch2 * ARM_PITCH2_MOTOR_DIR) };
	DataBuffer<float_t> Pos_pitch3 = { static_cast<float_t>(posit_pitch3 * ARM_PITCH3_MOTOR_DIR) };			 			 ///<更新目标角度

	DataBuffer<float_t> PosMeasure_yaw = {static_cast<float_t>(motor[Y]->motorData[CDevMtr::DATA_POSIT])};
	DataBuffer<float_t> PosMeasure_pitch1 = {static_cast<float_t>(motor[P1]->motorData[CDevMtr::DATA_POSIT])};
	DataBuffer<float_t> PosMeasure_pitch2 = {static_cast<float_t>(motor[P2]->motorData[CDevMtr::DATA_POSIT])};
	DataBuffer<float_t> PosMeasure_pitch3 = {static_cast<float_t>(motor[P3]->motorData[CDevMtr::DATA_POSIT])};///<获取测量值

	auto Spd_yaw = pidPosCtrl_yaw.UpdatePidController(Pos_yaw, PosMeasure_yaw);								///<角度环
	auto Spd_pitch1 = pidPosCtrl_pitch1.UpdatePidController(Pos_pitch1, PosMeasure_pitch1);
	auto Spd_pitch2 = pidPosCtrl_pitch2.UpdatePidController(Pos_pitch2, PosMeasure_pitch2);
	auto Spd_pitch3 = pidPosCtrl_pitch3.UpdatePidController(Pos_pitch3, PosMeasure_pitch3);

	DataBuffer<float_t> SpdMeasure_yaw = {static_cast<float_t>(motor[Y]->motorData[CDevMtr::DATA_SPEED])};
	DataBuffer<float_t> SpdMeasure_pitch1 = {static_cast<float_t>(motor[P1]->motorData[CDevMtr::DATA_SPEED])};
	DataBuffer<float_t> SpdMeasure_pitch2 = {static_cast<float_t>(motor[P2]->motorData[CDevMtr::DATA_SPEED])};
	DataBuffer<float_t> SpdMeasure_pitch3 = {static_cast<float_t>(motor[P3]->motorData[CDevMtr::DATA_SPEED])};

	auto output_yaw = pidSpdCtrl_yaw.UpdatePidController(Spd_yaw, SpdMeasure_yaw);							///<速度环
	auto output_pitch1 = pidSpdCtrl_pitch1.UpdatePidController(Spd_pitch1, SpdMeasure_pitch1);
	auto output_pitch2 = pidSpdCtrl_pitch2.UpdatePidController(Spd_pitch2, SpdMeasure_pitch2);
	auto output_pitch3 = pidSpdCtrl_pitch3.UpdatePidController(Spd_pitch3, SpdMeasure_pitch3);

	// if(is_record)
	// {
	// 	if(Is_Recording_ArmTorque) ///< 正在记录数据
	// 	{
	// 		if(index < RECORD_MAX - 1) 
	// 		{
	// 			arm_Info[PITCH1][index] = output_pitch1; ///< 大p的扭矩
	// 			arm_Info[PITCH2][index] = output_pitch2; ///< 小p的扭矩
	// 			index ++;
	// 			///< 这里还差用来传输数据的代码
	// 		}
	// 		else ///< 数据记录完毕
	// 		{
	// 			Is_Recording_ArmTorque = false; ///< 停止记录数据
	// 		}
	// 	}	
	// }

	if (onlyGravity_) {
		output_pitch1[0] = this->grav_ff_pitch1;
		output_pitch2[0] = this->grav_ff_pitch2;
		output_pitch3[0] = this->grav_ff_pitch3;
	} else {
		output_pitch1[0] += this->grav_ff_pitch1;
		output_pitch2[0] += this->grav_ff_pitch2;
		output_pitch3[0] += this->grav_ff_pitch3;
	}
	
	mtrOutputBuffer = { 
		static_cast<int16_t>(output_yaw[0]),
		static_cast<int16_t>(output_pitch1[0]),
		static_cast<int16_t>(output_pitch2[0]), 
	    static_cast<int16_t>(output_pitch3[0])};

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

/**
 * @brief Pitch3电机输出更新函数
 * 
 * @param posit_pitch3 Pitch3目标位置
 * @retval int16_t 电机输出值
 */
EAppStatus CModArm::CComJoint::_UpdateOutput_Pitch3(float_t posit_pitch3) {
	DataBuffer<float_t> Pos_pitch3 = { static_cast<float_t>(posit_pitch3 * ARM_PITCH3_MOTOR_DIR) };
	DataBuffer<float_t> PosMeasure_pitch3 = {static_cast<float_t>(motor[P3]->motorData[CDevMtr::DATA_POSIT])};

	auto Spd_pitch3 = pidPosCtrl_pitch3.UpdatePidController(Pos_pitch3, PosMeasure_pitch3);

	DataBuffer<float_t> SpdMeasure_pitch3 = {static_cast<float_t>(motor[P3]->motorData[CDevMtr::DATA_SPEED])};

	auto output_pitch3 = pidSpdCtrl_pitch3.UpdatePidController(Spd_pitch3, SpdMeasure_pitch3);

	mtrOutputBuffer[CComJoint::P3] = static_cast<int16_t>(output_pitch3[0]);
	return APP_OK;
}

} // namespace my_engineer
