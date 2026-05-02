/**
 * @file com_end.cpp
 * @author ciallo (1002046597@qq.com)
 * @brief 机械臂末端组件
 * @version 1.1
 * @date 2025-01-14
 * @lastedit：2026-04-30
 * @details V1.1: 两阶段初始化：先标定Pitch（堵转），再标定Roll（堵转）
 *
 * @copyright Copyright (c) 2025
 *
 */


#include "mod_arm.hpp"
float_t end_l_test = 0.f;
float_t end_r_test = 0.f;

namespace my_engineer {

/**
 * @brief 初始化机械臂末端组件
 */
EAppStatus CModArm::CComEnd::InitComponent(SModInitParam_Base &param) {
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	auto armParam = static_cast<SModInitParam_Arm &>(param);

	motor[L] = MotorIDMap.at(armParam.MotorID_End_L);
	motor[R] = MotorIDMap.at(armParam.MotorID_End_R);

	mtrCanTxNode[L] = armParam.MotorTxNode_End_L;
	mtrCanTxNode[R] = armParam.MotorTxNode_End_R;

	armParam.endPosPidParam.threadNum = 2;
	pidPosCtrl.InitPID(&armParam.endPosPidParam);

	armParam.endSpdPidParam.threadNum = 2;
	pidSpdCtrl.InitPID(&armParam.endSpdPidParam);

	mtrOutputBuffer.fill(0);

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新组件
 */
EAppStatus CModArm::CComEnd::UpdateComponent() {
	if (componentStatus == APP_RESET) {
		mtrOutputBuffer.fill(0);
		return APP_ERROR;
	}

	endInfo.posit_Roll 	=  (motor[L]->motorData[CDevMtr::DATA_POSIT] + motor[R]->motorData[CDevMtr::DATA_POSIT]) / 2;
	endInfo.posit_Pitch = ((motor[R]->motorData[CDevMtr::DATA_POSIT] - endInfo.posit_Roll) - (motor[L]->motorData[CDevMtr::DATA_POSIT] - endInfo.posit_Roll))/2.0;

	endInfo.isPositArrived_Pitch = (abs(endCmd.setPosit_Pitch - endInfo.posit_Pitch) < 8192 * 2);
	endInfo.isPositArrived_Roll = (abs(endCmd.setPosit_Roll - endInfo.posit_Roll) < 8192 * 2);

	switch (Component_FSMFlag_) {

		case FSM_RESET: {
			mtrOutputBuffer.fill(0);
			endCmd = SEndCmd{};
			pitchCalibrated_ = false;
			rollCalibrated_ = false;
			initState_ = EEndInitState::PITCH;
			initStateTick_ = 0;
			pidPosCtrl.ResetPidController();
			pidSpdCtrl.ResetPidController();
			return APP_OK;
		}

		case FSM_PREINIT: {
			endCmd = SEndCmd{};
			motor[L]->motorData[CDevMtr::DATA_POSIT] = 0;
			motor[R]->motorData[CDevMtr::DATA_POSIT] = 0;
			mtrOutputBuffer.fill(0);
			pitchCalibrated_ = false;
			rollCalibrated_ = false;
			initState_ = EEndInitState::PITCH;
			initStateTick_ = 0;
			pidPosCtrl.ResetPidController();
			pidSpdCtrl.ResetPidController();
			Component_FSMFlag_ = FSM_INIT;
			return APP_OK;
		}

		case FSM_INIT: {
			switch (initState_) {
				case EEndInitState::PITCH: {//pitch轴的标定
					endCmd.setPosit_Roll = 0;

					if (!pitchCalibrated_) {
						if ((motor[L]->motorStatus == CDevMtr::EMotorStatus::STALL) &&
							(motor[R]->motorStatus == CDevMtr::EMotorStatus::STALL)) {
							motor[L]->motorData[CDevMtr::DATA_POSIT] = -(static_cast<int32_t>(0.5f * 8192) + rangeLimit_Pitch);
							motor[R]->motorData[CDevMtr::DATA_POSIT] = (static_cast<int32_t>(0.5f * 8192) + rangeLimit_Pitch);
							mtrOutputBuffer.fill(0);
							pidPosCtrl.ResetPidController();
							pidSpdCtrl.ResetPidController();
							pitchCalibrated_ = true;
							initStateTick_ = 0;
							return APP_OK;
						}

						endCmd.setPosit_Pitch += 200;
						return _UpdateOutput(static_cast<float_t>(endCmd.setPosit_Pitch),
											static_cast<float_t>(endCmd.setPosit_Roll));
					}

					endCmd.setPosit_Pitch = PhyPositToMtrPosit_Pitch(ARM_END_PITCH_INIT_ANGLE);

					if (abs(endInfo.posit_Pitch - endCmd.setPosit_Pitch) < 8192 * 2) { //末端pitch的到位检查
						mtrOutputBuffer.fill(0);
						pidPosCtrl.ResetPidController();
						pidSpdCtrl.ResetPidController();
						initState_ = EEndInitState::ROLL;
						initStateTick_ = 0;
						return APP_OK;
					}

					return _UpdateOutput(static_cast<float_t>(endCmd.setPosit_Pitch),
										static_cast<float_t>(endCmd.setPosit_Roll));
				}

				case EEndInitState::ROLL: {//末端roll的标定
					endCmd.setPosit_Pitch = PhyPositToMtrPosit_Pitch(ARM_END_PITCH_INIT_ANGLE);
					if (!rollCalibrated_) {
						endCmd.setPosit_Roll += 200;
						if (initStateTick_ < 150) {
							++initStateTick_;			//避免pitch轴堵转标定的残留
							return _UpdateOutput(static_cast<float_t>(endCmd.setPosit_Pitch),
												static_cast<float_t>(endCmd.setPosit_Roll));
						}

						if ((motor[L]->motorStatus == CDevMtr::EMotorStatus::STALL) &&
							(motor[R]->motorStatus == CDevMtr::EMotorStatus::STALL)) {
							rollZeroOffset = endInfo.posit_Roll -
								static_cast<int32_t>(ARM_END_ROLL_STALL_ANGLE * ARM_END_ROLL_MOTOR_RATIO);//减去末端roll的零偏
							endCmd.setPosit_Roll = PhyPositToMtrPosit_Roll(ARM_END_ROLL_INIT_ANGLE);
							mtrOutputBuffer.fill(0);
							pidPosCtrl.ResetPidController();
							pidSpdCtrl.ResetPidController();
							rollCalibrated_ = true;
							initStateTick_ = 0;
							return APP_OK;
						}

						return _UpdateOutput(static_cast<float_t>(endCmd.setPosit_Pitch),
											static_cast<float_t>(endCmd.setPosit_Roll));
					}

					endCmd.setPosit_Roll = PhyPositToMtrPosit_Roll(ARM_END_ROLL_INIT_ANGLE);

					if (abs(endInfo.posit_Roll - endCmd.setPosit_Roll) < 8192 * 2) { //末端roll到位检查
						mtrOutputBuffer.fill(0);
						pidPosCtrl.ResetPidController();
						pidSpdCtrl.ResetPidController();
						initState_ = EEndInitState::DONE; //初始化检查的状态
						componentStatus = APP_OK;
						Component_FSMFlag_ = FSM_CTRL;
						return APP_OK;
					}

					return _UpdateOutput(static_cast<float_t>(endCmd.setPosit_Pitch),
										static_cast<float_t>(endCmd.setPosit_Roll));
				}

				case EEndInitState::DONE: {
					componentStatus = APP_OK;
					Component_FSMFlag_ = FSM_CTRL;
					return APP_OK;
				}
			}
			return APP_OK;
		}

		case FSM_CTRL: {
			end_l_test = motor[L]->motorData[CDevMtr::DATA_TORQUE];
			end_r_test = motor[R]->motorData[CDevMtr::DATA_TORQUE];
			return _UpdateOutput(static_cast<float_t>(endCmd.setPosit_Pitch),
								static_cast<float_t>(endCmd.setPosit_Roll));
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

	return APP_OK;
}

/**
 * @brief 物理位置转换为电机位置
 * 
 * @param phyPosit 
 * @return int32_t 
 */
int32_t CModArm::CComEnd::PhyPositToMtrPosit_Pitch(float_t phyPosit) {
	const int32_t zeroOffset = ARM_END_PITCH_MOTOR_OFFSET;
	const float_t scale = 1590.2117f;

	return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}

int32_t CModArm::CComEnd::PhyPositToMtrPosit_Roll(float_t phyPosit) {
	const float_t scale = ARM_END_ROLL_MOTOR_RATIO;

	return (static_cast<int32_t>(phyPosit * scale) + rollZeroOffset);
}

/**
 * @brief 电机位置转换为物理位置
 * 
 * @param mtrPosit 
 * @return float_t 
 */
float_t CModArm::CComEnd::MtrPositToPhyPosit_Pitch(int32_t mtrPosit) {
	const int32_t zeroOffset = ARM_END_PITCH_MOTOR_OFFSET;
	const float_t scale = 1590.2117f;

	return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}

float_t CModArm::CComEnd::MtrPositToPhyPosit_Roll(int32_t mtrPosit) {
	const float_t scale = ARM_END_ROLL_MOTOR_RATIO;

	return (static_cast<float_t>(mtrPosit - rollZeroOffset) / scale);
}

/**
 * @brief 输出更新函数
 * 
 * @param posit_Pitch 
 * @param posit_Roll 
 * @return EAppStatus 
 */
EAppStatus CModArm::CComEnd::_UpdateOutput(float_t posit_Pitch, float_t posit_Roll) {
	DataBuffer<float_t> endPos = {
    posit_Roll - posit_Pitch,
    posit_Roll + posit_Pitch,
	};

	DataBuffer<float_t> endPosMeasure = {
		static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_POSIT]),
		static_cast<float_t>(motor[R]->motorData[CDevMtr::DATA_POSIT])
	};

	auto endSpd = pidPosCtrl.UpdatePidController(endPos, endPosMeasure);

	DataBuffer<float_t> endSpdMeasure = {
		static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_SPEED]),
		static_cast<float_t>(motor[R]->motorData[CDevMtr::DATA_SPEED])
	};

	auto output = pidSpdCtrl.UpdatePidController(endSpd, endSpdMeasure);

	mtrOutputBuffer = {
		static_cast<int16_t>(output[L]),
		static_cast<int16_t>(output[R])
	};

	return APP_OK;
}

} // namespace my_engineer
