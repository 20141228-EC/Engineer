/******************************************************************************
 * @brief
 *
 * @file         com_storage.cpp
 * @author       ciallo (1002046597@qq.com)
 * @version      V1.0
 * @date         2026-05-26
 *
 * @copyright    Copyright (c) 2026
 *
 ******************************************************************************/

#include "mod_gimbal.hpp"

namespace my_engineer {

/**
 * @brief 初始化存矿石组件
 *
 * @param param
 * @return EAppStatus
 */
EAppStatus CModGimbal::CComStorage::InitComponent(SModInitParam_Base &param) {
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	auto gimbalParam = static_cast<SModInitParam_Gimbal &>(param);

	motor[L] = MotorIDMap.at(gimbalParam.storageMotorID_L);
	motor[R] = MotorIDMap.at(gimbalParam.storageMotorID_R);

	mtrCanTxNode[L] = gimbalParam.storageMotorTxNode_L;
	mtrCanTxNode[R] = gimbalParam.storageMotorTxNode_R;

	gimbalParam.storagePosPidParam_L.threadNum = 1;
	pidPosCtrl_L_storage.InitPID(&gimbalParam.storagePosPidParam_L);
	gimbalParam.storageSpdPidParam_L.threadNum = 1;
	pidSpdCtrl_L_storage.InitPID(&gimbalParam.storageSpdPidParam_L);

	gimbalParam.storagePosPidParam_R.threadNum = 1;
	pidPosCtrl_R_storage.InitPID(&gimbalParam.storagePosPidParam_R);
	gimbalParam.storageSpdPidParam_R.threadNum = 1;
	pidSpdCtrl_R_storage.InitPID(&gimbalParam.storageSpdPidParam_R);

	mtrOutputBuffer.fill(0);

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新存矿石组件
 *
 * @return EAppStatus
 */
EAppStatus CModGimbal::CComStorage::UpdateComponent() {
	if (componentStatus == APP_RESET) {
		mtrOutputBuffer.fill(0);
		return APP_ERROR;
	}

	storageInfo.posit_L_storage = motor[L]->motorData[CDevMtr::DATA_POSIT] * STORAGE_L_MOTOR_DIR;		///<将电机的机械角度更新到组件中
	storageInfo.isPositArrived_L_storage = std::abs(storageInfo.posit_L_storage - storageCmd.setPosit_L_storage) < STORAGE_MOTOR_ARRIVE_ERR;
	storageInfo.posit_R_storage = motor[R]->motorData[CDevMtr::DATA_POSIT] * STORAGE_R_MOTOR_DIR;		///<将电机的机械角度更新到组件中
	storageInfo.isPositArrived_R_storage = std::abs(storageInfo.posit_R_storage - storageCmd.setPosit_R_storage) < STORAGE_MOTOR_ARRIVE_ERR;

	storageInfo.isAvailable = true;

	switch (Component_FSMFlag_) {

		case FSM_RESET: {
			mtrOutputBuffer.fill(0);
			pidPosCtrl_L_storage.ResetPidController();
			pidSpdCtrl_L_storage.ResetPidController();
			pidPosCtrl_R_storage.ResetPidController();
			pidSpdCtrl_R_storage.ResetPidController();
			storageCmd = SStorageCmd();
			return APP_OK;
		}

		case FSM_PREINIT: {
			mtrOutputBuffer.fill(0);
			pidPosCtrl_L_storage.ResetPidController();
			pidSpdCtrl_L_storage.ResetPidController();
			pidPosCtrl_R_storage.ResetPidController();
			pidSpdCtrl_R_storage.ResetPidController();

			motor[L]->motorData[CDevMtr::DATA_POSIT] = motor[L]->motorData[CDevMtr::DATA_ANGLE] - POSIT_STORAGE_L_MACH;
			while(motor[L]->motorData[CDevMtr::DATA_POSIT] < -32767)
				motor[L]->motorData[CDevMtr::DATA_POSIT] += 65535;
			motor[L]->motorData[CDevMtr::DATA_POSIT] += POSIT_STORAGE_L_MACH_PHY * STORAGE_L_MOTOR_SCALE * STORAGE_L_MOTOR_DIR;

			motor[R]->motorData[CDevMtr::DATA_POSIT] = motor[R]->motorData[CDevMtr::DATA_ANGLE] - POSIT_STORAGE_R_MACH;
			while(motor[R]->motorData[CDevMtr::DATA_POSIT] < -32767)
				motor[R]->motorData[CDevMtr::DATA_POSIT] += 65535;
			motor[R]->motorData[CDevMtr::DATA_POSIT] += POSIT_STORAGE_R_MACH_PHY * STORAGE_R_MOTOR_SCALE * STORAGE_R_MOTOR_DIR;

			storageCmd.setPosit_L_storage = PhyPositToMtrPosit_L(STORAGE_L_MOTOR_INIT_POSIT);
			storageCmd.setPosit_R_storage = PhyPositToMtrPosit_R(STORAGE_R_MOTOR_INIT_POSIT);
			Component_FSMFlag_ = FSM_INIT;

			_UpdateOutput(static_cast<float_t>(storageCmd.setPosit_L_storage),
				static_cast<float_t>(storageCmd.setPosit_R_storage));
			return APP_OK;
		}

		case FSM_INIT: {
			if (storageInfo.isPositArrived_L_storage && storageInfo.isPositArrived_R_storage) {				///<如果到了目标的位置
				pidPosCtrl_L_storage.ResetPidController();
				pidSpdCtrl_L_storage.ResetPidController();
				pidPosCtrl_R_storage.ResetPidController();
				pidSpdCtrl_R_storage.ResetPidController();
				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;
				return APP_OK;
			}

			_UpdateOutput(static_cast<float_t>(storageCmd.setPosit_L_storage),
				static_cast<float_t>(storageCmd.setPosit_R_storage));
			return APP_OK;
		}

		case FSM_CTRL: {
			return _UpdateOutput(static_cast<float_t>(storageCmd.setPosit_L_storage),
				static_cast<float_t>(storageCmd.setPosit_R_storage));
		}

		default: {
			StopComponent();
			mtrOutputBuffer.fill(0);
			pidPosCtrl_L_storage.ResetPidController();
			pidSpdCtrl_L_storage.ResetPidController();
			pidPosCtrl_R_storage.ResetPidController();
			pidSpdCtrl_R_storage.ResetPidController();
			componentStatus = APP_ERROR;
			return APP_ERROR;
		}
	}

	return APP_OK;
}

/*------------------------------------------------------------------------------------*/
// 物理位置转换为电机位置（左存矿）
int32_t CModGimbal::CComStorage::PhyPositToMtrPosit_L(float_t phyPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = STORAGE_L_MOTOR_SCALE;

	return static_cast<int32_t>((phyPosit * scale) + zeroOffset);
}

// 电机位置转换为物理位置（左存矿）
float_t CModGimbal::CComStorage::MtrPositToPhyPosit_L(int32_t mtrPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = STORAGE_L_MOTOR_SCALE;

	return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}
/*------------------------------------------------------------------------------------*/
// 物理位置转换为电机位置（右存矿）
int32_t CModGimbal::CComStorage::PhyPositToMtrPosit_R(float_t phyPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = STORAGE_R_MOTOR_SCALE;

	return static_cast<int32_t>((phyPosit * scale) + zeroOffset);
}

// 电机位置转换为物理位置（右存矿）
float_t CModGimbal::CComStorage::MtrPositToPhyPosit_R(int32_t mtrPosit) {
	const int32_t zeroOffset = 0;
	const float_t scale = STORAGE_R_MOTOR_SCALE;

	return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}
/*------------------------------------------------------------------------------------*/
EAppStatus CModGimbal::CComStorage::_UpdateOutput(float_t posit_L, float_t posit_R) {

	DataBuffer<float_t> Pos_L = {static_cast<float_t>(posit_L * STORAGE_L_MOTOR_DIR)};
	DataBuffer<float_t> Pos_R = {static_cast<float_t>(posit_R * STORAGE_R_MOTOR_DIR)};

	DataBuffer<float_t> PosMeasure_L = {static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_POSIT])};
	DataBuffer<float_t> PosMeasure_R = {static_cast<float_t>(motor[R]->motorData[CDevMtr::DATA_POSIT])};

	auto Spd_L = pidPosCtrl_L_storage.UpdatePidController(Pos_L, PosMeasure_L);							///<角度环
	auto Spd_R = pidPosCtrl_R_storage.UpdatePidController(Pos_R, PosMeasure_R);

	DataBuffer<float_t> SpdMeasure_L = {static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_SPEED])};
	DataBuffer<float_t> SpdMeasure_R = {static_cast<float_t>(motor[R]->motorData[CDevMtr::DATA_SPEED])};

	auto output_L = pidSpdCtrl_L_storage.UpdatePidController(Spd_L, SpdMeasure_L);						///<速度环
	auto output_R = pidSpdCtrl_R_storage.UpdatePidController(Spd_R, SpdMeasure_R);

	mtrOutputBuffer = {
		static_cast<int16_t>(output_L[0]),
		static_cast<int16_t>(output_R[0])};

	return APP_OK;
}

/**
 * @brief 左存矿电机输出更新函数，单独控制两个存矿石电机
 *
 * @param posit_L 左电机目标位置
 * @return EAppStatus
 */
EAppStatus CModGimbal::CComStorage::_UpdateOutput_L(float_t posit_L) {
	if (motor[L] == nullptr) {
		mtrOutputBuffer[L] = 0;
		return APP_OK;
	}

	DataBuffer<float_t> Pos_L = {static_cast<float_t>(posit_L * STORAGE_L_MOTOR_DIR)};
	DataBuffer<float_t> PosMeasure_L = {static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_POSIT])};

	auto Spd_L = pidPosCtrl_L_storage.UpdatePidController(Pos_L, PosMeasure_L);

	DataBuffer<float_t> SpdMeasure_L = {static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_SPEED])};

	auto output_L = pidSpdCtrl_L_storage.UpdatePidController(Spd_L, SpdMeasure_L);

	mtrOutputBuffer[L] = static_cast<int16_t>(output_L[0]);
	return APP_OK;
}

/**
 * @brief 右存矿电机输出更新函数
 *
 * @param posit_R 右电机目标位置
 * @return EAppStatus
 */
EAppStatus CModGimbal::CComStorage::_UpdateOutput_R(float_t posit_R) {
	if (motor[R] == nullptr) {
		mtrOutputBuffer[R] = 0;
		return APP_OK;
	}

	DataBuffer<float_t> Pos_R = {static_cast<float_t>(posit_R * STORAGE_R_MOTOR_DIR)};
	DataBuffer<float_t> PosMeasure_R = {static_cast<float_t>(motor[R]->motorData[CDevMtr::DATA_POSIT])};

	auto Spd_R = pidPosCtrl_R_storage.UpdatePidController(Pos_R, PosMeasure_R);

	DataBuffer<float_t> SpdMeasure_R = {static_cast<float_t>(motor[R]->motorData[CDevMtr::DATA_SPEED])};

	auto output_R = pidSpdCtrl_R_storage.UpdatePidController(Spd_R, SpdMeasure_R);

	mtrOutputBuffer[R] = static_cast<int16_t>(output_R[0]);
	return APP_OK;
}
void CModGimbal::CComStorage::ChooseStoreOre(Eore_station ore_station_){
	auto idx = static_cast<size_t>(ore_station_);
	if (idx >= 1 && idx <= 6) {
		if (idx <= 3) {
			storageCmd.setPosit_R_storage = PhyPositToMtrPosit_R(OreStationAngle[idx]);
			} else {
				storageCmd.setPosit_L_storage = PhyPositToMtrPosit_L(OreStationAngle[idx]);
			}
	}
}
} // namespace my_engineer
