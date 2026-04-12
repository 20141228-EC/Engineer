// /**
//  * @file com_storage.cpp
//  * @author sllllr
//  * @brief 存矿组件
//  * @version 2.0
//  * @date 2026-03-11
//  *
//  * @copyright Copyright (c) 2026
//  *
//  */

// #include "mod_gimbal.hpp"

// namespace my_engineer {

// /**
//  * @brief 初始化存矿组件
//  *
//  * @param param
//  * @return EAppStatus
//  */
// EAppStatus CModGimbal::CComStorage::InitComponent(SModInitParam_Base &param){
// 	// 检查param是否正确
// 	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

// 	// 类型转换
// 	auto gimbalParam = static_cast<SModInitParam_Gimbal &>(param);

// 	// 保存双电机指针
// 	motor[F] = MotorIDMap.at(gimbalParam.storageMotorID_F);
// 	motor[B] = MotorIDMap.at(gimbalParam.storageMotorID_B);

// 	// 设置CAN发送节点
// 	mtrCanTxNode[F] = gimbalParam.storageMotorTxNode_F;
// 	mtrCanTxNode[B] = gimbalParam.storageMotorTxNode_B;

// 	// 初始化PID控制器 (threadNum = 2 用于双电机)
// 	gimbalParam.storagePosPidParam.threadNum = 2;
// 	pidPosCtrl.InitPID(&gimbalParam.storagePosPidParam);

// 	gimbalParam.storageSpdPidParam.threadNum = 2;
// 	pidSpdCtrl.InitPID(&gimbalParam.storageSpdPidParam);

// 	// 初始化电机数据输出缓冲区
// 	mtrOutputBuffer.fill(0);

// 	Component_FSMFlag_ = FSM_RESET;
// 	componentStatus = APP_OK;

// 	return APP_OK;
// }

// /**
//  * @brief 更新组件
//  *
//  */
// EAppStatus CModGimbal::CComStorage::UpdateComponent() {
// 	// 检查组件状态
// 	if (componentStatus == APP_RESET) return APP_ERROR;
	
//     // 安全检查：如果电机指针为空，则返回错误
//     if (motor[F] == nullptr || motor[B] == nullptr) {
//         return APP_ERROR;
//     }

// 	// 更新组件信息
// 	storageInfo.posit_F = (motor[F]->motorData[CDevMtr::DATA_POSIT] * GIMBAL_STORAGE_MOTOR_DIR_F);
// 	storageInfo.isPositArrived_F = (abs(storageCmd.setPosit * GIMBAL_STORAGE_MOTOR_DIR_F - storageInfo.posit_F) < 8192 * 1);     // 这个系数待调
//     storageInfo.posit_B = (motor[B]->motorData[CDevMtr::DATA_POSIT] * GIMBAL_STORAGE_MOTOR_DIR_B);
// 	storageInfo.isPositArrived_B = (abs(storageCmd.setPosit * GIMBAL_STORAGE_MOTOR_DIR_B - storageInfo.posit_B) < 8192 * 1);     // 这个系数待调

// 	switch (Component_FSMFlag_){
// 		case FSM_RESET: {
// 			// 重置状态下，电机数据输出缓冲区始终为0
// 			mtrOutputBuffer.fill(0);
// 			pidPosCtrl.ResetPidController();
// 			pidSpdCtrl.ResetPidController();
// 			return APP_OK;
// 		}

// 		case FSM_PREINIT: {
// 			// 预初始化状态下，清零目标值和电机位置
// 			storageCmd.setPosit = static_cast<int32_t>(0);
// 			motor[F]->motorData[CDevMtr::DATA_POSIT] = 0;
// 			motor[B]->motorData[CDevMtr::DATA_POSIT] = 0;
// 			mtrOutputBuffer.fill(0);
// 			pidPosCtrl.ResetPidController();
// 			pidSpdCtrl.ResetPidController();
// 			Component_FSMFlag_ = FSM_INIT;
// 			return APP_OK;
// 		}

// 		case FSM_INIT: {
// 			// 任一电机堵转，说明初始化完成
// 			if (motor[F]->motorStatus == CDevMtr::EMotorStatus::STALL ||
// 				motor[B]->motorStatus == CDevMtr::EMotorStatus::STALL) {
// 				storageCmd = SStorageCmd();	// 堵转之后清零目标值
// 				// 补偿超出限位的值
// 				motor[F]->motorData[CDevMtr::DATA_POSIT] = static_cast<int32_t>(0.1 * 8192 + rangeLimit) * GIMBAL_STORAGE_MOTOR_DIR_F;
// 				motor[B]->motorData[CDevMtr::DATA_POSIT] = static_cast<int32_t>(0.1 * 8192 + rangeLimit) * GIMBAL_STORAGE_MOTOR_DIR_B;
// 				pidPosCtrl.ResetPidController();
// 				pidSpdCtrl.ResetPidController();
// 				Component_FSMFlag_ = FSM_CTRL;
// 				componentStatus = APP_OK;
// 				return APP_OK;
// 			}
// 			storageCmd.setPosit += 400;
// 			return _UpdateOutput(static_cast<float_t>(storageCmd.setPosit));
// 		}

// 		case FSM_CTRL: {
// 			// 限制位置
// 			// storageCmd.setPosit = std::clamp(storageCmd.setPosit, static_cast<int32_t>(0), rangeLimit);
// 			// 更新输出
// 			return _UpdateOutput(static_cast<float_t>(storageCmd.setPosit));
// 		}

// 		default: {
// 			StopComponent();
// 			mtrOutputBuffer.fill(0);
// 			pidPosCtrl.ResetPidController();
// 			pidSpdCtrl.ResetPidController();
// 			componentStatus = APP_ERROR;
// 			return APP_ERROR;
// 		}
// 	}

// 	return APP_OK;
// }

// /**
//  * @brief 物理位置转换为电机位置
//  *
//  * @param phyPosit
//  * @return int32_t
//  */
// int32_t CModGimbal::CComStorage::PhyPositToMtrPosit(float_t phyPosit){
// 	const int32_t zeroOffset = 0;
// 	const float_t scale      = GIMBAL_STORAGE_MOTOR_RATIO;

// 	return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
// }

// /**
//  * @brief 电机位置转换为物理位置
//  *
//  * @param mtrPosit
//  * @return float_t
//  */
// float_t CModGimbal::CComStorage::MtrPositToPhyPosit(int32_t mtrPosit){
// 	const int32_t zeroOffset = 0;
// 	const float_t scale = GIMBAL_STORAGE_MOTOR_RATIO;

// 	return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
// }

// /**
//  * @brief 输出更新函数 (双电机同步)
//  *
//  * @param posit 目标位置
//  * @return EAppStatus
//  */
// EAppStatus CModGimbal::CComStorage::_UpdateOutput(float_t posit) {
// 	// 位置环 - 双电机目标位置（方向相反）
// 	DataBuffer<float_t> storagePos = {
// 		static_cast<float_t>(posit) * GIMBAL_STORAGE_MOTOR_DIR_F,
// 		static_cast<float_t>(posit) * GIMBAL_STORAGE_MOTOR_DIR_B,
// 	};

// 	DataBuffer<float_t> storagePosMeasure = {
// 		static_cast<float_t>(motor[F]->motorData[CDevMtr::DATA_POSIT]),
// 		static_cast<float_t>(motor[B]->motorData[CDevMtr::DATA_POSIT])
// 	};

// 	auto storageSpd = pidPosCtrl.UpdatePidController(storagePos, storagePosMeasure);

// 	// 速度环
// 	DataBuffer<float_t> storageSpdMeasure = {
// 		static_cast<float_t>(motor[F]->motorData[CDevMtr::DATA_SPEED]),
// 		static_cast<float_t>(motor[B]->motorData[CDevMtr::DATA_SPEED])
// 	};

// 	auto output = pidSpdCtrl.UpdatePidController(storageSpd, storageSpdMeasure);

// 	mtrOutputBuffer = {
// 		static_cast<int16_t>(output[F]),
// 		static_cast<int16_t>(output[B])
// 	};

// 	return APP_OK;
// }

// } // namespace my_engineer