// /**
//  * @file com_horizontal.cpp
//  * @author sllllr
//  * @brief 云台平动组件
//  * @version 1.0
//  * @date 2026-03-11
//  *
//  *
//  * @copyright Copyright (c) 2026
//  *
//  */

// #include "mod_gimbal.hpp"

// namespace my_engineer {

// /**
//  * @brief 初始化云台平动组件
//  *
//  * @param param
//  * @return EAppStatus
//  */
// EAppStatus CModGimbal::CComHorizontal::InitComponent(SModInitParam_Base &param){
// 	// 检查param是否正确
// 	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

// 	// 类型转换
// 	auto gimbalParam = static_cast<SModInitParam_Gimbal &>(param);

// 	// 保存电机指针
// 	motor = MotorIDMap.at(gimbalParam.horizontalMotorID);

// 	// 设置发送节点
// 	mtrCanTxNode = gimbalParam.horizontalMotorTxNode;

// 	// 初始化PID控制器
// 	gimbalParam.horizontalPosPidParam.threadNum = 1;
// 	pidPosCtrl.InitPID(&gimbalParam.horizontalPosPidParam);

// 	gimbalParam.horizontalSpdPidParam.threadNum = 1;
// 	pidSpdCtrl.InitPID(&gimbalParam.horizontalSpdPidParam);

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
// EAppStatus CModGimbal::CComHorizontal::UpdateComponent() {
// 	// 检查组件状态
// 	if (componentStatus == APP_RESET) return APP_ERROR;
	
//     if (motor == nullptr) {
//         return APP_ERROR;
//     }

// 	// 更新组件信息
// 	horizontalInfo.posit = (motor->motorData[CDevMtr::DATA_POSIT] * GIMBAL_HORIZONTAL_MOTOR_DIR);
// 	horizontalInfo.isPositArrived = (abs(horizontalCmd.setPosit - horizontalInfo.posit) < 8192 * 1);    ///< 暂且给1 后面得调整

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
// 			horizontalCmd.setPosit = static_cast<int32_t>(0);
// 			motor->motorData[CDevMtr::DATA_POSIT] = 0;
// 			mtrOutputBuffer.fill(0);
// 			pidPosCtrl.ResetPidController();
// 			pidSpdCtrl.ResetPidController();
// 			Component_FSMFlag_ = FSM_INIT;
// 			return APP_OK;
// 		}

// 		case FSM_INIT: {
// 			// 电机堵转，说明初始化完成
// 			if (motor->motorStatus == CDevMtr::EMotorStatus::STALL) {
// 				horizontalCmd = SHorizontalCmd();		// 堵转之后清零目标值
// 				// 补偿超出限位的值
// 				motor->motorData[CDevMtr::DATA_POSIT] = static_cast<int32_t>(0.1 * 8192 + rangeLimit) * GIMBAL_HORIZONTAL_MOTOR_DIR;
// 				pidPosCtrl.ResetPidController();
// 				pidSpdCtrl.ResetPidController();
// 				Component_FSMFlag_ = FSM_CTRL;
// 				componentStatus = APP_OK;
// 				return APP_OK;
// 			}
// 			horizontalCmd.setPosit += 400;
// 			return _UpdateOutput(static_cast<float_t>(horizontalCmd.setPosit));
// 		}

// 		case FSM_CTRL: {
// 			// 限制位置
// 			// horizontalCmd.setPosit = std::clamp(horizontalCmd.setPosit, static_cast<int32_t>(0), rangeLimit);
// 			// 更新输出
// 			return _UpdateOutput(static_cast<float_t>(horizontalCmd.setPosit));
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
// int32_t CModGimbal::CComHorizontal::PhyPositToMtrPosit(float_t phyPosit){
// 	const int32_t zeroOffset = 0;
// 	const float_t scale      = GIMBAL_HORIZONTAL_MOTOR_RATIO;

// 	return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
// }

// /**
//  * @brief 电机位置转换为物理位置
//  *
//  * @param mtrPosit
//  * @return float_t
//  */
// float_t CModGimbal::CComHorizontal::MtrPositToPhyPosit(int32_t mtrPosit){
// 	const int32_t zeroOffset = 0;
// 	const float_t scale = GIMBAL_HORIZONTAL_MOTOR_RATIO;

// 	return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
// }

// /**
//  * @brief 输出更新函数
//  *
//  * @param posit
//  * @return EAppStatus
//  */
// EAppStatus CModGimbal::CComHorizontal::_UpdateOutput(float_t posit) {
// 	// 位置环
// 	DataBuffer<float_t> horizontalPos = {
// 		static_cast<float_t>(posit) * GIMBAL_HORIZONTAL_MOTOR_DIR,
// 	};

// 	DataBuffer<float_t> horizontalPosMeasure = {
// 		static_cast<float_t>(motor->motorData[CDevMtr::DATA_POSIT])
// 	};

// 	auto horizontalSpd = pidPosCtrl.UpdatePidController(horizontalPos, horizontalPosMeasure);

// 	// 速度环
// 	DataBuffer<float_t> horizontalSpdMeasure = {
// 		static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED])
// 	};

// 	auto output = pidSpdCtrl.UpdatePidController(horizontalSpd, horizontalSpdMeasure);

// 	mtrOutputBuffer = {
// 		static_cast<int16_t>(output[0])
// 	};

// 	return APP_OK;
// }

// } // namespace my_engineer