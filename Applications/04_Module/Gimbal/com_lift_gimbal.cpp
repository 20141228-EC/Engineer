/**
 * @file com_lift_gimbal.cpp
 * @author Ciallo～(∠·ω< )⌒☆
 * @brief 云台升降组件 (双M2006同步控制)
 * @version 2.0
 * @date 2025-12-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "mod_gimbal.hpp"

namespace my_engineer {

/**
 * @brief 初始化云台升降组件
 *
 * @param param
 * @return EAppStatus
 */
EAppStatus CModGimbal::CComLift::InitComponent(SModInitParam_Base &param){
	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto gimbalParam = static_cast<SModInitParam_Gimbal &>(param);

	// 保存双电机指针
	motor[L] = MotorIDMap.at(gimbalParam.liftMotorID_L);
	motor[R] = MotorIDMap.at(gimbalParam.liftMotorID_R);

	// 设置CAN发送节点
	mtrCanTxNode[L] = gimbalParam.liftMotorTxNode_L;
	mtrCanTxNode[R] = gimbalParam.liftMotorTxNode_R;

	// 初始化PID控制器 (threadNum = 2 用于双电机)
	gimbalParam.liftPosPidParam.threadNum = 2;
	pidPosCtrl.InitPID(&gimbalParam.liftPosPidParam);

	gimbalParam.liftSpdPidParam.threadNum = 2;
	pidSpdCtrl.InitPID(&gimbalParam.liftSpdPidParam);

	// 初始化电机数据输出缓冲区
	mtrOutputBuffer.fill(0);

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新组件
 *
 */
EAppStatus CModGimbal::CComLift::UpdateComponent() {
	// 检查组件状态
	if (componentStatus == APP_RESET) return APP_ERROR;

	// 更新组件信息 - 取两电机位置平均值
	liftInfo.posit = (motor[L]->motorData[CDevMtr::DATA_POSIT] * GIMBAL_LIFT_MOTOR_DIR_L
					+ motor[R]->motorData[CDevMtr::DATA_POSIT] * GIMBAL_LIFT_MOTOR_DIR_R) / 2;
	liftInfo.isPositArrived = (abs(liftCmd.setPosit - liftInfo.posit) < 8192 * 1);

	switch (Component_FSMFlag_){
		case FSM_RESET: {
			// 重置状态下，电机数据输出缓冲区始终为0
			mtrOutputBuffer.fill(0);
			pidPosCtrl.ResetPidController();
			pidSpdCtrl.ResetPidController();
			return APP_OK;
		}

		case FSM_PREINIT: {
			// 预初始化状态下，清零目标值和电机位置
			liftCmd.setPosit = static_cast<int32_t>(0);
			motor[L]->motorData[CDevMtr::DATA_POSIT] = 0;
			motor[R]->motorData[CDevMtr::DATA_POSIT] = 0;
			mtrOutputBuffer.fill(0);
			pidPosCtrl.ResetPidController();
			pidSpdCtrl.ResetPidController();
			Component_FSMFlag_ = FSM_INIT;
			return APP_OK;
		}

		case FSM_INIT: {
			// 任一电机堵转，说明初始化完成
			if (motor[L]->motorStatus == CDevMtr::EMotorStatus::STALL ||
				motor[R]->motorStatus == CDevMtr::EMotorStatus::STALL) {
				liftCmd = SLiftCmd();	// 堵转之后清零目标值
				// 补偿超出限位的值
				motor[L]->motorData[CDevMtr::DATA_POSIT] = static_cast<int32_t>(0.1 * 8192 + rangeLimit) * GIMBAL_LIFT_MOTOR_DIR_L;
				motor[R]->motorData[CDevMtr::DATA_POSIT] = static_cast<int32_t>(0.1 * 8192 + rangeLimit) * GIMBAL_LIFT_MOTOR_DIR_R;
				pidPosCtrl.ResetPidController();
				pidSpdCtrl.ResetPidController();
				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;
				return APP_OK;
			}
			liftCmd.setPosit += 400;
			return _UpdateOutput(static_cast<float_t>(liftCmd.setPosit));
		}

		case FSM_CTRL: {
			// 限制位置
			liftCmd.setPosit = std::clamp(liftCmd.setPosit, static_cast<int32_t>(0), rangeLimit);
			// 更新输出
			return _UpdateOutput(static_cast<float_t>(liftCmd.setPosit));
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
int32_t CModGimbal::CComLift::PhyPositToMtrPosit(float_t phyPosit){
	const int32_t zeroOffset = 0;
	const float_t scale      = GIMBAL_LIFT_MOTOR_RATIO;

	return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}

/**
 * @brief 电机位置转换为物理位置
 *
 * @param mtrPosit
 * @return float_t
 */
float_t CModGimbal::CComLift::MtrPositToPhyPosit(int32_t mtrPosit){
	const int32_t zeroOffset = 0;
	const float_t scale = GIMBAL_LIFT_MOTOR_RATIO;

	return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}

/**
 * @brief 输出更新函数 (双电机同步)
 *
 * @param posit 目标位置
 * @return EAppStatus
 */
EAppStatus CModGimbal::CComLift::_UpdateOutput(float_t posit) {
	// 位置环 - 双电机目标位置（方向相反）
	DataBuffer<float_t> liftPos = {
		static_cast<float_t>(posit) * GIMBAL_LIFT_MOTOR_DIR_L,
		static_cast<float_t>(posit) * GIMBAL_LIFT_MOTOR_DIR_R,
	};

	DataBuffer<float_t> liftPosMeasure = {
		static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_POSIT]),
		static_cast<float_t>(motor[R]->motorData[CDevMtr::DATA_POSIT])
	};

	auto liftSpd = pidPosCtrl.UpdatePidController(liftPos, liftPosMeasure);

	// 速度环
	DataBuffer<float_t> liftSpdMeasure = {
		static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_SPEED]),
		static_cast<float_t>(motor[R]->motorData[CDevMtr::DATA_SPEED])
	};

	auto output = pidSpdCtrl.UpdatePidController(liftSpd, liftSpdMeasure);

	mtrOutputBuffer = {
		static_cast<int16_t>(output[L]),
		static_cast<int16_t>(output[R])
	};

	return APP_OK;
}

} // namespace my_engineer
