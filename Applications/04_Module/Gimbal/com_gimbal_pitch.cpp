/******************************************************************************
 * @brief    云台Pitch轴组件
 *
 * @file     com_gimbal_pitch.cpp
 * @author   ciallo
 * @version  V1.0
 * @date     2026-06-08
 *
 *
 * @copyright Copyright (c) 2026
 *
 ******************************************************************************/

#include "mod_gimbal.hpp"

namespace my_engineer {

/**
 * @brief 初始化云台Pitch组件
 */
EAppStatus CModGimbal::CComGimbalPitch::InitComponent(SModInitParam_Base &param) {
    if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    auto gimbalParam = static_cast<SModInitParam_Gimbal &>(param);

    motor = MotorIDMap.at(gimbalParam.pitchMotorID);
    mtrCanTxNode = gimbalParam.pitchMotorTxNode;

    // 初始化位置PID
    gimbalParam.PitchPosPidParam.threadNum = 1;
    pidPosCtrl.InitPID(&gimbalParam.PitchPosPidParam);

    // 初始化速度PID
    gimbalParam.PitchSpdPidParam.threadNum = 1;
    pidSpdCtrl.InitPID(&gimbalParam.PitchSpdPidParam);

    mtrOutputBuffer = 0;

    Component_FSMFlag_ = FSM_RESET;
    componentStatus = APP_OK;

    return APP_OK;
}



/**
 * @brief 更新云台Pitch组件
 */
EAppStatus CModGimbal::CComGimbalPitch::UpdateComponent() {
    if (componentStatus == APP_RESET) {
        mtrOutputBuffer = 0;
        return APP_ERROR;
    }

    // 更新当前位置 (编码器值)
    pitchInfo.posit = motor->motorData[CDevMtr::DATA_POSIT] * GIMBAL_PITCH_MOTOR_DIR;

    // 到位判断
    pitchInfo.isPositArrived = (abs(pitchInfo.posit - pitchCmd.setPosit) < 8192 );

    switch (Component_FSMFlag_) {

        case FSM_RESET: {
            mtrOutputBuffer = 0;
            pitchCmd = SPitchCmd{};
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            return APP_OK;
        }

        case FSM_PREINIT: {
            mtrOutputBuffer = 0;
            motor->motorData[CDevMtr::DATA_POSIT] = 0;
            pitchCmd = SPitchCmd{};
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT: {
            // 等待到达初始位置附近
            if ((motor->motorStatus == CDevMtr::EMotorStatus::STALL)) {
                motor->motorData[CDevMtr::DATA_POSIT] = GIMBAL_PITCH_MEC;
                pitchCmd.setPosit = PhyPositToMtrPosit(GIMBAL_PITCH_INIT_ANGLE);
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                Component_FSMFlag_ = FSM_CTRL;
                componentStatus = APP_OK;
                return APP_OK;
            }
            pitchCmd.setPosit -= 500;
            return _UpdateOutput(static_cast<float_t>(pitchCmd.setPosit));
        }
        case FSM_CTRL: {

            return _UpdateOutput(static_cast<float_t>(pitchCmd.setPosit));
        }

        default: {
            StopComponent();
            mtrOutputBuffer = 0;
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            componentStatus = APP_ERROR;
            return APP_ERROR;
        }
    }
}
/**
 * @brief 物理位置转换为电机位置
 *
 * @param phyPosit
 * @return int32_t
 */
int32_t CModGimbal::CComGimbalPitch::PhyPositToMtrPosit(float_t phyPosit){
	const int32_t zeroOffset = 0;
	const float_t scale      = GIMBAL_PITCH_MOTOR_RATIO;

	return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}

/**
 * @brief 电机位置转换为物理位置
 *
 * @param mtrPosit
 * @return float_t
 */
float_t CModGimbal::CComGimbalPitch::MtrPositToPhyPosit(int32_t mtrPosit){
	const int32_t zeroOffset = 0;
	const float_t scale = GIMBAL_PITCH_MOTOR_RATIO;

	return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}

/**
 * @brief 输出更新函数
 *
 * @param posit
 * @return EAppStatus
 */
EAppStatus CModGimbal::CComGimbalPitch::_UpdateOutput(float_t posit) {
	// 位置环
	DataBuffer<float_t> pitchPos = {
		static_cast<float_t>(posit) * GIMBAL_PITCH_MOTOR_DIR,
	};

	DataBuffer<float_t> pitchPosMeasure = {
		static_cast<float_t>(motor->motorData[CDevMtr::DATA_POSIT])
	};

	auto pitchSpd = pidPosCtrl.UpdatePidController(pitchPos, pitchPosMeasure);

	// 速度环
	DataBuffer<float_t> pitchSpdMeasure = {
		static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED])
	};

	auto output = pidSpdCtrl.UpdatePidController(pitchSpd, pitchSpdMeasure);

	mtrOutputBuffer = {
		static_cast<int16_t>(output[0] + GIMBAL_PITCH_GRAV_FF)
	};

	return APP_OK;
}
} // namespace my_engineer
