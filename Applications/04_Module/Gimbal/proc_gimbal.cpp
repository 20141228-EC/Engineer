/**
 * @file proc_gimbal.cpp
 * @author Ciallo～(∠·ω< )⌒☆
 * @brief 云台任务
 * @version 2.1
 * @date 2025-01-19
 *
 * @note 支持舵机/电机切换，通过 USE_PITCH_SERVO 宏控制
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "mod_gimbal.hpp"
#include "RTT_DEBUG.h"

// RTT 调试开关 (1=开启, 0=关闭)
#define GIMBAL_RTT_DEBUG_ENABLE 0
// RTT 打印间隔 (ms)
#define GIMBAL_RTT_PRINT_INTERVAL 100

namespace my_engineer {

/**
 * @brief 云台模块任务
 *
 * @param argument
 */
void CModGimbal::StartGimbalModuleTask(void *argument) {

	// 要求参数为CModGimbal类的实例，如果传入为空则删除任务并返回
	if (argument == nullptr) proc_return();

	// 类型转换
	auto &gimbal = *static_cast<CModGimbal *>(argument);

#if GIMBAL_RTT_DEBUG_ENABLE
	// RTT 打印计数器
	uint32_t rttPrintCounter = 0;
#endif

	// 任务循环
	while (true) {

		// FSM
		switch (gimbal.Module_FSMFlag_) {

			case FSM_RESET: {

				gimbal.gimbalInfo.isModuleAvailable = false;
				gimbal.comLift_.StopComponent();
#ifdef USE_PITCH_SERVO
				gimbal.comPitchServo_.StopComponent();
#else
				gimbal.comPitch_.StopComponent();
#endif

				proc_waitMs(20);
				continue; // 跳过下面的代码，直接进入下一次循环
			}

			case FSM_INIT: {

				proc_waitMs(250); // 等待系统稳定

				// 启动升降组件
				gimbal.comLift_.StartComponent();
				proc_waitUntil(gimbal.comLift_.componentStatus == APP_OK);

#ifdef USE_PITCH_SERVO
				// 启动俯仰组件 (舵机版本)
				gimbal.comPitchServo_.StartComponent();
				proc_waitUntil(gimbal.comPitchServo_.componentStatus == APP_OK);
#else
				// 启动俯仰组件 (电机版本)
				gimbal.comPitch_.StartComponent();
				proc_waitUntil(gimbal.comPitch_.componentStatus == APP_OK);
#endif

				gimbal.gimbalCmd = SGimbalCmd();
				gimbal.gimbalCmd.set_posit_lift = GIMBAL_LIFT_PHYSICAL_RANGE;
				gimbal.gimbalCmd.set_posit_pitch = 0.0f;
				gimbal.gimbalInfo.isModuleAvailable = true;
				gimbal.Module_FSMFlag_ = FSM_CTRL;

#if GIMBAL_RTT_DEBUG_ENABLE
#ifdef USE_PITCH_SERVO
				RTT_LOG_INFO("[云台] 初始化完成(舵机模式)，进入控制模式");
#else
				RTT_LOG_INFO("[云台] 初始化完成(电机模式)，进入控制模式");
#endif
#endif

				break;
			}

			case FSM_CTRL: {

				// 限制控制量
				gimbal.RestrictGimbalCommand_();

				// 将控制量转换为电机控制量 - 升降
				gimbal.comLift_.liftCmd.setPosit =
					CComLift::PhyPositToMtrPosit(gimbal.gimbalCmd.set_posit_lift);

#ifdef USE_PITCH_SERVO
				// 将控制量转换为舵机控制量 - 俯仰 (舵机版本)
				gimbal.comPitchServo_.pitchCmd.setPosit =
					CComPitchServo::PhyPositToSetPosit(gimbal.gimbalCmd.set_posit_pitch);
#else
				// 将控制量转换为电机控制量 - 俯仰 (电机版本)
				gimbal.comPitch_.pitchCmd.setPosit =
					CComPitch::PhyPositToMtrPosit(gimbal.gimbalCmd.set_posit_pitch);
#endif

#if GIMBAL_RTT_DEBUG_ENABLE
				// 定时打印调试信息
				if (++rttPrintCounter >= GIMBAL_RTT_PRINT_INTERVAL) {
					rttPrintCounter = 0;

					// ------ 位置数据 -------
					RTT_LOG_INFO("[云台] ------ 位置数据------");
					RTT_LOG_INFO("[升降] 设定:%.2fmm 实际:%.2fmm | 电机设定:%d 电机实际:%d",
						gimbal.gimbalCmd.set_posit_lift,
						gimbal.gimbalInfo.posit_lift,
						gimbal.comLift_.liftCmd.setPosit,
						gimbal.comLift_.liftInfo.posit);

#ifdef USE_PITCH_SERVO
					RTT_LOG_INFO("[俯仰-舵机] 设定:%.2fdeg 实际:%.2fdeg",
						gimbal.gimbalCmd.set_posit_pitch,
						gimbal.gimbalInfo.posit_pitch);
#else
					RTT_LOG_INFO("[俯仰-电机] 设定:%.2fdeg 实际:%.2fdeg | 电机设定:%d 电机实际:%d",
						gimbal.gimbalCmd.set_posit_pitch,
						gimbal.gimbalInfo.posit_pitch,
						gimbal.comPitch_.pitchCmd.setPosit,
						gimbal.comPitch_.pitchInfo.posit);
#endif

					// ------- PID 输出数据 ------
					RTT_LOG_INFO("[云台] ----- PID输出 ------");
					RTT_LOG_INFO("[升降] 左电机输出:%d 右电机输出:%d",
						gimbal.comLift_.mtrOutputBuffer[0],
						gimbal.comLift_.mtrOutputBuffer[1]);

#ifndef USE_PITCH_SERVO
					RTT_LOG_INFO("[俯仰] 电机输出:%d",
						gimbal.comPitch_.mtrOutputBuffer[0]);
#endif

					// ------- 状态标志 ---------
					RTT_LOG_INFO("[云台] 到位状态: 升降=%d 俯仰=%d",
						gimbal.gimbalInfo.isPositArrived_Lift,
						gimbal.gimbalInfo.isPositArrived_Pitch);
				}
#endif

				proc_waitMs(1);
				break;
			}

			default: { gimbal.StopModule(); }
		}
	}

	// 任务退出
	gimbal.moduleTaskHandle = nullptr;
	proc_return();

}

} // namespace my_engineer
