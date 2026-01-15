/******************************************************************************
 * @file         proc_controller.cpp
 * @author       Fish_Joe (2328339747@qq.com), Ciallo
 * @brief        控制器模块任务处理
 * @version      V1.1
 * @date         2025-04-01
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-15
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#include "mod_controller.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    创建控制器模块任务
 ******************************************************************************/
void CModController::StartControllerModuleTask(void *argument) {

	// 要求参数为CModController类的实例，如果传入为空则删除任务并返回
	if (argument == nullptr) proc_return();

	// 类型转换
	auto &controller = *static_cast<CModController *>(argument);

	// 任务循环
	while (true) {

		switch (controller.Module_FSMFlag_) {

			case FSM_RESET: {
				
				controller.ControllerInfo.isModuleAvailable = false;
				controller.comYaw_.StopComponent();
				controller.comPitch1_.StopComponent();
				controller.comPitch2_.StopComponent();
				controller.comRoll_.StopComponent();
				controller.comPitchEnd_.StopComponent();
				controller.comBuzzer_.StopComponent();

				proc_waitMs(20);
				continue; // 跳过下面的代码，直接进入下一次循环
			}

			case FSM_INIT: {

				proc_waitMs(250); // 等待系统稳定

				// controller.comRocker_.StartComponent();
				controller.comBuzzer_.StartComponent();
				controller.comPitch1_.StartComponent();
				controller.comPitch2_.StartComponent();
				proc_waitUntil(controller.comPitch1_.componentStatus == APP_OK
					&& controller.comPitch2_.componentStatus == APP_OK);
				controller.comPitch1_.pitch1Cmd.setParam[EMotorParam::POSIT] = 30.0f;
				controller.comPitch2_.pitch2Cmd.setParam[EMotorParam::POSIT] = 35.0f;
				proc_waitUntil(controller.comPitch1_.pitch1Info.isPositArrived
					&& controller.comPitch2_.pitch2Info.isPositArrived);

				controller.comRoll_.StartComponent();
				controller.comPitchEnd_.StartComponent();
				proc_waitUntil(controller.comRoll_.componentStatus == APP_OK
					&& controller.comPitchEnd_.componentStatus == APP_OK);
				
				controller.comYaw_.StartComponent();
				proc_waitUntil(controller.comYaw_.componentStatus == APP_OK);

				controller.comYaw_.yawCmd.isFree = true; ///< 允许自由控制
				controller.comPitch1_.pitch1Cmd.isFree = true; ///< 允许自由控制
				controller.comPitch2_.pitch2Cmd.isFree = true; ///< 允许自由控制
				controller.comRoll_.rollCmd.isFree = true; ///< 允许自由控制
				controller.comPitchEnd_.pitchEndCmd.isFree = true; ///< 允许自由控制

				controller.comBuzzer_.buzzerCmd.musicType = CDevBuzzer::MusicType::STARTUP;

				controller.ControllerCmd = SControllerCmd();
				controller.ControllerCmd.isFree = true;
				controller.ControllerInfo.isModuleAvailable = true;
				controller.Module_FSMFlag_ = FSM_CTRL;
				controller.moduleStatus = APP_OK;
				
				continue;
			}

			case FSM_CTRL: {

				// 限制控制量
				controller.RestrictControllerCommand_();

				static bool last_StartControl = false;

				// // 开始进行控制
				// if (controller.ControllerCmd.StartControl &&
				// 	!last_StartControl)
				// {
				// 	controller.ControllerInfo.isReturnSuccess = false;
				// 	controller.ControllerCmd.isFree = false;
				// }
				// last_StartControl = controller.ControllerCmd.StartControl;

				// 将控制量传递给组件
				controller.comYaw_.yawCmd.isFree = controller.ControllerCmd.isFree;
				controller.comPitch1_.pitch1Cmd.isFree = controller.ControllerCmd.isFree;
				controller.comPitch2_.pitch2Cmd.isFree = controller.ControllerCmd.isFree;
				controller.comRoll_.rollCmd.isFree = controller.ControllerCmd.isFree;
				controller.comPitchEnd_.pitchEndCmd.isFree = controller.ControllerCmd.isFree;

				
				if(!controller.ControllerCmd.isFree) {
					controller.comYaw_.yawCmd.setPosit = CModController::CComYaw::PhyPositToMtrPosit(controller.ControllerCmd.cmd_yaw);
					controller.comPitch1_.pitch1Cmd.setParam[EMotorParam::POSIT] =  controller.ControllerCmd.cmd_pitch1;
					controller.comPitch2_.pitch2Cmd.setParam[EMotorParam::POSIT] =  controller.ControllerCmd.cmd_pitch2;
					/*----------- Roll和PitchEnd改为MIT模式，直接传入角度值 -----------*/
					controller.comRoll_.rollCmd.setParam[EMotorParam::POSIT] = controller.ControllerCmd.cmd_roll;
					controller.comPitchEnd_.pitchEndCmd.setParam[EMotorParam::POSIT] = controller.ControllerCmd.cmd_pitch_end;
				}

				// 检查是否归位完成
				if (controller.comPitch1_.pitch1Info.isPositArrived &&
						controller.comPitch2_.pitch2Info.isPositArrived &&
						controller.comYaw_.yawInfo.isPositArrived &&
						controller.comRoll_.rollInfo.isPositArrived &&
						controller.comPitchEnd_.pitchEndInfo.isPositArrived){

						controller.ControllerInfo.isReturnSuccess = true;
						controller.ControllerCmd.isFree = true;
				}

				proc_waitMs(1);

				continue;
			}

			default: { controller.StopModule(); }
		}
	}

	// 任务退出
	controller.moduleTaskHandle = nullptr;
	proc_return();
}

} // namespace my_engineer
