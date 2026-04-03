/**
 * @file proc_gimbal.cpp
 * @author ciallo
 * @version 2.0
 * @date 2026-03-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mod_gimbal.hpp"

namespace my_engineer {

/**
 * @brief 云台模块任务入口
 *
 * @param argument 指向 CModGimbal 实例的指针
 */
void CModGimbal::StartGimbalModuleTask(void *argument) {

	if (argument == nullptr) proc_return();

	auto &gimbal = *static_cast<CModGimbal *>(argument);

	while (true) {

		switch (gimbal.Module_FSMFlag_) {

			case FSM_RESET: {
				gimbal.gimbalInfo.isModuleAvailable = false;
				gimbal.comVisualyaw_.StopComponent();
				proc_waitMs(20);
				continue;
			}

			case FSM_INIT: {
				// 初始化: 等待电机上线后启动组件
				proc_waitMs(250); // 等待DM_MIT电机使能完成
				gimbal.comVisualyaw_.StartComponent();
				proc_waitUntil(gimbal.comVisualyaw_.componentStatus == APP_OK);

				// 设置初始目标角度
				gimbal.gimbalCmd = SGimbalCmd();
				gimbal.gimbalCmd.set_visualyaw = GIMBAL_VISUAL_MOTOR_INIT_ANGLE;
				gimbal.gimbalInfo.isModuleAvailable = true;
				gimbal.Module_FSMFlag_ = FSM_CTRL;
				gimbal.moduleStatus = APP_OK;
				break;
			}

			case FSM_CTRL: {
				// 写入组件目标角度
				gimbal.RestrictGimbalCommand_();

				gimbal.comVisualyaw_.VisuallyawCmd.setAngle =
					CComVisualyaw::PhyAngleToMtrAngle(gimbal.gimbalCmd.set_visualyaw);

				proc_waitMs(1);
				break;
			}

			default: { gimbal.StopModule(); }
		}
	}

	gimbal.moduleTaskHandle = nullptr;
	proc_return();

}

} // namespace my_engineer
