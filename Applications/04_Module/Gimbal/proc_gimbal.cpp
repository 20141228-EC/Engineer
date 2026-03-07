/**
 * @file proc_gimbal.cpp
 * @author Ciallo～(∠·ω< )⌒☆
 * @brief 云台任务
 * @version 2.0
 * @date 2025-12-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "mod_gimbal.hpp"

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

	// 任务循环
	while (true) {

		// FSM
		switch (gimbal.Module_FSMFlag_) {

			case FSM_RESET: {

				gimbal.gimbalInfo.isModuleAvailable = false;
				gimbal.comYaw_.StopComponent();

				proc_waitMs(20);
				continue; // 跳过下面的代码，直接进入下一次循环
			}

			case FSM_INIT: {

				proc_waitMs(250); // 等待系统稳定

				// 启动升降组件
				gimbal.comYaw_.StartComponent();
				gimbal.gimbalCmd = SGimbalCmd();
				gimbal.gimbalCmd.set_encoder_yaw = 0;
				gimbal.gimbalCmd.set_posit_yaw = 0.0f;
				gimbal.gimbalInfo.isModuleAvailable = true;
				gimbal.Module_FSMFlag_ = FSM_CTRL;
				break;
			}

			case FSM_CTRL: {

				// 限制控制量
				gimbal.RestrictGimbalCommand_();

				// 将控制量转换为电机控制量 - 升降
				gimbal.comYaw_.yawCmd.setPosit = gimbal.gimbalCmd.set_posit_yaw;

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
