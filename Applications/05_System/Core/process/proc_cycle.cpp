/******************************************************************************
 * @brief
 *
 * @file         proc_cycle.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2026-7-18
 *
 * @copyright    Copyright (c) 2026
 *
 ******************************************************************************/

#include "Core.hpp"
#include "utils.h"
#include <cmath>

namespace my_engineer {

/******************************************************************************
 * @brief    底盘往复旋转任务
 *
 * 运动过程：
 * 初始角度
 * -> 向一个方向旋转90°
 * -> 反方向旋转180°
 * -> 再反方向旋转180°
 * -> 持续往复
 ******************************************************************************/
void CSystemCore::StartCycleTask(void *arg)
{
    if (arg == nullptr) {
        proc_return();
    }

    // 获取 SystemCore 句柄
    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;

    core.pchassis_->chassisCmd.isAutoCtrl = true;
    core.pgimbal_->gimbalCmd.isAutoCtrl = true;
    core.parm_->armCmd.isAutoCtrl = true;

    /* Set Chassis */
    core.pchassis_->chassisCmd.L_length = CLIMBING_HIP_ANGLE;

    /* Set Arm */
    // core.parm_->armCmd.gripClose = true;
    // core.parm_->armCmd.set_angle_Yaw = CYCLE_YAW_ANGLE;
	// core.parm_->armCmd.set_angle_Pitch1 = CYCLE_PITCH1_ANGLE;
	// core.parm_->armCmd.set_angle_Pitch2 = CYCLE_PITCH2_ANGLE;
	// core.parm_->armCmd.set_angle_Roll = CYCLE_ROLL_ANGLE;
	// core.parm_->armCmd.set_angle_end_pitch = CYCLE_END_PITCH_ANGLE;
	// core.parm_->armCmd.set_angle_end_roll = CYCLE_END_ROLL_ANGLE;
    // core.parm_->armCmd.set_length_grip = CYCLE_GRIP_LENGTH;
	core.pgimbal_->gimbalCmd.set_visualyaw = CYCLE_GIMBAL_YAW_ANGLE;
    core.pgimbal_->gimbalCmd.set_pitch = CYCLE_GIMBAL_PITCH_ANGLE;



    const float_t currentYawAngle = core.pchassis_->filter->Imu_Ekf_Info.yaw;

    float_t rotateSign = -1.0f;
    float_t targetYawAngle = math::loopLimit(currentYawAngle + rotateSign * 90.0f, -180.0f, 180.0f);

    proc_waitMs(300);

    core.movemode_ = EMoveMode::CYCLE;  // 更新系统层标志位
	core.pchassis_->MovMode = CModChassis::EmovMode::CYCLE;     // 更新模块层标志位

    while (keyboard.key_Shift)      // 按住shift
    {
        const float_t currentYaw =
            core.pchassis_->filter->Imu_Ekf_Info.yaw;

        float_t yawError = math::loopLimit(targetYawAngle - currentYaw, -180.0f, 180.0f);

        if (std::fabs(yawError) > 170.0f)
        {
            yawError = rotateSign * std::fabs(yawError);
        }

        if (std::fabs(yawError) > 5.f)
        {
            core.pchassis_->chassisCmd.speed_W =
                math::sign(yawError) * -100.f;
        }
        else
        {
            core.pchassis_->chassisCmd.speed_W = 0.0f;

            // 每次到达目标后切换旋转方向
            rotateSign *= -1.0f;

            // 下一个目标相对于当前目标旋转180°
            targetYawAngle = math::loopLimit(targetYawAngle + rotateSign * 90.0f, -180.0f, 180.0f);
        }

        if(keyboard.key_B){
            core.pchassis_->chassisCmd.L_length += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 0.015f;
        }

        proc_waitMs(1);
    }

proc_exit:
    core.pchassis_->chassisCmd.speed_W = 0.0f;
    core.pchassis_->chassisCmd.isAutoCtrl = false;
    core.pgimbal_->gimbalCmd.isAutoCtrl = false;
    core.parm_->armCmd.isAutoCtrl = false;
    core.gimbal_auto_ctrl = false;
    core.autoCtrlTaskHandle_ = nullptr;
    core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

    proc_return();
}

} // namespace my_engineer