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
 * 进入任务时记录当前角度为基准角
 * -> 先转到基准角逆时针90°位置
 * -> 再转到基准角顺时针90°位置
 * -> 在两个端点之间持续往复，总范围180°
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

    // 当前yaw轴为基准
    const float_t baseYawAngle = core.pchassis_->filter->Imu_Ekf_Info.yaw;

    // 逆时针目标
    const float_t counterClockwiseTarget = math::loopLimit(baseYawAngle - 90.0f, -180.0f, 180.0f);

    // 顺时针目标
    const float_t clockwiseTarget = math::loopLimit(baseYawAngle + 90.0f, -180.0f, 180.0f);

    // false时目标为逆时针 true时目标为顺时针
    bool movingToClockwiseTarget = false;

    // 这里偷懒了 没做半圈处理 直接给个方向变量确保不转错方向
    float_t rotateSign = -1.0f;

    float_t targetYawAngle = counterClockwiseTarget;    // 先逆时针

    proc_waitMs(300);

    core.movemode_ = EMoveMode::CYCLE;  // 更新系统层标志位
    core.pchassis_->MovMode = CModChassis::EmovMode::CYCLE; // 更新模块层标志位

    // 按住Shift
    while (keyboard.key_Shift)
    {
        const float_t currentYaw = core.pchassis_->filter->Imu_Ekf_Info.yaw;

        float_t yawError = math::loopLimit(targetYawAngle - currentYaw, -180.0f, 180.0f);

        if (std::fabs(yawError) > 170.0f)
        {
            yawError = rotateSign * std::fabs(yawError);
        }

        if (std::fabs(yawError) > 5.0f)
        {
            core.pchassis_->chassisCmd.speed_W = math::sign(yawError) * -100.0f;
        }
        else
        {
            // 到达当前目标点
            core.pchassis_->chassisCmd.speed_W = 0.0f;

            if (movingToClockwiseTarget)
            {
                // 目标切换为逆时针端点
                movingToClockwiseTarget = false;
                rotateSign = -1.0f;
                targetYawAngle = counterClockwiseTarget;
            }
            else
            {
                // 目标切换为顺时针端点
                movingToClockwiseTarget = true;
                rotateSign = 1.0f;
                targetYawAngle = clockwiseTarget;
            }
        }

        // 允许抬腿
        if (keyboard.key_B)
        {
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