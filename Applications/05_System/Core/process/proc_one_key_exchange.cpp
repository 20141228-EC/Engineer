/******************************************************************************
 * @brief    一键兑六矿任务
 *
 * @file     proc_one_key_exchange.cpp
 * @author   ciallo
 * @version  V1.0
 * @date     2026-5-28
 * @note     流程：每次自动取下一颗，等操作手手动兑换后点鼠标确认，再取下一颗
 *           1.存矿电机预转位
 *           2.自动取下
 *           3.操作手用控制器手动兑换
 *           4.点鼠标确认
 *           5.下一颗
 *
 * @copyright Copyright (c) 2026
 *
 ******************************************************************************/
#include "proc_common.hpp"
#include "proc_traj_data.hpp"

namespace my_engineer {

void CSystemCore::StartOneKeyExchangeTask(void *arg) {
    if (arg == nullptr) proc_return();

    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;
    auto &arm  = *core.parm_;
    auto &gimbal = *core.pgimbal_;

    CAlgoTrajPlayback player;

    /*----  等待用户确认启动 ----*/
    arm.armCmd.isAutoCtrl = true;
    while (true) {
        if (keyboard.mouse_L) break;
        if (keyboard.key_Ctrl && keyboard.key_Z) goto proc_exit;
        proc_waitMs(5);
    }

    /*---------------------------开始循环-----------------------------*/
    for (int step = core.oreTaskStep_; step < OreStepCount; step++) {
        const auto &cfg = OreStepConfig[step];
        if (!cfg.enabled) continue;

        // 记录断点
        core.oreTaskStep_ = step;

        gimbal.ChooseStoreOre(static_cast<CModGimbal::EStorageSlot>(cfg.slot));// 存矿电机先转到位

        {
            const auto &clip = (step < 3) ? ExchangeClipL : ExchangeClipR;
            float_t lastTarget[J::COUNT];

            if (!PlayFrameSegment(arm, clip.frame, 0, player, true, 0.0f, nullptr))
                goto proc_exit;
            Extrarow(clip.frame, 0, lastTarget);
            proc_waitMs(50);

            for (int seg = 1; seg < clip.frameCount; seg++) {
                ReadArmjoint(arm, lastTarget);
                if (!PlayFrameSegment(arm, clip.frame, seg, player, true, 0.0f, lastTarget))
                    goto proc_exit;
                Extrarow(clip.frame, seg, lastTarget);
            }
        }

        /*切换到自定义控制器模式，操作手手动兑换*/
        arm.armCmd.isAutoCtrl = false;
        arm.armCmd.set_angle_Yaw       = arm.armInfo.angle_Yaw;
        arm.armCmd.set_angle_Pitch1    = arm.armInfo.angle_Pitch1;
        arm.armCmd.set_angle_Pitch2    = arm.armInfo.angle_Pitch2;
        arm.armCmd.set_angle_Roll      = arm.armInfo.angle_Roll;
        arm.armCmd.set_angle_end_pitch = arm.armInfo.angle_end_pitch;
        arm.armCmd.set_angle_end_roll  = arm.armInfo.angle_end_roll;
        if (SysControllerLink.IsControllerOnline()) {
            SysControllerLink.robotInfo.controlled_by_controller = true;
            core.use_Controller_ = true;
            arm.armCmd.isCustomCtrl = true;
        }

        while (true) {
            if (keyboard.mouse_L) break;      // 操作手兑换完毕，继续取
            if (keyboard.key_Ctrl && keyboard.key_Z) goto proc_exit;
            proc_waitMs(5);
        }

        // 收回控制权，回到自动模式
        arm.armCmd.isCustomCtrl = false;
        core.use_Controller_ = false;
        SysControllerLink.robotInfo.controlled_by_controller = false;
        arm.armCmd.isAutoCtrl = true;
    }

    // 完整跑完，重置断点
    core.oreTaskStep_ = 0;

proc_exit:
    core.parm_->armCmd.isAutoCtrl = false;
    core.autoCtrlTaskHandle_ = nullptr;
    core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

    arm.armCmd.set_angle_Yaw       = arm.armInfo.angle_Yaw;
    arm.armCmd.set_angle_Pitch1    = arm.armInfo.angle_Pitch1;
    arm.armCmd.set_angle_Pitch2    = arm.armInfo.angle_Pitch2;
    arm.armCmd.set_angle_Roll      = arm.armInfo.angle_Roll;
    arm.armCmd.set_angle_end_pitch = arm.armInfo.angle_end_pitch;
    arm.armCmd.set_angle_end_roll  = arm.armInfo.angle_end_roll;

    if (SysControllerLink.IsControllerOnline()) {
        SysControllerLink.robotInfo.controlled_by_controller = true;
        core.use_Controller_ = true;
        arm.armCmd.isCustomCtrl = true;
    } else {
        SysControllerLink.robotInfo.controlled_by_controller = false;
        core.use_Controller_ = false;
        arm.armCmd.isCustomCtrl = false;
    }
    core.armmode_ = EArmMode::NORMAL;
    proc_return();
}

} // namespace my_engineer
