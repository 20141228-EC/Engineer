/******************************************************************************
 * @brief    一键取六矿任务
 *
 * @file     proc_one_key_ore.cpp
 * @author   ciallo
 * @version  V2.0
 * @date     2026-5-28
 * @note:    断点续取：
 *           1. 如果在抓取阶段退出，则下次从该颗矿的抓取重新开始
 *           2. 如果在存矿阶段退出，则下次跳过抓取，直接从存矿开始
 *           3. 如果完整跑则完重置断点
 *
 * @copyright Copyright (c) 2026
 *
 ******************************************************************************/
#include "proc_common.hpp"
#include "proc_traj_data.hpp"

namespace my_engineer {

void CSystemCore::StartOneKeyOreTask(void *arg) {
    if (arg == nullptr) proc_return();

    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;
    auto &arm  = *core.parm_;
    auto &gimbal = *core.pgimbal_;

    CAlgoTrajPlayback player;

    /*---------------------等待确认-------------------*/
    arm.armCmd.isAutoCtrl = true;
    while (true) {
        if (keyboard.mouse_L) break;
        if (keyboard.key_Ctrl && keyboard.key_Z) goto proc_exit;
        proc_waitMs(5);
    }

    /*-------------------记忆起始位置------------------------*/
    for (int step = core.oreTaskStep_; step < OreStepCount; step++) {
        const auto &cfg = OreStepConfig[step];
        if (!cfg.enabled) continue;

        core.oreTaskStep_ = step;

        gimbal.ChooseStoreOre(static_cast<CModGimbal::EStorageSlot>(cfg.slot));

        /*--------------------取---------------------*/
        if (!core.oreGetDone_) {
            const auto &clip = cfg.getClip;
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

            core.oreGetDone_ = true;
        }

        /*--------------------存---------------------*/
        {
            const auto &clip = cfg.storeClip;
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

        core.oreGetDone_ = false;  // 该颗矿完成，重置

        if (keyboard.key_Ctrl && keyboard.key_Z) goto proc_exit;
    }

    // 完整跑完，重置
    core.oreTaskStep_ = 0;
    core.oreGetDone_ = false;

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
