/******************************************************************************
 * @brief        
 * 
 * @file         proc_traj_grab.cpp
 * @author       ciallo
 * @version      V2.0
 * @date         2026-4-18
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/

#include "proc_common.hpp"
#include "System.hpp"
#include "traj_data.hpp"

namespace my_engineer {

    /******************************************************************************
    * @brief    存能量单元任务
    ******************************************************************************/
    void CSystemCore::StartStoreTask(void *arg) {
        if (arg == nullptr) proc_return();

        auto &core = *reinterpret_cast<CSystemCore *>(arg);
        CStoreOreTaskRunner runner(core);

        // ---- 状态初始化 ----
        core.storeEndRollPose_ = EStoreEndRollPose::DOWN;//默认是向下的
        runner.endRollOffset_ = STORE_ROLL_DOWN_OFFSET;

        // 防止上次任务残留的 armCmd 值导致机械臂跳动 / 起点漂移
        runner.arm_.armCmd.set_angle_Yaw       = runner.arm_.armInfo.angle_Yaw;
        runner.arm_.armCmd.set_angle_Pitch1    = runner.arm_.armInfo.angle_Pitch1;
        runner.arm_.armCmd.set_angle_Pitch2    = runner.arm_.armInfo.angle_Pitch2;
        runner.arm_.armCmd.set_angle_Pitch3    = runner.arm_.armInfo.angle_Pitch3;
        runner.arm_.armCmd.set_angle_Roll      = runner.arm_.armInfo.angle_Roll;
        runner.arm_.armCmd.set_angle_end_pitch = runner.arm_.armInfo.angle_end_pitch;
        runner.arm_.armCmd.set_angle_end_roll  = runner.arm_.armInfo.angle_end_roll;

        // // 等待所有按键释放，防止残留状态直接进入上一次的模式
        // while (runner.keyboard_.key_Ctrl || runner.keyboard_.mouse_L || runner.keyboard_.mouse_R) {
        //     proc_waitMs(5);
        // }

        // 选择存矿/兑矿轨迹（控制器功能按键 / 键盘手动选择）
        ETrajID trajId;
        // if (core.exchange_side_ == CSystemCore::EExchangeSide::AUTO) {
        //     // // 控制器自动兑矿模式
        //     // trajId = TRAJ_AUTO;
        //     // core.armmode_ = EArmMode::AUTO;
        //     // core.exchange_side_ = CSystemCore::EExchangeSide::NONE;
        // } else {

        // }
        while (true) {
                if (runner.keyboard_.mouse_L) {
                    trajId = TRAJ_STORE_L;
                    core.armmode_ = EArmMode::STORE_L_ORE;
                    break;
                }
                if (runner.keyboard_.mouse_R) {
                    trajId = TRAJ_STORE_R;
                    core.armmode_ = EArmMode::STORE_R_ORE;
                    break;
                }
                proc_waitMs(5);
            }
        if (core.pgimbal_) core.pgimbal_->gimbalCmd.set_visualyaw = EXCHANGE_ORE_GIMBLE_YAW_ANGLE;

        runner.arm_.armCmd.isAutoCtrl = true; ///< 阻止外部ControlFromKeyboard_干扰，手动/自动都需要

        // 重新标定末端
        runner.arm_.armCmd.resetEndAll = true;
        while (runner.arm_.comEnd_.initState_ != CModArm::CComEnd::EEndInitState::DONE) {
            proc_waitMs(1);
        }

        // ---- 主流程 ----
        if (trajId == TRAJ_AUTO) {
            if (!runner.RunAutoOreTask())
                goto proc_exit;
        } else {
            if (!runner.RunManualStoreTask(trajId))
                goto proc_exit;
        }

proc_exit:
        runner.arm_.armCmd.isAutoCtrl = false;
        core.autoCtrlTaskHandle_ = nullptr;
        core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

        //退出时把 armCmd 同步到当前实际位姿
        runner.arm_.armCmd.set_angle_Yaw       = runner.arm_.armInfo.angle_Yaw;
        runner.arm_.armCmd.set_angle_Pitch1    = runner.arm_.armInfo.angle_Pitch1;
        runner.arm_.armCmd.set_angle_Pitch2    = runner.arm_.armInfo.angle_Pitch2;
        runner.arm_.armCmd.set_angle_Pitch3    = runner.arm_.armInfo.angle_Pitch3;
        runner.arm_.armCmd.set_angle_Roll      = runner.arm_.armInfo.angle_Roll;
        runner.arm_.armCmd.set_angle_end_pitch = runner.arm_.armInfo.angle_end_pitch;
        runner.arm_.armCmd.set_angle_end_roll  = runner.arm_.armInfo.angle_end_roll;

        // 任务结束默认进入自定义控制器模式
        if (SysControllerLink.IsControllerOnline()) {
            SysControllerLink.robotInfo.controlled_by_controller = true;
            core.use_Controller_ = true;
            runner.arm_.armCmd.isCustomCtrl = true;     ///< 同步标志位，避免下个周期的窗口期行为异常
        } else {
            SysControllerLink.robotInfo.controlled_by_controller = false;
            core.use_Controller_ = false;
            runner.arm_.armCmd.isCustomCtrl = false;
        }
        core.armmode_ = EArmMode::NORMAL;   //没有任务的状态
        core.storeEndRollPose_ = EStoreEndRollPose::DOWN;

        proc_return();
    }

    // 纯手动取矿
bool CStoreOreTaskRunner::RunManualStoreTask(ETrajID trajId) {
    auto &Traj = (trajId == TRAJ_STORE_R) ? StoresingleClip_R : StoresingleClip_L;

    // Roll 手动标定：R 键切换上下，Ctrl 确认
    while (!keyboard_.key_Ctrl) {
        if (edge_.key_R == CSystemRemote::ERemoteEdge::Rising) {
            if (core_.storeEndRollPose_ == CSystemCore::EStoreEndRollPose::DOWN) {
                core_.storeEndRollPose_ = CSystemCore::EStoreEndRollPose::UP;
                endRollOffset_ = STORE_ROLL_UP_OFFSET;
            } else {
                core_.storeEndRollPose_ = CSystemCore::EStoreEndRollPose::DOWN;
                endRollOffset_ = STORE_ROLL_DOWN_OFFSET;
            }
        }
        proc_waitMs(1);
    }

    // 对齐末端 roll
    if (!AlignEndRollToStore(Traj, endRollOffset_)) return false;

    // 逐段播放：seg 0 等位 + 逐帧 + lastTarget 链式
    return PlayTrajRows(arm_, Traj.frame, Traj.frameCount, endRollOffset_);
}

// 对齐末端的参数
bool CStoreOreTaskRunner::AlignEndRollToStore(const TrajClip &clip, float_t rollOff) {
    float_t firstFrameTarget[J::COUNT];
    Extrarow(clip.frame, 0, firstFrameTarget);

    float_t preAlignTarget[J::COUNT];
    ReadArmjoint(arm_, preAlignTarget);                              // 当前位置作为起点
    preAlignTarget[J::J_ENDR] = firstFrameTarget[J::J_ENDR] + rollOff; // 仅修改 end_roll

    const bool gripNow = (arm_.armInfo.gripState == CModArm::SArmInfo::EGripState::HOLD);
    // 提高末端roll转速猛转、注意限位块不要撞坏了
    SPlayJointTargetOptions opt;
    opt.speedScale = 10.0f;
    opt.gripDuringMotion = gripNow;
    opt.gripAfter = gripNow;
    opt.startOverride = nullptr;
    if (!PlayJointTarget(arm_, preAlignTarget, opt)) {
        return false;
    }
    return true;
}

// 取矿自动任务
bool CStoreOreTaskRunner::RunAutoOreTask() {
    // 选矿阶段：Q减 E加 R清零 Ctrl确认
    // 等按键释放
    while (keyboard_.key_Ctrl || keyboard_.key_Q || keyboard_.key_E || keyboard_.key_R) {
        proc_waitMs(5);
    }
    while (true) {
        if (edge_.key_Q == CSystemRemote::ERemoteEdge::Rising) {
            core_.oreTaskStep_ = (core_.oreTaskStep_ - 1 + OreStepCount) % OreStepCount;
        }
        if (edge_.key_E == CSystemRemote::ERemoteEdge::Rising) {
            core_.oreTaskStep_ = (core_.oreTaskStep_ + 1) % OreStepCount;
        }
        if (edge_.key_R == CSystemRemote::ERemoteEdge::Rising) {
            core_.oreTaskStep_ = 0;
        }
        if (keyboard_.key_Ctrl) break;
        proc_waitMs(5);
    }

    for (int step = core_.oreTaskStep_; step < OreStepCount; step++) {
        const auto &oreStep = OreStepConfig[step];
        const bool needStore = (step % 3 != 2);      // 第3、6矿不存，留在夹爪上等兑换
        const float_t rollOff = (step >= 2 && step <= 4) ? STORE_ROLL_UP_OFFSET : STORE_ROLL_DOWN_OFFSET;//在2、3、4步的矿石需要翻转

        /*------------------取矿---------------------*/
        if (!PlayGetClip(oreStep.getClip, rollOff)) return false;

        /*------------------存矿---------------------*/
        if (needStore) {
            if (!AlignEndRollToStore(oreStep.storeClip, rollOff)) return false;
            if (!PlayTrajRows(arm_, oreStep.storeClip.frame, oreStep.storeClip.frameCount, rollOff)) return false;
        }

        // 每成功完成一次存取矿就记录进度
        core_.oreTaskStep_ = step + 1;

        // 第3矿 / 第6矿取完不存，退出等待兑换
        if (!needStore) {
            core_.oreGetDone_ = (step >= OreStepCount - 1);
            return true;
        }
    }

    // 全部 6 矿完成
    core_.oreGetDone_ = true;
    core_.oreTaskStep_ = 0;
    return true;
}

bool CStoreOreTaskRunner::PlayGetClip(const TrajClip &clip, float_t rollOff) {
    float_t lastTarget[J::COUNT];

    if (std::fabs(rollOff) > 1e-3f) {
        if (!AlignEndRollToStore(clip, rollOff)) return false;
    }

    // 找第一个夹爪闭合帧
    int gripCloseSeg = -1;
    for (int seg = 1; seg < clip.frameCount; seg++) {
        if (!ExtractGripClose(clip.frame, seg - 1)
            && ExtractGripClose(clip.frame, seg)) {
            gripCloseSeg = seg;
            break;
        }
    }
    // 这里的作用是将轨迹分割为两段，前一段用梯形的加减速，后一段用五次的插值
    const bool useQuintic = (gripCloseSeg > 0 && gripCloseSeg < clip.frameCount - 1);

    // 前段: 逐帧 PlayTrajRow
    {
        const int frontEnd = useQuintic ? gripCloseSeg : clip.frameCount;
        if (!PlayTrajRows(arm_, clip.frame, frontEnd, rollOff, false, lastTarget))
            return false;
    }

    // Shift 确认（Ctrl+Z 中断 + 30s 超时）
    if (useQuintic) {
        const uint32_t tShift = HAL_GetTick();
        while (true) {
            if (keyboard_.key_Ctrl && keyboard_.key_Z) {
                return false;
            }
            if (keyboard_.key_Shift) {
                while (keyboard_.key_Shift) proc_waitMs(1);
                break;
            }
            if (HAL_GetTick() - tShift >= 30000) {
                return false;
            }
            proc_waitMs(1);
        }

        // 播放夹爪闭合帧
        if (!PlayTrajRow(arm_, clip.frame, gripCloseSeg, rollOff,
                              lastTarget, false, true))
            return false;
        Extrarow(clip.frame, gripCloseSeg, lastTarget);
        lastTarget[J::J_ENDR] += rollOff;
    }

    // 后段: 五次连续插值   
    if (useQuintic) {
        static CAlgoQuinticSpline spline;
        if (!PlaySplineRange(arm_, clip,gripCloseSeg, clip.frameCount - 1, spline, rollOff))
            return false;
    }
    return true;
}

} // namespace my_engineer
