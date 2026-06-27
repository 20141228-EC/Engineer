/******************************************************************************
 * @brief        轨迹回放模块实现
 * @file         proc_traj_common.cpp
 * @author       ciallo
 * @version      V2.0
 * @date         2026-4-23
 ******************************************************************************/

 #include "proc_common.hpp"

//debug
extern "C" {
volatile int32_t traj_dbg_seg = -1;
volatile int32_t traj_dbg_exit_reason = 0;      // 0 running, 1 ok, 2 ctrl_z, 3 关节超时, 4 夹爪超时
volatile int32_t traj_dbg_warn_reason = 0;      // 0 none, 3 等待慢关节, 4 夹爪闭合的慢
volatile int32_t traj_dbg_wait_joint = -1;
volatile float traj_dbg_max_joint_error = 0.0f;
volatile float traj_dbg_joint_current = 0.0f;
volatile float traj_dbg_joint_target = 0.0f;
volatile int32_t traj_dbg_wait_grip = 0;
volatile float traj_dbg_grip_cmd = 0.0f;
volatile float traj_dbg_grip_info = 0.0f;
}
 namespace my_engineer{

    /** @brief 读取机械臂关节角度
     *  @param arm 机械臂对象
     *  @param output 输出数组
     */

    void ReadArmjoint(const CModArm &arm,float_t output[J::COUNT]){
        output[J::J_YAW]  = arm.armInfo.angle_Yaw;
        output[J::J_P1]   = arm.armInfo.angle_Pitch1;
        output[J::J_P2]   = arm.armInfo.angle_Pitch2;
        output[J::J_ROLL] = arm.armInfo.angle_Roll;
        output[J::J_ENDP] = arm.armInfo.angle_end_pitch;
        output[J::J_ENDR] = arm.armInfo.angle_end_roll;
    }

    /** @brief 写入机械臂关节角度
     *  @param arm 机械臂对象
     *  @param output 输出数组
     */
    void WriteArmjoint(CModArm &arm,const float_t output[J::COUNT]){
        arm.armCmd.set_angle_Yaw = output[J::J_YAW];
        arm.armCmd.set_angle_Pitch1 = output[J::J_P1];
        arm.armCmd.set_angle_Pitch2 = output[J::J_P2];
        arm.armCmd.set_angle_Roll = output[J::J_ROLL];
        arm.armCmd.set_angle_end_pitch = output[J::J_ENDP];
        arm.armCmd.set_angle_end_roll = output[J::J_ENDR];
    }

    /** @brief 提取轨迹
     *  @param traj 轨迹数据
     *  @param row 轨迹行号
     *  @param output 输出数组
     */

    void Extrarow(const float_t traj[][FC_COUNT], int row, float_t output[J::COUNT]){
        output[J::J_YAW]  = traj[row][FC_YAW];
        output[J::J_P1]   = traj[row][FC_P1];
        output[J::J_P2]   = traj[row][FC_P2];
        output[J::J_ROLL] = traj[row][FC_ROLL];
        output[J::J_ENDP] = traj[row][FC_ENDP];
        output[J::J_ENDR] = traj[row][FC_ENDR];
    }

    /** @brief 检查关节角度是否到达目标的角度
     */
    bool CheckAllJointsArrived(const CModArm &arm, const float_t target[J::COUNT], const SArrivalCheckConfig &cfg) {
        float_t current[J::COUNT];
        ReadArmjoint(arm, current);

        bool arrived = true;
        int32_t maxErrJoint = -1;
        float_t maxErr = 0.0f;

        //等待超时标记关节方便debug
        for (int i = 0; i < J::COUNT; i++) {
            float_t err = std::fabs(current[i] - target[i]);
            if (err > maxErr) {
                maxErr = err;
                maxErrJoint = i;
                traj_dbg_joint_current = current[i];
                traj_dbg_joint_target = target[i];
            }
            if (err > cfg.toleranceDeg) {
                arrived = false;
            }
        }

        traj_dbg_wait_joint = maxErrJoint;
        traj_dbg_max_joint_error = maxErr;

        return arrived;
    }

    // 提取第 row 行的夹爪状态：0=夹紧, 非0=松开
    bool ExtractGripClose(const float_t traj[][FC_COUNT], int row) {
        return traj[row][FC_GRIP] < 1.0f;
    }

    //控制夹爪的张开和闭合
    void WriteGripCommand(CModArm &arm, bool close) {
        if (close) {
            arm.armCmd.gripClose = true;
            arm.armCmd.gripOpen = false;
        } else {
            arm.armCmd.gripClose = false;
            arm.armCmd.gripOpen = true;
        }
    }

    //检查夹爪是否到位
    bool CheckGripArrived(const CModArm &arm, bool close) {
        const auto state = arm.armInfo.gripState;
        if (close) {
            return state == CModArm::SArmInfo::EGripState::HOLD && arm.armInfo.length_grip <=  ARM_END_GRIP_PHYSICAL_RANGE_MAX - GRIP_CLOSE_Stop_distance;
        }

        return state == CModArm::SArmInfo::EGripState::RELEASE && arm.armInfo.length_grip >= ARM_END_GRIP_PHYSICAL_RANGE_MAX - GRIP_OPEN_Stop_distance;
    }

    //辅助debug夹爪函数
    bool WaitGripArrived(CModArm &arm, bool close, bool checkctrl,
                         const SArrivalCheckConfig &cfg) {
        const uint32_t startTick = HAL_GetTick();

        while (true) {
            if(checkctrl && SysRemote.remoteInfo.keyboard.key_Ctrl
               && SysRemote.remoteInfo.keyboard.key_Z) {
                traj_dbg_exit_reason = 2;
                return false;
            }

            WriteGripCommand(arm, close);

            const bool gripArrived = CheckGripArrived(arm, close);
            traj_dbg_wait_grip = gripArrived ? 0 : 1;
            traj_dbg_grip_cmd = arm.armCmd.set_length_grip;
            traj_dbg_grip_info = arm.armInfo.length_grip;

            if (gripArrived) {
                traj_dbg_exit_reason = 1;
                return true;
            }

            if (HAL_GetTick() - startTick >= cfg.gripTimeoutMs) {
                traj_dbg_warn_reason = 4;
            }

            if (HAL_GetTick() - startTick >= cfg.hardTimeoutMs) {
                traj_dbg_exit_reason = 4;
                return false;
            }

            proc_waitMs(1);
        }
    }


    // 五次关节播放器
    bool PlayJointTarget(CModArm &arm,
                         const float_t target[J::COUNT],
                         const SPlayJointTargetOptions &opt) {

        const SArrivalCheckConfig arrivalCfg;
        float_t current[J::COUNT];
        float_t joints[J::COUNT];

        // 规划起点
        if (opt.startOverride != nullptr) {
            for (int i = 0; i < J::COUNT; i++) current[i] = opt.startOverride[i];
        } else {
            ReadArmjoint(arm, current);
        }

        CAlgoQuintic qplayer;
        qplayer.speedScale = opt.speedScale;
        qplayer.PlanPointToPoint(current, target, opt.minTimeS);

        SPlayJointTargetState s;
        uint32_t startTick = HAL_GetTick();

        while (true) {
            // Ctrl+Z 打断
            if (SysRemote.remoteInfo.keyboard.key_Ctrl
                && SysRemote.remoteInfo.keyboard.key_Z) {
                traj_dbg_exit_reason = 2;
                return false;
            }

            const uint32_t nowTick = HAL_GetTick();
            float_t elapsed = static_cast<float_t>(nowTick - startTick) / 1000.0f;
            const bool frameTargetCommanded = qplayer.IsFinished(elapsed);

            // 关节角度播放
            if (frameTargetCommanded) {
                WriteArmjoint(arm, target);
            } else {
                qplayer.Evaluate(elapsed, joints);
                WriteArmjoint(arm, joints);
            }

            // 运动期间夹爪保持 opt.gripDuringMotion
            WriteGripCommand(arm, opt.gripDuringMotion);

            // 到位检查
            if (frameTargetCommanded) {
                if (opt.waitForArrival) {
                    if (!s.arrivalCheckStarted) {
                        s.arrivalCheckStarted = true;
                        s.arrivalStartTick = nowTick;
                    }
                    if (!s.frameJointsArrived) {
                        if (CheckAllJointsArrived(arm, target, arrivalCfg)) {
                            if (!s.stableTiming) {
                                s.stableTiming = true;
                                s.stableStartTick = nowTick;
                            }
                            if (nowTick - s.stableStartTick >= arrivalCfg.stableMs) {
                                s.frameJointsArrived = true;
                            }
                        } else {
                            s.stableTiming = false;
                        }
                        if (nowTick - s.arrivalStartTick >= arrivalCfg.timeoutMs) {
                            traj_dbg_warn_reason = 3;
                        }
                        if (nowTick - s.arrivalStartTick >= arrivalCfg.hardTimeoutMs) {
                            traj_dbg_exit_reason = 3;
                            return false;
                        }
                    }
                    if (s.frameJointsArrived) break;
                } else {
                    break;
                }
            }

            proc_waitMs(1);
        }

        // 关节到位后才切换到本段目标夹爪状态
        WriteArmjoint(arm, target);
        WriteGripCommand(arm, opt.gripAfter);
        traj_dbg_exit_reason = 1;
        return true;
    }

    // 按轨迹第 seg 行播放一段
    // earlyGrip=true: 段开始就切夹爪；false: 关节到位后才切
    bool PlayTrajRow(CModArm &arm,
                     const float_t traj[][FC_COUNT], int seg,
                     const float_t *prevTarget,
                     bool waitForArrival,
                     float_t endRollOffset,
                     bool earlyGrip) {
        float_t target[J::COUNT];
        const bool gripAfter = ExtractGripClose(traj, seg);   // 本段目标夹爪状态

        // 判断夹爪在本段是否真的切换
        const bool gripActuallyChanged = (seg == 0)
            ? !CheckGripArrived(arm, gripAfter)
            : (gripAfter != ExtractGripClose(traj, seg - 1));

        // earlyGrip: 段一开始就切；否则运动期间保持上一段状态，到位后才切
        bool gripDuringMotion;
        if (earlyGrip) {
            gripDuringMotion = gripAfter;
        } else if (seg == 0) {
            gripDuringMotion = (arm.armInfo.gripState == CModArm::SArmInfo::EGripState::HOLD);
        } else {
            gripDuringMotion = ExtractGripClose(traj, seg - 1);
        }

        Extrarow(traj, seg, target);
        target[J::J_ENDR] += endRollOffset;

        traj_dbg_seg = seg;
        traj_dbg_exit_reason = 0;
        traj_dbg_warn_reason = 0;
        traj_dbg_wait_joint = -1;

        SPlayJointTargetOptions opt;
        opt.speedScale = traj[seg][FC_SPEED];
        opt.gripDuringMotion = gripDuringMotion;
        opt.gripAfter = gripAfter;
        opt.startOverride = prevTarget;
        opt.minTimeS = 0.f;
        opt.waitForArrival = waitForArrival || gripActuallyChanged;  // 夹爪切换时强制等关节到位
        if (!PlayJointTarget(arm, target, opt))
            return false;

        // 夹爪有切换，关节到位后等夹爪到位再继续
        if (gripActuallyChanged) {
            const SArrivalCheckConfig cfg;
            return WaitGripArrived(arm, gripAfter, true, cfg);
        }
        return true;
    }


    // 按 traj 第 0..segEnd-1 行逐段播放
    bool PlayTrajRows(CModArm &arm,
                      const float_t traj[][FC_COUNT],
                      int segEnd,
                      float_t *lastTargetOut,
                      float_t endRollOffset,
                      bool earlyGrip) {
        if (segEnd <= 0) return true;

        float_t lastTarget[J::COUNT];

        // seg 0: 固定起点等到位
        if (!PlayTrajRow(arm, traj, 0, nullptr, true, endRollOffset, earlyGrip))
            return false;
        Extrarow(traj, 0, lastTarget);
        lastTarget[J::J_ENDR] += endRollOffset;

        // seg 1..segEnd-1: 逐段播放，prevTarget 链式
        for (int seg = 1; seg < segEnd; seg++) {
            const bool isLastSeg = (seg == segEnd - 1);
            const bool gripChanged = (ExtractGripClose(traj, seg) != ExtractGripClose(traj, seg - 1));
            const bool waitArrival = isLastSeg || gripChanged;
            if (!PlayTrajRow(arm, traj, seg, lastTarget, waitArrival, endRollOffset, earlyGrip))
                return false;
            Extrarow(traj, seg, lastTarget);
            lastTarget[J::J_ENDR] += endRollOffset;
        }

        if (lastTargetOut != nullptr) {
            for (int i = 0; i < J::COUNT; i++) lastTargetOut[i] = lastTarget[i];
        }
        return true;
    }

    // 五次插值范围连续播放
    bool PlaySplineRange(CModArm &arm,
                         const TrajClip &clip,
                         int segFrom,
                         int segTo,
                         CAlgoQuinticSpline &spline,
                         float_t rollOff) {
        if (segTo < 0) segTo = clip.frameCount - 1;
        const int n = segTo - segFrom + 1;
        if (n < 2) return true;

        spline.Build(clip.frame, clip.frameCount, segFrom, segTo, rollOff);

        const uint32_t t0 = HAL_GetTick();

        float_t joints[CAlgoQuinticSpline::AXES];

        while (true) {
            // Ctrl+Z 中断
            if (SysRemote.remoteInfo.keyboard.key_Ctrl
                && SysRemote.remoteInfo.keyboard.key_Z) {
                traj_dbg_exit_reason = 2;
                return false;
            }

            const uint32_t nowTick = HAL_GetTick();
            const float_t elapsedMs = static_cast<float_t>(nowTick - t0);

            // 轨迹结束：写入最终帧关节角度 + 等到位
            if (spline.IsFinished(elapsedMs)) {
                float_t finalTarget[J::COUNT];
                Extrarow(clip.frame, segTo, finalTarget);
                finalTarget[J::J_ENDR] += rollOff;
                WriteArmjoint(arm, finalTarget);

                {
                    const SArrivalCheckConfig cfg;
                    uint32_t arrivalTick = nowTick;
                    bool stable = false;
                    uint32_t stableTick = 0;
                    while (true) {
                        if (SysRemote.remoteInfo.keyboard.key_Ctrl
                            && SysRemote.remoteInfo.keyboard.key_Z) {
                            traj_dbg_exit_reason = 2;
                            return false;
                        }
                        if (CheckAllJointsArrived(arm, finalTarget, cfg)) {
                            if (!stable) { stable = true; stableTick = HAL_GetTick(); }
                            if (HAL_GetTick() - stableTick >= cfg.stableMs) break;
                        } else {
                            stable = false;
                            if (HAL_GetTick() - arrivalTick >= cfg.hardTimeoutMs) {
                                traj_dbg_exit_reason = 3;
                                return false;
                            }
                        }
                        proc_waitMs(1);
                    }
                }
                traj_dbg_exit_reason = 1;
                return true;
            }

            // 五次样条计算关节角度
            spline.Evaluate(elapsedMs, joints);
            WriteArmjoint(arm, joints);

            proc_waitMs(1);
        }
    }

 }// namespace my_engineer
