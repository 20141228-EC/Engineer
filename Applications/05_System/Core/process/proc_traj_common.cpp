/******************************************************************************
 * @brief        轨迹回放模块实现
 * @file         proc_traj_common.cpp
 * @author       ciallo
 * @version      V2.0
 * @date         2026-4-23
 ******************************************************************************/

 #include "proc_common.hpp"

 namespace my_engineer{
    

    /** @brief 读取机械臂关节角度
     *  @param arm 机械臂对象
     *  @param output 输出数组
     */

    void ReadArmjoint(const CModArm &arm,float_t output[7]){
        output[J::J_YAW]  = arm.armInfo.angle_Yaw;
        output[J::J_P1]   = arm.armInfo.angle_Pitch1;
        output[J::J_P2]   = arm.armInfo.angle_Pitch2;
        output[J::J_P3]   = arm.armInfo.angle_Pitch3;
        output[J::J_ROLL] = arm.armInfo.angle_Roll;
        output[J::J_ENDP] = arm.armInfo.angle_end_pitch;
        output[J::J_ENDR] = arm.armInfo.angle_end_roll;
    }

    /** @brief 写入机械臂关节角度
     *  @param arm 机械臂对象
     *  @param output 输出数组
     */
    void WriteArmjoint(CModArm &arm,const float_t output[7]){
        arm.armCmd.set_angle_Yaw = output[J::J_YAW];
        arm.armCmd.set_angle_Pitch1 = output[J::J_P1];
        arm.armCmd.set_angle_Pitch2 = output[J::J_P2];
        arm.armCmd.set_angle_Pitch3 = output[J::J_P3];
        arm.armCmd.set_angle_Roll = output[J::J_ROLL];
        arm.armCmd.set_angle_end_pitch = output[J::J_ENDP];
        arm.armCmd.set_angle_end_roll = output[J::J_ENDR];
    }

    /** @brief 提取轨迹
     *  @param traj 轨迹数据
     *  @param row 轨迹行号
     *  @param output 输出数组
     */

    void Extrarow(const float_t traj[][FC_COUNT], int row, float_t output[7]){
        for(int i = 0;i < 7;i++){
            output[i] = traj[row][i+1];
        }
    }

    /** @brief 检查关节角度是否到达目标的角度
     */
    bool CheckAllJointsArrived(const CModArm &arm, const float_t target[J::COUNT], const SArrivalCheckConfig &cfg) {
        float_t current[J::COUNT];
        ReadArmjoint(arm, current);

        for (int i = 0; i < J::COUNT; i++) {
            if (std::fabs(current[i] - target[i]) > cfg.toleranceDeg) {
                return false;
            }
        }

        return true;
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
            return state == CModArm::SArmInfo::EGripState::HOLD && arm.armInfo.length_grip <=  ARM_END_GRIP_PHYSICAL_RANGE_MAX - GRIP_CLOSE_Stop_distance;//增强判断依据防止夹爪的状态误判
        }

        return state == CModArm::SArmInfo::EGripState::RELEASE && arm.armInfo.length_grip >= ARM_END_GRIP_PHYSICAL_RANGE_MAX - GRIP_OPEN_Stop_distance;
    }

    //等待夹爪到位
    bool WaitGripArrived(CModArm &arm, bool close,
                         const SArrivalCheckConfig &cfg) {
        const uint32_t startTick = HAL_GetTick();

        while (true) {
            // ctrl+z操作手打断回放避免实际位姿错误或者出现干涉
            if (SysRemote.remoteInfo.keyboard.key_Ctrl
                && SysRemote.remoteInfo.keyboard.key_Z) {
                return false;
            }

            WriteGripCommand(arm, close);

            if (CheckGripArrived(arm, close)) {
                return true;
            }

            if (HAL_GetTick() - startTick >= cfg.hardTimeoutMs) {
                return false;
            }

            proc_waitMs(1);
        }
    }

    /** @brief 五次点到点播放器
     *  @param arm    臂的控制和信息参数
     *  @param target 目标关节角度
     *  @param opt    参数speedScale/gripDuringMotion/gripAfter/startOverride/minTimeS/waitForArrival
     */
    bool PlayJointTarget(CModArm &arm,
                     const float_t target[J::COUNT],
                     const SPlayJointTargetOptions &opt) {

        // 从 opt 解出参数
        const float_t speedScale      = opt.speedScale;
        const bool gripDuringMotion   = opt.gripDuringMotion;
        const bool gripAfter          = opt.gripAfter;
        const float_t *startOverride  = opt.startOverride;
        const float_t minTimeS        = opt.minTimeS;
        const bool waitForArrival     = opt.waitForArrival;

        const SArrivalCheckConfig arrivalCfg;//到位检查函数

        /*-----------------------  功能函数  ---------------------------*/
        // 夹爪控制，true=夹紧, false=松开
        float_t current[J::COUNT];
        float_t joints[J::COUNT];

        // 规划起点：优先使用 startOverride（上一段 target），否则读实时反馈
        if (startOverride != nullptr) {
            for (int i = 0; i < J::COUNT; i++) current[i] = startOverride[i];
        } else {
            ReadArmjoint(arm, current);
        }

        CAlgoQuintic qplayer;
        qplayer.speedScale = speedScale;                       // 设置当前的速度比例
        qplayer.PlanPointToPoint(current, target, minTimeS);

        SPlayJointTargetState s;

        uint32_t startTick = HAL_GetTick();//获取时间轴

        while(true){

            // ctrl+z操作手打断回放避免实际位姿错误或者出现干涉
            if (SysRemote.remoteInfo.keyboard.key_Ctrl
                && SysRemote.remoteInfo.keyboard.key_Z) {
                return false;
            }

            const uint32_t nowTick = HAL_GetTick();
            const uint32_t elapsedMs = nowTick - startTick;
            float_t elapsed = static_cast<float_t>(elapsedMs) / 1000.0f;//转换成当前秒数
            const bool frameTargetCommanded = qplayer.IsFinished(elapsed);

            // 关节角度播放
            if(frameTargetCommanded) {
                WriteArmjoint(arm, target);
            } else {
                qplayer.Evaluate(elapsed, joints);
                WriteArmjoint(arm, joints);
            }

            // 运动期间夹爪保持上一段状态
            WriteGripCommand(arm, gripDuringMotion);

            // 到位检查（waitForArrival=false 时跳过，轨迹结束直接退出）
            if(frameTargetCommanded) {
                if(waitForArrival) {
                    if(!s.arrivalCheckStarted) {
                        s.arrivalCheckStarted = true;
                        s.arrivalStartTick = nowTick;
                    }

                    if(!s.frameJointsArrived) {
                        if(CheckAllJointsArrived(arm, target, arrivalCfg)) {
                            if(!s.stableTiming) {
                                s.stableTiming = true;
                                s.stableStartTick = nowTick;
                            }

                            if(nowTick - s.stableStartTick >= arrivalCfg.stableMs) {
                                s.frameJointsArrived = true;
                            }
                        } else {
                            s.stableTiming = false;
                        }

                        if(nowTick - s.arrivalStartTick >= arrivalCfg.hardTimeoutMs) {
                            return false;
                        }
                    }

                    if(s.frameJointsArrived) break;
                } else {
                    break;  // 不等位：轨迹时间到立即退出
                }
            }

            proc_waitMs(1);
        }

        // 关节到位后才切换到本段目标夹爪状态
        WriteArmjoint(arm, target);
        WriteGripCommand(arm, gripAfter);
        return true;
    }


    /*再原来的播放器的基础上再封装一个速度读取的函数
    *  prevTarget 非空：用上一段 target 做起点
    *  prevTarget 为空：用实时反馈做起点
    *  earlyGrip  true: 夹爪在段开始时切换(与关节运动重叠)，false: 关节到位后才切换
    * */
    bool PlayTrajRow(CModArm &arm,
                                const float_t traj[][FC_COUNT], int seg,
                                float_t endRollOffset,
                                const float_t *prevTarget,
                                bool earlyGrip,
                                bool waitForArrival){
        float_t target[J::COUNT];
        const bool gripAfter = ExtractGripClose(traj, seg);   // 本段目标状态

        // 判断夹爪状态在本段是否真的发生了切换
        const bool gripActuallyChanged = (seg == 0)
            ? !CheckGripArrived(arm, gripAfter)
            : (gripAfter != ExtractGripClose(traj, seg - 1));

        // earlyGrip: 段一开始就切换夹爪
        // 关节运动期间保持上一段状态，到位后才切换
        bool gripDuringMotion;
        if (earlyGrip) {
            gripDuringMotion = gripAfter;
        } else if (seg == 0) {
            gripDuringMotion = (arm.armInfo.gripState == CModArm::SArmInfo::EGripState::HOLD);
        } else {
            gripDuringMotion = ExtractGripClose(traj, seg - 1);
        }

        float_t speed = traj[seg][FC_SPEED];
        Extrarow(traj, seg, target);
        target[J::J_ENDR] += endRollOffset;

        // 关节运动期间保持 gripDuringMotion；关节到位后才切到 gripAfter
        SPlayJointTargetOptions opt;
        opt.speedScale = speed;
        opt.gripDuringMotion = gripDuringMotion;
        opt.gripAfter = gripAfter;
        opt.startOverride = prevTarget;
        opt.minTimeS = 0.f;  // 最小时间 150ms，保证五次多项式平滑插值
        opt.waitForArrival = waitForArrival;
        if (!PlayJointTarget(arm, target, opt)) {
            return false;
        }
        if (gripActuallyChanged) {   // 夹爪有切换，等待夹爪到位
            const SArrivalCheckConfig arrivalCfg;
            return WaitGripArrived(arm, gripAfter, arrivalCfg);
        }
        return true;
    }


    bool PlayTrajRows(CModArm &arm,
                        const float_t traj[][FC_COUNT],
                        int segEnd,
                        float_t endRollOffset,
                        bool earlyGrip,
                        float_t *lastTargetOut) {
        if (segEnd <= 0) return true;

        float_t lastTarget[J::COUNT];

        // seg 0: 固定起点 + 等到位
        if (!PlayTrajRow(arm, traj, 0, endRollOffset, nullptr, earlyGrip, true))
            return false;
        Extrarow(traj, 0, lastTarget);
        lastTarget[J::J_ENDR] += endRollOffset;

        // seg 1 to segEnd-1: 逐段播放
        for (int seg = 1; seg < segEnd; seg++) {
            const bool isLastSeg = (seg == segEnd - 1);
            const bool gripChanged = (ExtractGripClose(traj, seg) != ExtractGripClose(traj, seg - 1));
            const bool waitArrival = isLastSeg || gripChanged;
            if (!PlayTrajRow(arm, traj, seg, endRollOffset, lastTarget, earlyGrip, waitArrival))
                return false;
            Extrarow(traj, seg, lastTarget);
            lastTarget[J::J_ENDR] += endRollOffset;
        }

        if (lastTargetOut != nullptr) {
            for (int i = 0; i < J::COUNT; i++) lastTargetOut[i] = lastTarget[i];
        }
        return true;
    }


    /**
     * @brief 五次样条段范围连续播放
     */
    bool PlaySplineRange(CModArm &arm, const TrajClip &clip,
                             int segFrom, int segTo,
                             CAlgoQuinticSpline &spline,
                             float_t rollOff) {

        if (segTo < 0) segTo = clip.frameCount - 1;
        const int n = segTo - segFrom + 1;
        if (n < 2) return true;

        // 时间轴内部归零
        spline.Build(clip.frame, clip.frameCount, segFrom, segTo, rollOff);

        const uint32_t t0 = HAL_GetTick();

        // 初始夹爪状态
        bool gripClose = ExtractGripClose(clip.frame, segFrom);
        WriteGripCommand(arm, gripClose);

        float_t joints[CAlgoQuinticSpline::AXES];
        int prevSeg = -1;

        while (true) {
            // Ctrl+Z 中断
            if (SysRemote.remoteInfo.keyboard.key_Ctrl
                && SysRemote.remoteInfo.keyboard.key_Z) {
                return false;
            }

            const uint32_t nowTick = HAL_GetTick();
            const float_t elapsedMs = static_cast<float_t>(nowTick - t0);

            // 轨迹结束：写入最终帧关节角度
            if (spline.IsFinished(elapsedMs)) {
                float_t finalTarget[J::COUNT];
                Extrarow(clip.frame, segTo, finalTarget);
                finalTarget[J::J_ENDR] += rollOff;
                WriteArmjoint(arm, finalTarget);

                // 等待关节到位
                {
                    const SArrivalCheckConfig cfg;
                    uint32_t arrivalTick = nowTick;
                    bool stable = false;
                    uint32_t stableTick = 0;
                    while (true) {
                        if (SysRemote.remoteInfo.keyboard.key_Ctrl
                            && SysRemote.remoteInfo.keyboard.key_Z) {
                            return false;
                        }
                        if (CheckAllJointsArrived(arm, finalTarget, cfg)) {
                            if (!stable) { stable = true; stableTick = HAL_GetTick(); }
                            if (HAL_GetTick() - stableTick >= cfg.stableMs) break;
                        } else {
                            stable = false;
                            if (HAL_GetTick() - arrivalTick >= cfg.hardTimeoutMs) {
                                return false;
                            }
                        }
                        proc_waitMs(1);
                    }
                }

                gripClose = ExtractGripClose(clip.frame, segTo);
                WriteGripCommand(arm, gripClose);

                const SArrivalCheckConfig cfg;
                if (!WaitGripArrived(arm, gripClose, cfg)) {
                    return false;
                }
                return true;
            }

            // 段切换时更新夹爪
            const int seg = spline.GetSegmentIndex(elapsedMs);
            if (seg != prevSeg) {
                prevSeg = seg;
                // seg 是样条内部段号，对应原 traj 的 segFrom+seg
                gripClose = ExtractGripClose(clip.frame, segFrom + seg);
                WriteGripCommand(arm, gripClose);
            }

            // 五次样条计算关节角度
            spline.Evaluate(elapsedMs, joints);
            WriteArmjoint(arm, joints);
            }

            proc_waitMs(1);
        }

}// namespace my_engineer
