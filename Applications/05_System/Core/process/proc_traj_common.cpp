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

    /*----------------------------------------功能函数------------------------------------------------*/
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
    /** @brief 拷贝机械臂关节角度
     *  @param input 机械臂对象
     *  @param target 输出数组
     */
    void CopyArmJoint(float_t inputjoint[J::COUNT],const float_t targetjoint[J::COUNT]){
        inputjoint[J::J_YAW] = targetjoint[J::J_YAW];
        inputjoint[J::J_P1] = targetjoint[J::J_P1];
        inputjoint[J::J_P2] = targetjoint[J::J_P2];
        inputjoint[J::J_ROLL] = targetjoint[J::J_ROLL];
        inputjoint[J::J_ENDP] = targetjoint[J::J_ENDP];
        inputjoint[J::J_ENDR] = targetjoint[J::J_ENDR];
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
        for (int i = 0; i < J::COUNT; i++) {
            float_t err = std::fabs(current[i] - target[i]);
            if (err > cfg.toleranceDeg) {
                arrived = false;
            }
        }

        return arrived;
    }

    // 提取第 row 行的夹爪状态：0=夹紧, 非0=松开
    bool ExtractGripClose(const float_t traj[][FC_COUNT], int row) {
        return traj[row][FC_GRIP] < 1.0f;
    }

    //控制夹爪的张开和闭合，false是闭合，true是张开
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
                return false;
            }

            WriteGripCommand(arm, close);

            const bool gripArrived = CheckGripArrived(arm, close);

            if (gripArrived) {
                return true;
            }

            if (HAL_GetTick() - startTick >= cfg.hardTimeoutMs) {
                return false;
            }

            proc_waitMs(1);
        }
    }

    void SetParam(CAlgoQuinticSpline &spline, CAlgoTrajPlayback::SConfig &param){
        for (int i = 0; i < CAlgoQuinticSpline::AXES; i++) {
                spline.velLimit[i] = static_cast<float>(param.jointParams[i].velMax);
                spline.accLimit[i] = static_cast<float>(param.jointParams[i].accMax);
            }
    }

    /*----------------------------------------播放执行函数------------------------------------------------*/
    /*  单段播放：把机械臂从当前位姿五次多项式平滑运动到一组目标关节角
        输入是 target[6] 数组（非轨迹文件），自带到位检查和 Ctrl+Z 打断
        用途：只有目标姿态、无轨迹文件时用（瞄准、归位、对齐 roll）
    */ 
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
        if (opt.jointParamsOverride) {
            for (int i = 0; i < J::COUNT; i++)
                qplayer.config_.jointParams[i] = opt.jointParamsOverride[i];
        }
        qplayer.PlanPointToPoint(current, target, opt.minTimeS);

        SPlayJointTargetState s;
        uint32_t startTick = HAL_GetTick();

        while (true) {
            // Ctrl+Z 打断
            if (SysRemote.remoteInfo.keyboard.key_Ctrl
                && SysRemote.remoteInfo.keyboard.key_Z) {
                return false;
            }

            const uint32_t nowTick = HAL_GetTick();
            float_t elapsed = static_cast<float_t>(nowTick - startTick) / 1000.0f;

            const bool frameTargetCommanded = qplayer.IsFinished(elapsed);// 自动计算路径的运动时间

            // 关节角度播放
            if (frameTargetCommanded) {
                WriteArmjoint(arm, target);
            } else {
                qplayer.Evaluate(elapsed, joints);
                WriteArmjoint(arm, joints);
            }

            // 运动期间夹爪保持 opt.gripDuringMotion,如果传入了保持命令则不动
            if(!opt.gripKeepCurrent){
                WriteGripCommand(arm, opt.gripDuringMotion);
            }

            // 到位检查
            if (frameTargetCommanded) {
                if (opt.waitForArrival) {
                    if (!s.arrivalCheckStarted) {
                        s.arrivalCheckStarted = true;
                        s.arrivalStartTick = nowTick;
                    }
                    if (!s.frameJointsArrived) {
                        if (CheckAllJointsArrived(arm, target, arrivalCfg)) { //到位检查
                            if (!s.stableTiming) {
                                s.stableTiming = true;
                                s.stableStartTick = nowTick;
                            }
                            if (nowTick - s.stableStartTick >= arrivalCfg.stableMs) {
                                s.frameJointsArrived = true;// 超时检测
                            }
                        } else {
                            s.stableTiming = false;
                        }
                        if (nowTick - s.arrivalStartTick >= arrivalCfg.hardTimeoutMs) {
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
        if(!opt.gripKeepCurrent){
            WriteGripCommand(arm, opt.gripAfter);
        }
        return true;
    }

    /* 单关节平滑规划到绝对角度*/
    bool MoveSingleJointToAbs(CModArm &arm,
                              int jointIdx,
                              float_t targetAbs,
                              const SPlayJointTargetOptions &opt) {
        float_t current[J::COUNT];
        ReadArmjoint(arm, current);              // 只读一次反馈

        float_t target[J::COUNT];
        for (int i = 0; i < J::COUNT; i++) target[i] = current[i];
        target[jointIdx] = targetAbs;           // 仅修改目标关节

        SPlayJointTargetOptions newOpt = opt;
        newOpt.startOverride = current;

        return PlayJointTarget(arm, target, newOpt);
    }

    /* 
       单段播放：按轨迹第 seg 行播放一段，内部调用 PlayJointTarget 
       比 PlayJointTarget 多处理轨迹行解析、endRollOffset、夹爪切换语义
       earlyGrip=true 段开始就切夹爪，false 关节到位后才切
       用途：只播轨迹某一帧时用（如 Shift 确认后的夹爪闭合帧）
    */
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

        SPlayJointTargetOptions opt;
        opt.speedScale = traj[seg][FC_SPEED];
        opt.gripDuringMotion = gripDuringMotion;
        opt.gripAfter = gripAfter;
        opt.startOverride = nullptr;// 直接传上一次的目标值的效果比较差，所以这里直接设置nullptr
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

    /* 
       多段播放：按轨迹第 0..segEnd-1 行逐段播放，段间停顿到位（stop-and-go）
       中间段不等到位以保持连贯，除非末段或夹爪切换
       用途：要求中间精确到位时用（如夹爪闭合前的取矿前段）
    */
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

    /*
        多段播放：用五次样条把 segFrom..segTo 拟合为连续曲线，全程不停顿
        关键帧处位置/速度/加速度都连续，只在整段结束时检查一次到位
        用途：要求平滑衔接、不能停顿时用（如夹住矿后的拔出段）
    */ 
    bool PlaySplineRange(CModArm &arm,
                         const TrajClip &clip,
                         int segFrom,
                         int segTo,
                         CAlgoQuinticSpline &spline,
                         float_t rollOff,
                         float_t speedScale) {
        if (segTo < 0) segTo = clip.frameCount - 1;
        const int n = segTo - segFrom + 1;
        if (n < 2) return true;

        spline.speedScale = speedScale;
        spline.Build(clip.frame, clip.frameCount, segFrom, segTo, rollOff);

        const uint32_t t0 = HAL_GetTick();

        float_t joints[CAlgoQuinticSpline::AXES];

        while (true) {
            // Ctrl+Z 中断
            if (SysRemote.remoteInfo.keyboard.key_Ctrl
                && SysRemote.remoteInfo.keyboard.key_Z) {
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
                return true;
            }

            // 五次样条计算关节角度
            spline.Evaluate(elapsedMs, joints);
            WriteArmjoint(arm, joints);

            proc_waitMs(1);
        }
    }

 }// namespace my_engineer
