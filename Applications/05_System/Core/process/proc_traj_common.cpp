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

    void ReadArmjoint(const CModArm &arm,float_t output[7]){
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
    void WriteArmjoint(CModArm &arm,const float_t output[7]){
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
    // 提取第 row 行的关节速度状态
    float_t ExtractSpeed(const float_t traj[][FC_COUNT], int row) {
        return traj[row][FC_SPEED];
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

    /** @brief 轨迹播放器
     *  @param arm 臂的控制和信息参数
     *  @param target 目标关节角度
     *  @param gripDuringMotion 关节运动过程中夹爪保持的状态，关节运动过程中保持的夹爪状态（true=夹紧, false=松开）
     *  @param gripAfter        关节到位之后才切换的夹爪状态
     *  @param player 播放器的内部速度参数定义
     *  @param startOverride 非空：用其作为规划起点
     *  @param minTimeS 可选最小总时长(秒)
     */
    bool PlaySegment(CModArm &arm, const float_t target[J::COUNT],
                           float_t speedScale,
                           bool gripDuringMotion, bool gripAfter,
                           CAlgoTrajPlayback &player, bool checkctrl,
                           const float_t *startOverride,
                           float_t minTimeS){

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

        player.speedScale = speedScale;                       // 设置当前的速度比例
        player.PlanMultiAxisTraj(current, target, minTimeS);  // 最小时长约束

        bool arrivalCheckStarted = false;
        bool stableTiming = false;
        bool frameJointsArrived = false;
        uint32_t arrivalStartTick = 0;
        uint32_t stableStartTick = 0;

        uint32_t startTick = HAL_GetTick();//获取时间轴

        while(true){

            // ctrl+z操作手打断回放避免实际位姿错误或者出现干涉
            if(checkctrl && SysRemote.remoteInfo.keyboard.key_Ctrl
               && SysRemote.remoteInfo.keyboard.key_Z) {
                traj_dbg_exit_reason = 2;
                return false;
            }

            const uint32_t nowTick = HAL_GetTick();
            const uint32_t elapsedMs = nowTick - startTick;
            float_t elapsed = static_cast<float_t>(elapsedMs) / 1000.0f;//转换成当前秒数
            const bool frameTargetCommanded = player.IsFinished(elapsed);// 本帧运动完成

            // 关节角度播放
            if(frameTargetCommanded) {
                WriteArmjoint(arm, target);
            } else {
                player.MultiAxisTrajDistance(elapsed, joints);//根据当前的秒数度取关节的信息
                WriteArmjoint(arm, joints);//将获取到的关节信息反写入arm中
            }

            // 运动期间夹爪保持上一段状态，禁止提前切换
            WriteGripCommand(arm, gripDuringMotion);

//测试代码：
            if(frameTargetCommanded) {
                if(!arrivalCheckStarted) {
                    arrivalCheckStarted = true;
                    arrivalStartTick = nowTick;
                }

                if(!frameJointsArrived) {
                    if(CheckAllJointsArrived(arm, target, arrivalCfg)) {
                        if(!stableTiming) {
                            stableTiming = true;
                            stableStartTick = nowTick;
                        }

                        if(nowTick - stableStartTick >= arrivalCfg.stableMs) {
                            frameJointsArrived = true;
                        }
                    } else {
                        stableTiming = false;
                    }

                    if(nowTick - arrivalStartTick >= arrivalCfg.timeoutMs) {
                        traj_dbg_warn_reason = 3;
                    }

                    //Debug: 超时退出
                    if(nowTick - arrivalStartTick >= arrivalCfg.hardTimeoutMs) {
                        traj_dbg_exit_reason = 3;
                        return false;
                    }
                }

                // 关节到位后退出，夹爪切换由 PlayFrameSegment
                if(frameJointsArrived) break;
            }

            proc_waitMs(1);
        }

        // 关节到位后才切换到本段目标夹爪状态（夹爪到位检查由 PlayFrameSegment 负责）
        WriteArmjoint(arm, target);
        WriteGripCommand(arm, gripAfter);
        traj_dbg_exit_reason = 1;
        return true;
    }


    //再原来的播放器的基础上再封装一个速度读取的函数
    //  prevTarget 非空：用上一段 target 做起点
    //  prevTarget 为空：用实时反馈做起点
    //  earlyGrip  true: 夹爪在段开始时切换(与关节运动重叠)，false: 关节到位后才切换
    bool PlayFrameSegment(CModArm &arm,
                                const float_t traj[][FC_COUNT], int seg,
                                CAlgoTrajPlayback &player, bool checkctrl,
                                float_t endRollOffset,
                                const float_t *prevTarget,
                                bool earlyGrip){
        float_t target[J::COUNT];
        const bool gripAfter = ExtractGripClose(traj, seg);   // 本段目标状态

        // 判断夹爪状态在本段是否真的发生了切换
        const bool gripActuallyChanged = (seg == 0)
            ? !CheckGripArrived(arm, gripAfter)
            : (gripAfter != ExtractGripClose(traj, seg - 1));

        // earlyGrip: 段一开始就切换夹爪
        // 否则: 关节运动期间保持上一段状态，到位后才切
        bool gripDuringMotion;
        if (earlyGrip) {
            gripDuringMotion = gripAfter;
        } else if (seg == 0) {
            gripDuringMotion = (arm.armInfo.gripState == CModArm::SArmInfo::EGripState::HOLD);
        } else {
            gripDuringMotion = ExtractGripClose(traj, seg - 1);
        }

        float_t speed = ExtractSpeed(traj, seg);
        Extrarow(traj, seg, target);
        target[J::J_ENDR] += endRollOffset;

        traj_dbg_seg = seg;
        traj_dbg_exit_reason = 0;
        traj_dbg_warn_reason = 0;
        traj_dbg_wait_joint = -1;
        traj_dbg_wait_grip = 0;

        // 关节运动期间保持 gripDuringMotion；关节到位后才切到 gripAfter
        if(!PlaySegment(arm, target, speed,
                        gripDuringMotion, gripAfter,
                        player, checkctrl,
                        prevTarget )) {// minTimeS = 0，按物理参数自由规划 
            return false;
        }
        if(gripActuallyChanged) {   // 夹爪有切换，等待夹爪到位
            const SArrivalCheckConfig arrivalCfg;
            return WaitGripArrived(arm, gripAfter, checkctrl, arrivalCfg);
        }
        return true;
    }


 }// namespace my_engineer
