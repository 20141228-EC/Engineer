/******************************************************************************
 * @brief        
 * 
 * @file         proc_traj_grab.cpp
 * @author       ciallo
 * @version      V1.0
 * @date         2026-4-18
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/

#include "proc_common.hpp"
#include "System.hpp"

namespace my_engineer {

    /******************************************************************************
    * @brief    存能量单元任务
    ******************************************************************************/
    void CSystemCore::StartStoreTask(void *arg){
        if (arg == nullptr) proc_return();

        auto &core = *reinterpret_cast<CSystemCore *>(arg);
        auto &keyboard = SysRemote.remoteInfo.keyboard;
        auto &edge = SysRemote.remoteInfo.keyboard_edge;
        auto &arm  = *core.parm_;

        CAlgoTrajPlayback player;
        float_t endRollOffset = STORE_ROLL_DOWN_OFFSET;
        core.storeEndRollPose_ = EStoreEndRollPose::DOWN;//默认是向下的

        // 防止上次任务残留的 armCmd 值导致机械臂跳动 / 起点漂移
        arm.armCmd.set_angle_Yaw       = arm.armInfo.angle_Yaw;
        arm.armCmd.set_angle_Pitch1    = arm.armInfo.angle_Pitch1;
        arm.armCmd.set_angle_Pitch2    = arm.armInfo.angle_Pitch2;
        arm.armCmd.set_angle_Pitch3    = arm.armInfo.angle_Pitch3;
        arm.armCmd.set_angle_Roll      = arm.armInfo.angle_Roll;
        arm.armCmd.set_angle_end_pitch = arm.armInfo.angle_end_pitch;
        arm.armCmd.set_angle_end_roll  = arm.armInfo.angle_end_roll;

        // 重新标定末端
        arm.armCmd.resetEndAll = true;
        while(arm.comEnd_.initState_ !=  CModArm::CComEnd::EEndInitState::DONE){
            proc_waitMs(1);
        }

        // 循环等待鼠标左键/右键选择轨迹
        ETrajID trajId;
        while(true){
            if(keyboard.mouse_L){
                trajId = TRAJ_GRAB_L;              ///< 左键: 存左
                core.armmode_ = EArmMode::STORE_L_ORE;  // 更新系统层标志位
                break;
            }
            if(keyboard.mouse_R){
                trajId = TRAJ_GRAB_R;              ///< 右键: 存右
                core.armmode_ = EArmMode::STORE_R_ORE; 
                break;
            }
            proc_waitMs(5);
        }

        auto it = TrajMap.find(trajId);
        if(it == TrajMap.end()) goto proc_exit;

        /* Roll 手动标定：F/G 键微调末端 Roll（与控制器模式一致），鼠标左键确认 */
        {
            arm.armCmd.isAutoCtrl = true;    ///< 阻止外部ControlFromKeyboard_干扰

            while(!keyboard.key_Ctrl){
                if(edge.key_R == CSystemRemote::ERemoteEdge::Rising) {
                    if(core.storeEndRollPose_ == EStoreEndRollPose::DOWN) {
                        core.storeEndRollPose_ = EStoreEndRollPose::UP;
                        endRollOffset = STORE_ROLL_UP_OFFSET;
                    } else {
                        core.storeEndRollPose_ = EStoreEndRollPose::DOWN;
                        endRollOffset = STORE_ROLL_DOWN_OFFSET;
                    }
                }
                proc_waitMs(1);
            }
            // 左键确认: 当前电机Roll位置设为零点偏移
            //arm.comEnd_.rollZeroOffset = arm.comEnd_.endInfo.posit_Roll;
        }

        {
            const auto Traj = it->second; // 取到轨迹帧
            arm.armCmd.isAutoCtrl = true;

            //对齐末端的roll
            {
                float_t firstFrameTarget[J::COUNT];
                Extrarow(Traj.frame, 0, firstFrameTarget);

                float_t preAlignTarget[J::COUNT];
                ReadArmjoint(arm, preAlignTarget);                              // 当前位置作为起点
                preAlignTarget[J::J_ENDR] = firstFrameTarget[J::J_ENDR] + endRollOffset; // 仅修改 end_roll

                const bool gripNow = (arm.armInfo.gripState == CModArm::SArmInfo::EGripState::HOLD);
                // 提高末端roll转速猛转、注意限位块不要撞坏了
                if (!PlaySegment(arm, preAlignTarget, 10.0f,
                                 gripNow, gripNow,
                                 player, true, nullptr)) {
                    goto proc_exit;
                }
            }

            // 从第二段开始用它作为规划起点，避免每次反馈起点漂移
            float_t lastTarget[J::COUNT];

            /*step 1 :臂先到达固定的起始位姿*/
            if(!PlayFrameSegment(arm, Traj.frame, 0, player, true, endRollOffset, nullptr)) goto proc_exit;
            Extrarow(Traj.frame, 0, lastTarget);
            lastTarget[J::J_ENDR] += endRollOffset;
            proc_waitMs(50);    //等待夹爪收缩

            /*step 2 :逐段播放轨迹每段从实际关节位置开始规划*/
            for(int seg = 1; seg < Traj.frameCount; seg++){
                // 读实际关节位置覆盖 lastTarget，清除上一段的累积跟踪误差
                ReadArmjoint(arm, lastTarget);
                if(!PlayFrameSegment(arm, Traj.frame, seg, player, true, endRollOffset, lastTarget)) goto proc_exit;
                // 更新 lastTarget 为当前段的 target
                Extrarow(Traj.frame, seg, lastTarget);
                lastTarget[J::J_ENDR] += endRollOffset;
            }
        }


    proc_exit:
        core.parm_->armCmd.isAutoCtrl = false;
        core.autoCtrlTaskHandle_ = nullptr;
        core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

        //退出时把 armCmd 同步到当前实际位姿
        arm.armCmd.set_angle_Yaw       = arm.armInfo.angle_Yaw;
        arm.armCmd.set_angle_Pitch1    = arm.armInfo.angle_Pitch1;
        arm.armCmd.set_angle_Pitch2    = arm.armInfo.angle_Pitch2;
        arm.armCmd.set_angle_Pitch3    = arm.armInfo.angle_Pitch3;
        arm.armCmd.set_angle_Roll      = arm.armInfo.angle_Roll;
        arm.armCmd.set_angle_end_pitch = arm.armInfo.angle_end_pitch;
        arm.armCmd.set_angle_end_roll  = arm.armInfo.angle_end_roll;

        core.armmode_ = EArmMode::NORMAL;   //没有任务的状态
        core.storeEndRollPose_ = EStoreEndRollPose::DOWN;
        proc_return();
    }
}
