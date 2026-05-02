/******************************************************************************
 * @brief        
 * 
 * @file         proc_traj_get.cpp
 * @author       ciallo
 * @version      V1.0
 * @date         2026-4-18
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/

#include "proc_common.hpp"

namespace my_engineer {

    /******************************************************************************
    * @brief    取能量单元任务
    ******************************************************************************/
    void CSystemCore::StartExchangeGetTask(void *arg){
        if (arg == nullptr) proc_return();

        auto &core = *reinterpret_cast<CSystemCore *>(arg);
        auto &keyboard = SysRemote.remoteInfo.keyboard;
        auto &arm  = *core.parm_;

        CAlgoTrajPlayback player;

        // 循环等待鼠标左键/右键选择轨迹
        ETrajID trajId;
        while(true){
            if(keyboard.mouse_L){
                trajId = TRAJ_GET_L;              ///< 左键: 取左
                core.armmode_ = EArmMode::STORE_L_ORE;  // 更新系统层标志位
                break;
            }
            if(keyboard.mouse_R){
                trajId = TRAJ_GET_R;              ///< 右键: 取右
                core.armmode_ = EArmMode::STORE_R_ORE; 
                break;
            }
            proc_waitMs(5);
        }

        auto it = TrajMap.find(trajId);
        if(it == TrajMap.end()) goto proc_exit;

        /* Roll 手动标定：F/G 键微调末端 Roll（与控制器模式一致），Ctrl键确认 */
        {
            arm.armCmd.isAutoCtrl = true;    ///< 阻止外部ControlFromKeyboard_干扰
            while(!keyboard.key_Ctrl){
                arm.armCmd.set_angle_end_roll += static_cast<float_t>(keyboard.key_F - keyboard.key_G) * 60.0f / 1000.f;
                proc_waitMs(1);
            }
            // Ctrl确认: 当前电机Roll位置设为零点偏移
            //arm.comEnd_.rollZeroOffset = arm.comEnd_.endInfo.posit_Roll;
        }

        {
            const auto Traj = it->second; // 取到轨迹帧
            arm.armCmd.isAutoCtrl = true;

            /*step 1 :臂先到达固定的起始位姿*/
            if(!PlayFrameSegment(arm, Traj.frame, 0, player, true)) goto proc_exit;
            proc_waitMs(50);    //等待夹爪收缩

            /*step 2 :逐段播放轨迹*/
            for(int seg = 1; seg < Traj.frameCount; seg++){
                if(!PlayFrameSegment(arm, Traj.frame, seg, player, true)) goto proc_exit;
            }
        }


    proc_exit:
        core.parm_->armCmd.isAutoCtrl = false;
        core.autoCtrlTaskHandle_ = nullptr;
        core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
        core.armmode_ = EArmMode::NORMAL;   //没有任务的状态
        proc_return();
    }
}