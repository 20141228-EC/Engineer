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
        arm.armCmd.resetEndAll = true;
        while(arm.comEnd_.initState_ !=  CModArm::CComEnd::EEndInitState::DONE){//如果没有初始化完成直接堵死在这里防止后续操作手手速过快
            proc_waitMs(1);
        }
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

            //从第二段开始用它作为规划起点，避免反馈起点漂移
            float_t lastTarget[J::COUNT];

            /*step 1 :臂先到达固定的起始位姿*/
            if(!PlayFrameSegment(arm, Traj.frame, 0, player, true, 0.0f, nullptr)) goto proc_exit;
            Extrarow(Traj.frame, 0, lastTarget);
            proc_waitMs(50);    //等待夹爪收缩

            /*step 2 :逐段播放轨迹*/
            for(int seg = 1; seg < Traj.frameCount; seg++){
                // 读实际关节位置覆盖 lastTarget，清除上一段的累积跟踪误差
                ReadArmjoint(arm, lastTarget);
                // earlyGrip=true: 夹爪切换与关节运动重叠，省去到位后单独等夹爪的时间
                if(!PlayFrameSegment(arm, Traj.frame, seg, player, true, 0.0f, lastTarget)) goto proc_exit;
                Extrarow(Traj.frame, seg, lastTarget);
            }
        }


    proc_exit:
        core.parm_->armCmd.isAutoCtrl = false;
        core.autoCtrlTaskHandle_ = nullptr;
        core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

        // 退出时把 armCmd 同步到当前实际位姿
        arm.armCmd.set_angle_Yaw       = arm.armInfo.angle_Yaw;
        arm.armCmd.set_angle_Pitch1    = arm.armInfo.angle_Pitch1;
        arm.armCmd.set_angle_Pitch2    = arm.armInfo.angle_Pitch2;
        arm.armCmd.set_angle_Pitch3    = arm.armInfo.angle_Pitch3;
        arm.armCmd.set_angle_Roll      = arm.armInfo.angle_Roll;
        arm.armCmd.set_angle_end_pitch = arm.armInfo.angle_end_pitch;
        arm.armCmd.set_angle_end_roll  = arm.armInfo.angle_end_roll;

        // 任务结束默认进入自定义控制器模式（仅在控制器在线时切换，否则保留键盘模式避免立刻被自动退出）
        if (SysControllerLink.IsControllerOnline()) {
            SysControllerLink.robotInfo.controlled_by_controller = true;
            core.use_Controller_ = true;
            arm.armCmd.isCustomCtrl = true;     ///< 同步标志位，避免下个周期的窗口期行为异常
        } else {
            SysControllerLink.robotInfo.controlled_by_controller = false;
            core.use_Controller_ = false;
            arm.armCmd.isCustomCtrl = false;
        }
        core.armmode_ = EArmMode::NORMAL;   //没有任务的状态
        proc_return();
    }
}