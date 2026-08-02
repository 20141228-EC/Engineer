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
#include "traj_data.hpp"

namespace my_engineer {

    /******************************************************************************
    * @brief    兑换能量单元任务
    ******************************************************************************/
    void CSystemCore::StartExchangeGetTask(void *arg){
        if (arg == nullptr) proc_return();

        auto &core = *reinterpret_cast<CSystemCore *>(arg);
        auto &keyboard = SysRemote.remoteInfo.keyboard;
        auto &arm  = *core.parm_;

        arm.armCmd.resetEndAll = true;
        while(arm.comEnd_.initState_ !=  CModArm::CComEnd::EEndInitState::DONE){//如果没有初始化完成直接堵死在这里防止后续操作手手速过快
            proc_waitMs(1);
        }
        // 选择取矿轨迹（控制器功能按键 / 键盘手动选择）
        ETrajID trajId;
        if (core.exchange_side_ == CSystemCore::EExchangeSide::LEFT) {
            trajId = TRAJ_EXCHANGE_L;
            core.armmode_ = EArmMode::EXCHANGE_L_ORE;
            core.exchange_side_ = CSystemCore::EExchangeSide::NONE;
        } else if (core.exchange_side_ == CSystemCore::EExchangeSide::RIGHT) {
            trajId = TRAJ_EXCHANGE_R;
            core.armmode_ = EArmMode::EXCHANGE_R_ORE;
            core.exchange_side_ = CSystemCore::EExchangeSide::NONE;
        } else {
            while(true){
                if(keyboard.mouse_L){
                    trajId = TRAJ_EXCHANGE_L;
                    core.armmode_ = EArmMode::EXCHANGE_L_ORE;
                    break;
                }
                if(keyboard.mouse_R){
                    trajId = TRAJ_EXCHANGE_R;
                    core.armmode_ = EArmMode::EXCHANGE_R_ORE;
                    break;
                }
                proc_waitMs(5);
            }
        }


            auto &Traj = (trajId == TRAJ_EXCHANGE_R) ? ExchangesingleClip_R : ExchangesingleClip_L;
        {
            arm.armCmd.isAutoCtrl = true;

            // 逐段播放
            if (!PlayTrajRows(arm, Traj.frame, Traj.frameCount))
                goto proc_exit;
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