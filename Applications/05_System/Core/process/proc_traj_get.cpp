/******************************************************************************
 * @brief        兑换能量单元任务
 *
 * @file         proc_traj_get.cpp
 * @author       ciallo
 * @version      V1.0
 * @date         2026-6-14
 *
 * @copyright    Copyright (c) 2026
 *
 ******************************************************************************/

#include "proc_common.hpp"
#include "proc_traj_data.hpp"

namespace my_engineer {

    /******************************************************************************
    * @brief    兑换能量单元任务
    ******************************************************************************/
    void CSystemCore::StartExchangeGetTask(void *arg){
        if (arg == nullptr) proc_return();

        auto &core = *reinterpret_cast<CSystemCore *>(arg);
        auto &keyboard = SysRemote.remoteInfo.keyboard;
        auto &arm  = *core.parm_;

        // 选择左/右取矿轨迹
        ETrajID trajId;
        if (core.exchange_side_ == CSystemCore::EExchangeSide::LEFT) {
            trajId = TRAJ_EXCHANGE_L;
            core.armmode_ = EArmMode::EXCHANGE_L_ORE;
            core.exchange_side_ = CSystemCore::EExchangeSide::NONE;  ///< 清零
        } else if (core.exchange_side_ == CSystemCore::EExchangeSide::RIGHT) {
            trajId = TRAJ_EXCHANGE_R;
            core.armmode_ = EArmMode::EXCHANGE_R_ORE;
            core.exchange_side_ = CSystemCore::EExchangeSide::NONE;  ///< 清零
        } else {
            // 键盘模式：循环等待鼠标左键/右键选择轨迹
            while(true){
                if(keyboard.mouse_L){
                    trajId = TRAJ_EXCHANGE_L;          ///< 兑左矿
                    core.armmode_ = EArmMode::EXCHANGE_L_ORE;  // 更新系统层标志位
                    break;
                }
                if(keyboard.mouse_R){
                    trajId = TRAJ_EXCHANGE_R;              ///< 右键: 取右
                    core.armmode_ = EArmMode::EXCHANGE_R_ORE;
                    break;
                }
                proc_waitMs(5);
            }
        }
        core.pgimbal_->gimbalCmd.set_visualyaw = EXCHANGE_ORE_GIMBLE_YAW_ANGLE;
        core.pgimbal_->gimbalCmd.set_pitch = EXCHANGE_ORE_GIMBLE_PITCH_ANGLE;

        auto &Traj = (trajId == TRAJ_EXCHANGE_R) ? ExchangesingleClip_R : ExchangesingleClip_L;
        {
            arm.armCmd.isAutoCtrl = true;

            // 逐段播放
            if (!PlayTrajRows(arm, Traj.frame, Traj.frameCount,FastJointParams))
                goto proc_exit;
        }


    proc_exit:
        core.parm_->armCmd.isAutoCtrl = false;
        core.autoCtrlTaskHandle_ = nullptr;
        core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

        core.pgimbal_->gimbalCmd.set_visualyaw = EXCHANGE_ORE_GIMBLE_INIT_ANGLE;
        core.pgimbal_->gimbalCmd.set_pitch = EXCHANGE_ORE_GIMBLE_INIT_ANGLE;
        // 退出时
        arm.armCmd.set_angle_Yaw       = arm.armInfo.angle_Yaw;
        arm.armCmd.set_angle_Pitch1    = arm.armInfo.angle_Pitch1;
        arm.armCmd.set_angle_Pitch2    = arm.armInfo.angle_Pitch2;
        arm.armCmd.set_angle_Roll      = arm.armInfo.angle_Roll;
        arm.armCmd.set_angle_end_pitch = arm.armInfo.angle_end_pitch;
        arm.armCmd.set_angle_end_roll  = arm.armInfo.angle_end_roll;
        proc_waitMs(500);
        
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
