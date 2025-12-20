/******************************************************************************
 * @brief        
 * 
 * @file         proc_save_ore.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-15
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    存矿任务
 ******************************************************************************/
void CSystemCore::StartSaveOreTask(void *arg) {

    if (arg == nullptr) proc_return();

    // 获取SystemCore句柄
    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;
    auto cnt = 0;
    const auto timeout = 15000 / 5; // 15s超时退出

    cnt = timeout;
    
    /*step 1*/

    while(true){

        if(keyboard.mouse_L){               ///< 按左键确认后开始执行

            /*Set Arm*/
            core.parm_->armCmd.set_angle_Yaw = SAVE_ORE_YAW_ANGLE;
            core.parm_->armCmd.set_angle_Pitch1 = SAVE_ORE_PITCH1_ANGLE;
            core.parm_->armCmd.set_angle_Pitch2 = SAVE_ORE_PITCH2_ANGLE;
            core.parm_->armCmd.set_angle_Roll = SAVE_ORE_ROLL_ANGLE;
            core.parm_->armCmd.set_angle_end_pitch = SAVE_ORE_END_PITCH_ANGLE;
            core.parm_->armCmd.set_angle_end_roll = SAVE_ORE_END_ROLL_ANGLE;
            core.parm_->armCmd.set_length_grip = SAVE_ORE_GRIP_LENGTH;

            // proc_waitMs(1000);

            // core.parm_->armCmd.set_length_grip = 0; // 松开夹爪

            // proc_waitMs(500);

            // // 回到初始位置
            // core.parm_->armCmd.set_angle_Yaw = 0;
            // core.parm_->armCmd.set_angle_Pitch1 = 0;
            // core.parm_->armCmd.set_angle_Pitch2 = 0;
            // core.parm_->armCmd.set_angle_Roll = 0;
            // core.parm_->armCmd.set_angle_end_pitch = 0;
            // core.parm_->armCmd.set_angle_end_roll = 0;
            // core.parm_->armCmd.set_length_grip = 0;

			// 后续看情况是否要加上面这段

            proc_waitMs(300);                           ///< 等待各电机到位
            break;                                      ///< 进入下一个step
        }
        else if(cnt > 0){
            cnt--;
        }
        else if(cnt <= 0){
            goto proc_exit;         ///< 超时退出
        }

        proc_waitMs(5);
    }

    /*step 2*/

    while(true){
        if(keyboard.mouse_R){
            break;                              ///< 右键退出任务
        }
        proc_waitMs(20);
    }

// 退出
proc_exit:
    core.parm_->armCmd.isAutoCtrl = false;
    core.autoCtrlTaskHandle_ = nullptr;
    core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
    core.movemode_ = EMoveMode::NONE;
    core.use_Controller_ = false;
    proc_return();

}

} // namespace my_engineer
