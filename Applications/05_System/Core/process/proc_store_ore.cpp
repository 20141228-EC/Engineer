/******************************************************************************
 * @brief        
 * 
 * @file         proc_store_ore.cpp
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
void CSystemCore::StartStoreOreTask(void *arg) {

    if (arg == nullptr) proc_return();

    // 获取SystemCore句柄
    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;
    auto cnt = 0;
    const auto timeout = 15000 / 5; // 15s超时退出

    cnt = timeout;
    
    /*step 1*/

    while(true){

        static EVarStatus phase1_is_arrived = false;
        static EVarStatus phase2_is_arrived = false;

        if(keyboard.mouse_L && !phase1_is_arrived && !phase2_is_arrived){   ///< 初次进入本任务，按左键确认后开始执行

            /*Set Arm*/
            core.parm_->armCmd.set_angle_Yaw = STORE_ORE_YAW_ANGLE_PHASE1;
            core.parm_->armCmd.set_angle_Pitch1 = STORE_ORE_PITCH1_ANGLE_PHASE1;
            core.parm_->armCmd.set_angle_Pitch2 = STORE_ORE_PITCH2_ANGLE_PHASE1;
            core.parm_->armCmd.set_angle_Pitch3 = STORE_ORE_PITCH3_ANGLE_PHASE1;
            core.parm_->armCmd.set_angle_Roll = STORE_ORE_ROLL_ANGLE_PHASE1;
            core.parm_->armCmd.set_angle_end_pitch = STORE_ORE_END_PITCH_ANGLE_PHASE1;
            core.parm_->armCmd.set_angle_end_roll = STORE_ORE_END_ROLL_ANGLE_PHASE1;
            core.parm_->armCmd.set_length_grip = STORE_ORE_GRIP_LENGTH_PHASE1;

            proc_waitMs(300);                           ///< 等待各电机到位
            phase1_is_arrived = true;
            break;                                      ///< 进入下一个step
        }
        else if(keyboard.mouse_L && phase1_is_arrived && !phase2_is_arrived){
            // core.parm_->
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