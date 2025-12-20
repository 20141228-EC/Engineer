/******************************************************************************
 * @brief        
 * 
 * @file         proc_exchange_ore.cpp
 * @author       sllllr
 * @version      V1.0
 * @date         2025-12-15
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    兑换矿石任务
 ******************************************************************************/
void CSystemCore::StartExchangeOreTask(void *arg) {

    if (arg == nullptr) proc_return();

    // 获取SystemCore句柄
    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;
    auto cnt = 0;
    const auto timeout = 150000 / 5; // 15s超时退出

    cnt = timeout;
    
    /*step 1*/

    while(true){

        if(keyboard.mouse_L){               ///< 按左键确认后臂抬到固定位置，后面由操作手用控制器操作

        /*Set Arm*/
        core.parm_->armCmd.set_angle_Yaw = EXCHANGE_ORE_YAW_ANGLE;
        core.parm_->armCmd.set_angle_Pitch1 = EXCHANGE_ORE_PITCH1_ANGLE;
        core.parm_->armCmd.set_angle_Pitch2 = EXCHANGE_ORE_PITCH2_ANGLE;
        core.parm_->armCmd.set_angle_Roll = EXCHANGE_ORE_ROLL_ANGLE;
        core.parm_->armCmd.set_angle_end_pitch = EXCHANGE_ORE_END_PITCH_ANGLE;
        core.parm_->armCmd.set_angle_end_roll = EXCHANGE_ORE_END_ROLL_ANGLE;
        core.parm_->armCmd.set_length_grip = EXCHANGE_ORE_GRIP_LENGTH;

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

    core.pchassis_->chassisCmd.L_length = 0;    ///< 腿长给多少后面待改
    core.use_Controller_ = true;                ///< 使用自定义控制器

    while(true){
        if(keyboard.mouse_R){
            break;                              ///< 右键退出任务
        }
        proc_waitMs(20);
    }

// 退出
proc_exit:
    core.parm_->armCmd.isAutoCtrl = false;
    core.use_Controller_ = false;
    core.autoCtrlTaskHandle_ = nullptr;
    core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
    core.movemode_ = EMoveMode::NONE;
    core.use_Controller_ = false;
    proc_return();

}

} // namespace my_engineer