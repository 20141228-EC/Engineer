/******************************************************************************
 * @brief        
 * 
 * @file         proc_energy_unit.cpp
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
 * @brief    抓取能量单元任务
 ******************************************************************************/
void CSystemCore::StartEnergyUnitTask(void *arg) {

    if (arg == nullptr) proc_return();

    // 获取SystemCore句柄
    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;
    
    const auto timeout = 15000 / 5; // 15s
    auto cnt = timeout;
    
    while(true){

        if(keyboard.mouse_L){
            // 左键确认后，臂动到预设位置
            core.parm_->armCmd.isAutoCtrl = true;
            core.gimbal_auto_ctrl = true;

            /* Set Arm */
            core.parm_->armCmd.set_angle_Yaw = GRAB_ENERGY_UNIT_YAW_ANGLE;
            core.parm_->armCmd.set_angle_Pitch1 = GRAB_ENERGY_UNIT_PITCH1_ANGLE;
            core.parm_->armCmd.set_angle_Pitch2 = GRAB_ENERGY_UNIT_PITCH2_ANGLE;
            core.parm_->armCmd.set_angle_Roll = GRAB_ENERGY_UNIT_ROLL_ANGLE;
            core.parm_->armCmd.set_angle_end_pitch = GRAB_ENERGY_UNIT_END_PITCH_ANGLE;
            core.parm_->armCmd.set_angle_end_roll = GRAB_ENERGY_UNIT_END_ROLL_ANGLE;
            core.parm_->armCmd.set_length_grip = GRAB_ENERGY_UNIT_GRIP_LENGTH;

            proc_waitMs(300); // 等待各电机到位
            break; // 进入下一个step
        }
        else if(cnt > 0){
            cnt--;
        }
        else if(cnt <= 0){
            goto proc_exit; // 超时退出
        }

        proc_waitMs(5);
    }

    /*step2*/
    core.use_Controller_ = true;                // 使用自定义控制器进行微调

    while(true){
        if(keyboard.mouse_R){
            break; // 右键退出任务
        }
        proc_waitMs(20);
    }

// 退出
proc_exit:
    core.parm_->armCmd.isAutoCtrl = false;
    core.gimbal_auto_ctrl = false;
    core.use_Controller_ = false;
    core.autoCtrlTaskHandle_ = nullptr;
    core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
    core.movemode_ = EMoveMode::NONE;
    proc_return();

}

} // namespace my_engineer