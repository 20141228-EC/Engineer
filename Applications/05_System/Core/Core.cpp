/**
 * @file Core.cpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 系统核心源文件
 * @version 1.0
 * @date 2024-11-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "Core.hpp"

namespace my_engineer {

// 实例化系统核心
CSystemCore SystemCore;

/**
 * @brief 初始化系统核心
 * 
 * @return EAppStatus 
 */
EAppStatus CSystemCore::InitSystemCore() {
    
    // 启动通信接口发送
    for (const auto &item : InterfaceIDMap) {
        item.second->StartTransfer();
    }


    // 初始化所有系统
    CSystemRemote::SSystemInitParam_Remote remoteInitParam;
    remoteInitParam.systemID = ESystemID::SYS_REMOTE;
    remoteInitParam.remoteDevID = EDeviceID::DEV_RC_DR16;
    SysRemote.InitSystem(&remoteInitParam);

    CSystemVision::SSystemInitParam_Vision visionInitParam;
    visionInitParam.systemID = ESystemID::SYS_VISION;
    visionInitParam.visionDevID = EDeviceID::DEV_VISION;
    SysVision.InitSystem(&visionInitParam);

    CSystemReferee::SSystemInitParam_Referee refereeInitParam;
    refereeInitParam.systemID = ESystemID::SYS_REFEREE;
    refereeInitParam.refereeDevID = EDeviceID::DEV_RM_REFEREE;
    SysReferee.InitSystem(&refereeInitParam);

    CSystemControllerLink::SSystemInitParam_ControllerLink controllerLinkInitParam;
    controllerLinkInitParam.systemID = ESystemID::SYS_CONTROLLER_LINK;
    controllerLinkInitParam.controllerLinkDevID = EDeviceID::DEV_CONTROLLER_LINK;
    SysControllerLink.InitSystem(&controllerLinkInitParam);

    CSystemESP32::SSystemInitParam_ESP32 esp32InitParam;
    esp32InitParam.systemID = ESystemID::SYS_ESP32;
    esp32InitParam.esp32DevID = EDeviceID::DEV_ESP32;
    SysESP32.InitSystem(&esp32InitParam);

    CSystemBoardLink::SSystemInitParam_BoardLink boardLinkInitParam;
    boardLinkInitParam.systemID = ESystemID::SYS_BOARD_LINK;
    boardLinkInitParam.boardLinkDevID = EDeviceID::DEV_BOARD_LINK;
    SysBoardLink.InitSystem(&boardLinkInitParam);

    // 获取模块的指针（安全查找，避免异常）
    auto it_chassis = ModuleIDMap.find(EModuleID::MOD_CHASSIS);
    if (it_chassis != ModuleIDMap.end() && it_chassis->second != nullptr) {
        pchassis_ = reinterpret_cast<CModChassis *>(it_chassis->second);
    }

    auto it_arm = ModuleIDMap.find(EModuleID::MOD_ARM);
    if (it_arm != ModuleIDMap.end() && it_arm->second != nullptr) {
        parm_ = reinterpret_cast<CModArm *>(it_arm->second);
    }

    auto it_gimbal = ModuleIDMap.find(EModuleID::MOD_GIMBAL);
    if (it_gimbal != ModuleIDMap.end() && it_gimbal->second != nullptr) {
        pgimbal_ = reinterpret_cast<CModGimbal *>(it_gimbal->second);
    }

    proc_waitMs(1200); // 等待系统初始化完成

    coreStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 更新处理
 * 
 */
void CSystemCore::UpdateHandler_() {

    // 检查系统核心状态
    if (coreStatus == APP_RESET) return;

    // 检查遥控器系统状态
    if (SysRemote.systemStatus == APP_RESET){
        RTT_LOG_ERROR("Remote is offline!!!");
        return;
    } 

    if (SysRemote.ResetFlag)
    {
        RESET_SYSTEM();
    }

    static bool last_use_Controller = false;
    static uint8_t zx_count = 0;
    static bool zx_flag = false;

    // static uint8_t print_cnt = 0;
    // if (print_cnt-- == 0) {
    //     print_cnt = 200;
    //     Print("------------------------------\n");
    //     if (parm_) {
    //         Print("Arm_Yaw_Cmd: %d, Info: %d, Err: %d\n",
    //               static_cast<int>(parm_->armCmd.set_angle_Yaw), static_cast<int>(parm_->armInfo.angle_Yaw),
    //               static_cast<int>(parm_->armInfo.angle_Yaw - parm_->armCmd.set_angle_Yaw));
    //         Print("Arm_Pitch1_Cmd: %d, Info: %d, Err: %d\n",
    //               static_cast<int>(parm_->armCmd.set_angle_Pitch1), static_cast<int>(parm_->armInfo.angle_Pitch1),
    //               static_cast<int>(parm_->armInfo.angle_Pitch1 - parm_->armCmd.set_angle_Pitch1));
    //         Print("Arm_Pitch2_Cmd: %d, Info: %d, Err: %d\n",
    //               static_cast<int>(parm_->armCmd.set_angle_Pitch2), static_cast<int>(parm_->armInfo.angle_Pitch2),
    //               static_cast<int>(parm_->armInfo.angle_Pitch2 - parm_->armCmd.set_angle_Pitch2));
    //         Print("Arm_Roll_Cmd: %d, Info: %d, Err: %d\n",
    //               static_cast<int>(parm_->armCmd.set_angle_Roll), static_cast<int>(parm_->armInfo.angle_Roll),
    //               static_cast<int>(parm_->armInfo.angle_Roll - parm_->armCmd.set_angle_Roll));
    //         Print("Arm_EndPitch_Cmd: %d, Info: %d, Err: %d\n",
    //               static_cast<int>(parm_->armCmd.set_angle_end_pitch), static_cast<int>(parm_->armInfo.angle_end_pitch),
    //               static_cast<int>(parm_->armInfo.angle_end_pitch - parm_->armCmd.set_angle_end_pitch));
    //         Print("Arm_EndRoll_Cmd: %d, Info: %d, Err: %d\n",
    //               static_cast<int>(parm_->armCmd.set_angle_end_roll), static_cast<int>(parm_->armInfo.angle_end_roll),
    //               static_cast<int>(parm_->armInfo.angle_end_roll - parm_->armCmd.set_angle_end_roll));
    //     }

    // }

    bool zx = SysRemote.remoteInfo.keyboard.key_Z && SysRemote.remoteInfo.keyboard.key_X; ///< 如果同时按下z和x
    if (SysRemote.remoteInfo.keyboard.key_Z && SysRemote.remoteInfo.keyboard.key_X) {
        zx_count++;
    }
    // 全部松开之后才清零计数器，否则会因为按下状态的抖动导致自定义控制器模式连续进出
    else if ((SysRemote.remoteInfo.keyboard.key_Z || SysRemote.remoteInfo.keyboard.key_X) == false) {
        zx_count = 0;
        zx_flag = false;
    }

    if (zx_count > 20 && zx_flag == false) {
        zx_flag = true;
        zx_count = 0;
        if (!use_Controller_ && !SysControllerLink.IsControllerOnline()) {
            // 自定义控制器不在线同时不是自定义控制器控制的时候无法切换
            pgimbal_->gimbalInfo.isIntoControll = false;//切换出来清空云台标志位
        } else {
            use_Controller_ = !use_Controller_;
        }
    }
    if (use_Controller_ != last_use_Controller) {
        if (use_Controller_ == true) {
            SysControllerLink.robotInfo.controlled_by_controller = true;
            if (parm_) {
                auto &armCmd = parm_->armCmd;
                const auto &armInfo = parm_->armInfo;
                armCmd.isCustomCtrl = true; 
                armCmd.set_angle_Yaw = armInfo.angle_Yaw;
                armCmd.set_angle_Pitch1 = armInfo.angle_Pitch1;
                armCmd.set_angle_Pitch2 = armInfo.angle_Pitch2;
                armCmd.set_angle_Pitch3 = armInfo.angle_Pitch3;
                armCmd.set_angle_Roll = armInfo.angle_Roll;
                armCmd.set_angle_end_pitch = armInfo.angle_end_pitch;
                armCmd.set_angle_end_roll = armInfo.angle_end_roll;
                if(armInfo.isGripped){
                    armCmd.set_length_grip = armInfo.holdLength_grip;
                    gripKeyboardCmd_ = EGripKeyboardCmd::CLOSE;
                    armCmd.gripClose = true;
                    armCmd.gripOpen = false;
                }else{
                    armCmd.set_length_grip = armInfo.length_grip;  ///< 保存当前夹爪位置，防止切换后意外张开
                    gripKeyboardCmd_ = EGripKeyboardCmd::HOLD;
                    armCmd.gripClose = false;
                    armCmd.gripOpen = false;
                }
            }
            // 自动任务部分
            // if (currentAutoCtrlProcess_ == EAutoCtrlProcess::EXCHANGE_ORE) {
    
            // }
            // else {
            //     StopAutoCtrlTask_(); // 停止自动任务运行
            //    // 根据当前在哪个自动任务中调整臂的初始角度
            // // SysControllerLink.robotInfo.controlled_by_controller = true;
            // // SysControllerLink.robotInfo.ask_reset_flag = true;
            // }
        }
        if (use_Controller_ == false) { //切换出自定义控制器的瞬间保留最后一帧数值避免后续出现大幅跳变，且切换出自定义控制器模式后不再受控制器输入影响
            SysControllerLink.robotInfo.controlled_by_controller = false;
            if (parm_) {
                auto &armCmd = parm_->armCmd;
                const auto &armInfo = parm_->armInfo;
                armCmd.isCustomCtrl = false;
                armCmd.set_angle_Yaw = armInfo.angle_Yaw;
                armCmd.set_angle_Pitch1 = armInfo.angle_Pitch1;
                armCmd.set_angle_Pitch2 = armInfo.angle_Pitch2;
                armCmd.set_angle_Pitch3 = armInfo.angle_Pitch3;
                armCmd.set_angle_Roll = armInfo.angle_Roll;
                armCmd.set_angle_end_pitch = armInfo.angle_end_pitch;
                armCmd.set_angle_end_roll = armInfo.angle_end_roll;
                if(armInfo.isGripped){
                    armCmd.set_length_grip = armInfo.holdLength_grip;
                    gripKeyboardCmd_ = EGripKeyboardCmd::CLOSE; 
                    armCmd.gripClose = true;
                    armCmd.gripOpen = false;
                }else{
                    armCmd.set_length_grip = armInfo.length_grip;
                    gripKeyboardCmd_ = EGripKeyboardCmd::HOLD;
                    armCmd.gripClose = false;
                    armCmd.gripOpen = false;
                }
            }
            if (pgimbal_) {
                pgimbal_->gimbalInfo.isIntoControll = false; ///< 清除云台归位标志，下次进入时重新归位
            }
        }
    }
    if (parm_) {   //反馈给控制器的数据
        // 角度
            SysControllerLink.robotInfo.arm.yaw       = parm_->armInfo.angle_Yaw;
            SysControllerLink.robotInfo.arm.pitch1     = parm_->armInfo.angle_Pitch1;
            SysControllerLink.robotInfo.arm.pitch2     = parm_->armInfo.angle_Pitch2;
            SysControllerLink.robotInfo.arm.pitch3     = parm_->armInfo.angle_Pitch3;
            SysControllerLink.robotInfo.arm.roll       = parm_->armInfo.angle_Roll;
            SysControllerLink.robotInfo.arm.pitch_end  = parm_->armInfo.angle_end_pitch;

            // 力矩/电流：直接传原始值，避免转物理量后被int16截断为0
            SysControllerLink.robotInfo.torque.yaw       = static_cast<float>(parm_->comjoint_.motor[CModArm::CComJoint::Y]->motorData[CDevMtr::DATA_CURRENT]);
            SysControllerLink.robotInfo.torque.pitch1    = static_cast<float>(parm_->comjoint_.motor[CModArm::CComJoint::P1]->motorData[CDevMtr::DATA_CURRENT]);
            SysControllerLink.robotInfo.torque.pitch2    = static_cast<float>(parm_->comjoint_.motor[CModArm::CComJoint::P2]->motorData[CDevMtr::DATA_CURRENT]);
            SysControllerLink.robotInfo.torque.pitch3    = static_cast<float>(parm_->comjoint_.motor[CModArm::CComJoint::P3]->motorData[CDevMtr::DATA_CURRENT]);
            SysControllerLink.robotInfo.torque.roll      = static_cast<float>(parm_->comRoll_.motor->motorData[CDevMtr::DATA_TORQUE]);
            SysControllerLink.robotInfo.torque.pitch_end = 0.f;  // 末端由双M2006差速驱动，暂不处理
        }
    SysControllerLink.robotInfo.robot_init_ok =
        (parm_ && parm_->armInfo.isModuleAvailable) && (SysRemote.systemStatus == APP_OK);

        // if (use_Controller_ == false) {
        //     // StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_DRIVE); // 已删除此自动流程
        //     // TODO: 决定切换出自定义控制器模式后的行为
        // }
    
    ControlFromEsp32_(); // ESP32控制

    // 每周期重置手动夹爪标志，由对应控制函数按需设置
    if (parm_ && !parm_->armCmd.isAutoCtrl) {
        parm_->armCmd.gripClose = false;
        parm_->armCmd.gripOpen = false;
        parm_->armCmd.set_speed_grip = 0.0f;
    }

    if (use_Controller_ == true){//在不主动切换模式的情况下，如果控制器掉线自动退出控制器模式
        // 控制器掉线保护
        if (!SysControllerLink.IsControllerOnline()) {
            use_Controller_ = false;
            SysControllerLink.robotInfo.controlled_by_controller = false;//控制器被反向控制
            if (parm_) { // 同理自动保存最后一帧的数据
                parm_->armCmd.isCustomCtrl = false;
                parm_->armCmd.set_angle_Yaw = parm_->armInfo.angle_Yaw;
                parm_->armCmd.set_angle_Pitch1 = parm_->armInfo.angle_Pitch1;
                parm_->armCmd.set_angle_Pitch2 = parm_->armInfo.angle_Pitch2;
                parm_->armCmd.set_angle_Pitch3 = parm_->armInfo.angle_Pitch3;
                parm_->armCmd.set_angle_Roll = parm_->armInfo.angle_Roll;
                parm_->armCmd.set_angle_end_pitch = parm_->armInfo.angle_end_pitch;
                parm_->armCmd.set_angle_end_roll = parm_->armInfo.angle_end_roll;
                parm_->armCmd.set_length_grip = parm_->armInfo.length_grip;  ///< 保存当前夹爪位置
            }
            if (pgimbal_) {
                pgimbal_->gimbalInfo.isIntoControll = false; ///< 控制器掉线也清除云台归位标志
            }
        } else {
            ControlFromController_();
            ctrlmode_ = ECtrlMode::CONTROLLER_CTRL; ///< 自定义控制器控制
        }
    }
    else
    {
        // 左下右上键盘控制
        if (SysRemote.remoteInfo.remote.switch_L == 2
        && SysRemote.remoteInfo.remote.switch_R == 1)
        {
            ControlFromKeyboard_();
            ctrlmode_ = ECtrlMode::KEY_CTRL; ///< 键鼠控制
        }
        else ///< 其他情况均为遥控器控制
        {
            ControlFromRemote_();
            ctrlmode_ = ECtrlMode::RC_CTRL; ///< 遥控器控制
        }
    }

    BoardLink_Info_Update_(); ///< 更新板间通信数据包
    
    last_use_Controller = use_Controller_;
    
    // if (SysRemote.ResetFlag)
    // {
    //     RESET_SYSTEM();
    // }

}

/**
 * @brief 心跳处理
 * 
 */
void CSystemCore::HeartbeatHandler_() {
    // 检查系统核心状态
    if (coreStatus == APP_RESET) return;

    static auto lastRemoteState = APP_RESET;
    auto currentRemoteState = SysRemote.systemStatus;

    // 遥控器掉线
    if (lastRemoteState == APP_OK && currentRemoteState != APP_OK) {

        use_Controller_ = false;
        // 停止所有自动操作
        StopAutoCtrlTask_();
        
        // 停止所有模块（添加空指针检查）
        if (pchassis_) pchassis_->StopModule();
        if (parm_) parm_->StopModule();
        if (pgimbal_) pgimbal_->StopModule();
        
    }

    lastRemoteState = currentRemoteState;
}

// 软件复位
void CSystemCore::RESET_SYSTEM() {

    if (pchassis_) pchassis_->StopModule();
    if (parm_) parm_->StopModule();
    if (pgimbal_) pgimbal_->StopModule();

    // 给段延迟让电机收到停止指令
    static uint16_t resetCnt = 200;
    if (resetCnt-- != 0) {
        return ;
    }

    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

/**
 * @brief 启动自动任务
 * 
 * @retval EAppStatus
 */
EAppStatus CSystemCore::StartAutoCtrlTask_(EAutoCtrlProcess process) {

    if (currentAutoCtrlProcess_ != EAutoCtrlProcess::NONE)
        return APP_BUSY;

    StopAutoCtrlTask_();    ///< 停止当前任务

    // 设置机械臂yaw轴限位（添加空指针检查）
    if (parm_) {
        parm_->should_limit_yaw = 0;
    }

    // if(!parm_->armInfo.isModuleAvailable){
    //     return APP_ERROR;
    // }

    switch (process)
    {
        case EAutoCtrlProcess::NONE: {
            return APP_ERROR;
        }

       case EAutoCtrlProcess::RETURN_ORIGIN: {
           currentAutoCtrlProcess_ = EAutoCtrlProcess::RETURN_ORIGIN;
           xTaskCreate(StartReturnOriginTask, "Return Origin Task",
                       512, this, proc_ModuleTaskPriority,
                       &autoCtrlTaskHandle_);
           return APP_OK;
       }

       case EAutoCtrlProcess::CLIMBING: {
           currentAutoCtrlProcess_ = EAutoCtrlProcess::CLIMBING;
           xTaskCreate(StartClimbingTask, "Climbing Task",
                       512, this, proc_ModuleTaskPriority,
                       &autoCtrlTaskHandle_);
           return APP_OK;
       }

       case EAutoCtrlProcess::DOWN_STAIR: {
           currentAutoCtrlProcess_ = EAutoCtrlProcess::DOWN_STAIR;
           xTaskCreate(StartDownStairTask, "DownStairs Task",
                       512, this, proc_ModuleTaskPriority,
                       &autoCtrlTaskHandle_);
           return APP_OK;
       }

       case EAutoCtrlProcess::STORE_ORE: {
        currentAutoCtrlProcess_ = EAutoCtrlProcess::STORE_ORE;
        xTaskCreate(StartStoreTask, "Save Ore Task",
                    512, this, proc_ModuleTaskPriority,
                    &autoCtrlTaskHandle_);
        return APP_OK;
        }

        case EAutoCtrlProcess::EXCHANGE_ORE : {
        currentAutoCtrlProcess_ = EAutoCtrlProcess::EXCHANGE_ORE;
        xTaskCreate(StartExchangeGetTask, "Exchange Ore Task",
                    512, this, proc_ModuleTaskPriority,
                    &autoCtrlTaskHandle_);
        return APP_OK;
        }

//        case EAutoCtrlProcess::ENERGY_UNIT: {
//            currentAutoCtrlProcess_ = EAutoCtrlProcess::ENERGY_UNIT;
//            xTaskCreate(StartEnergyUnitTask, "Grab Energy Unit Task",
//                        512, this, proc_ModuleTaskPriority,
//                        &autoCtrlTaskHandle_);
//            return APP_OK;
//        }

//        case EAutoCtrlProcess::EXCHANGE_ORE: {
//            currentAutoCtrlProcess_ = EAutoCtrlProcess::EXCHANGE_ORE;
//            xTaskCreate(StartExchangeOreTask, "Exchange Ore Task",
//                        512, this, proc_ModuleTaskPriority,
//                        &autoCtrlTaskHandle_);
//            return APP_OK;
//        }

//        case EAutoCtrlProcess::STORE_ORE: {
//            currentAutoCtrlProcess_ = EAutoCtrlProcess::STORE_ORE;
//            xTaskCreate(StartSaveOreTask, "Save Ore Task",
//                        512, this, proc_ModuleTaskPriority,
//                        &autoCtrlTaskHandle_);
//            return APP_OK;
//        }

//        case EAutoCtrlProcess::GROUND_ORE: {
//            currentAutoCtrlProcess_ = EAutoCtrlProcess::GROUND_ORE;
//            xTaskCreate(StartGroundOreTask, "Ground Ore Task",
//                        512, this, proc_ModuleTaskPriority,
//                        &autoCtrlTaskHandle_);
//            return APP_OK;
//        }

        default: return APP_ERROR;
    }
}

/**
 * @brief 停止自动任务
 * 
 * @retval EAppStatus
 */
EAppStatus CSystemCore::StopAutoCtrlTask_() {

    if (autoCtrlTaskHandle_ != nullptr) {
        vTaskDelete(autoCtrlTaskHandle_);
        autoCtrlTaskHandle_ = nullptr;
    }

    currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

    // 清除所有模块的自动控制标志（添加空指针检查）
    if (pchassis_) pchassis_->chassisCmd.isAutoCtrl = false;
    if (parm_) parm_->armCmd.isAutoCtrl = false;
    if (pgimbal_) pgimbal_->gimbalCmd.isAutoCtrl = false;

    return APP_OK;
}

/**
 * @brief 更新板通系统层
 * 
 * @retval null
 */
void CSystemCore::BoardLink_Info_Update_(){
    // 检查系统核心状态
    if (coreStatus == APP_RESET) return;

    // 包0数据更新
    SysBoardLink.ctrlFlags.pack_id = 0;
    SysBoardLink.ctrlFlags.rc_status = SysRemote.systemStatus;
    SysBoardLink.ctrlFlags.ctrl_mode = static_cast<uint8_t>(ctrlmode_);
    SysBoardLink.ctrlFlags.move_mode = static_cast<uint8_t>(movemode_);

    SysBoardLink.ctrlFlags.chassis_auto_ctrl = pchassis_->chassisCmd.isAutoCtrl;
    SysBoardLink.ctrlFlags.gimbal_auto_ctrl = gimbal_auto_ctrl;
    SysBoardLink.ctrlFlags.arm_auto_ctrl = parm_->armCmd.isAutoCtrl;

    // // 包1 - 控制器左臂后三轴命令包
    // SysBoardLink.controllerbackcmd_l.pack_id = 1;
    // SysBoardLink.controllerbackcmd_l.yaw = SysControllerLink.controllerInfo.left_arm.yaw;          ///< 左臂yaw
    // SysBoardLink.controllerbackcmd_l.pitch1 = SysControllerLink.controllerInfo.left_arm.pitch1;    ///< 左臂pitch1
    // SysBoardLink.controllerbackcmd_l.pitch2 = SysControllerLink.controllerInfo.left_arm.pitch2;    ///< 左臂pitch2

    // // 包2 - 控制器左臂前三轴命令包
    // SysBoardLink.controllerfrontcmd_l.pack_id = 2;
    // SysBoardLink.controllerfrontcmd_l.roll = SysControllerLink.controllerInfo.left_arm.roll;              ///< 左臂roll
    // SysBoardLink.controllerfrontcmd_l.pitch_end = SysControllerLink.controllerInfo.left_arm.pitch_end;    ///< 左臂末端pitch
    // SysBoardLink.controllerfrontcmd_l.roll_end = SysControllerLink.controllerInfo.rocker_LX;              ///< 左臂末端roll(增量式)
    // SysBoardLink.controllerfrontcmd_l.grip_close = SysControllerLink.controllerInfo.gripper_left_close;  ///< 左臂夹爪开合
    // SysBoardLink.controllerfrontcmd_l.chassis_speed = SysControllerLink.controllerInfo.rocker_RY;           ///< 底盘速度

}


}   // namespace my_engineer
