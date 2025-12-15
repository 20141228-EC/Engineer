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
    if (SysRemote.systemStatus == APP_RESET) return;

    static bool last_use_Controller = false;
    static uint8_t zx_count = 0;
    static bool zx_flag = false;

    static uint8_t print_cnt = 0;
    if (print_cnt-- == 0) {
        print_cnt = 200;
        Print("------------------------------\n");
        if (parm_) {
            Print("Arm_Yaw_Cmd: %d, Info: %d, Err: %d\n",
                  static_cast<int>(parm_->armCmd.set_angle_Yaw), static_cast<int>(parm_->armInfo.angle_Yaw),
                  static_cast<int>(parm_->armInfo.angle_Yaw - parm_->armCmd.set_angle_Yaw));
            Print("Arm_Pitch1_Cmd: %d, Info: %d, Err: %d\n",
                  static_cast<int>(parm_->armCmd.set_angle_Pitch1), static_cast<int>(parm_->armInfo.angle_Pitch1),
                  static_cast<int>(parm_->armInfo.angle_Pitch1 - parm_->armCmd.set_angle_Pitch1));
            Print("Arm_Pitch2_Cmd: %d, Info: %d, Err: %d\n",
                  static_cast<int>(parm_->armCmd.set_angle_Pitch2), static_cast<int>(parm_->armInfo.angle_Pitch2),
                  static_cast<int>(parm_->armInfo.angle_Pitch2 - parm_->armCmd.set_angle_Pitch2));
            Print("Arm_Roll_Cmd: %d, Info: %d, Err: %d\n",
                  static_cast<int>(parm_->armCmd.set_angle_Roll), static_cast<int>(parm_->armInfo.angle_Roll),
                  static_cast<int>(parm_->armInfo.angle_Roll - parm_->armCmd.set_angle_Roll));
            Print("Arm_EndPitch_Cmd: %d, Info: %d, Err: %d\n",
                  static_cast<int>(parm_->armCmd.set_angle_end_pitch), static_cast<int>(parm_->armInfo.angle_end_pitch),
                  static_cast<int>(parm_->armInfo.angle_end_pitch - parm_->armCmd.set_angle_end_pitch));
            Print("Arm_EndRoll_Cmd: %d, Info: %d, Err: %d\n",
                  static_cast<int>(parm_->armCmd.set_angle_end_roll), static_cast<int>(parm_->armInfo.angle_end_roll),
                  static_cast<int>(parm_->armInfo.angle_end_roll - parm_->armCmd.set_angle_end_roll));
        }

    }

    bool zx = SysRemote.remoteInfo.keyboard.key_Z && SysRemote.remoteInfo.keyboard.key_X; ///< 如果同时按下z和x
    if (SysRemote.remoteInfo.keyboard.key_Z && SysRemote.remoteInfo.keyboard.key_X) {
        zx_count++;
    }
    // 全部松开之后才清零计数器，否则会因为按下状态的抖动导致自定义控制器模式连续进出
    else if (SysRemote.remoteInfo.keyboard.key_Z || SysRemote.remoteInfo.keyboard.key_X == false) {
        zx_count = 0;
        zx_flag = false;
    }

    if (zx_count > 20 && zx_flag == false) {
        zx_flag = true;
        zx_count = 0;
        use_Controller_ = !use_Controller_;
        if (use_Controller_ == true) {
            // 根据当前在哪个自动任务中调整臂的初始角度
            if (currentAutoCtrlProcess_ == EAutoCtrlProcess::EXCHANGE_ORE) {

            }
            else {

            }
            SysControllerLink.robotInfo.controlled_by_controller = true;
            SysControllerLink.robotInfo.ask_return_flag = true;
            StopAutoCtrlTask_(); // 停止自动任务运行
        }
        if (use_Controller_ == false) {
            // StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_DRIVE); // 已删除此自动流程
            // TODO: 决定切换出自定义控制器模式后的行为
        }
    }
    
    ControlFromEsp32_(); // ESP32控制

    if (use_Controller_ == true)
    {
        ControlFromController_();
        ctrlmode_ = ECtrlMode::CONTROLLER_CTRL; ///< 自定义控制器控制
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
    
    if (SysRemote.ResetFlag)
    {
        RESET_SYSTEM();
    }

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
        // 停止所有自动操作
        // StopAutoCtrlTask_();
        
        // 停止所有模块（添加空指针检查）
        if (pchassis_) pchassis_->StopModule();
        // if (psubgantry_) psubgantry_->StopModule(); // 已删除
        if (parm_) parm_->StopModule();
        
    }

    lastRemoteState = currentRemoteState;
}

// 软件复位
void CSystemCore::RESET_SYSTEM() {

    if (pchassis_) pchassis_->StopModule();
    // if (psubgantry_) psubgantry_->StopModule(); // 已删除
    if (parm_) parm_->StopModule();

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

    switch (process)
    {
        case EAutoCtrlProcess::NONE: {
            return APP_ERROR;
        }

        // 删除涉及到子龙门的自动流程，对应的流程文件已备份至 process_subgantry_backup

        // case EAutoCtrlProcess::EXCHANGE: {
        //     currentAutoCtrlProcess_ = EAutoCtrlProcess::EXCHANGE;
        //     xTaskCreate(StartExchangeTask, "Exchange Task",
        //                 512, this, proc_ModuleTaskPriority,
        //                 &autoCtrlTaskHandle_);
        //     return APP_OK;
        // }

        // case EAutoCtrlProcess::RETURN_DRIVE: {
        //     currentAutoCtrlProcess_ = EAutoCtrlProcess::RETURN_DRIVE;
        //     xTaskCreate(StartReturnDriveTask, "Return Drive Task",
        //                 512, this, proc_ModuleTaskPriority,
        //                 &autoCtrlTaskHandle_);
        //     return APP_OK;
        // }

        // case EAutoCtrlProcess::RETURN_ORIGIN: {
        //     currentAutoCtrlProcess_ = EAutoCtrlProcess::RETURN_ORIGIN;
        //     xTaskCreate(StartReturnOriginTask, "Return Origin Task",
        //                 512, this, proc_ModuleTaskPriority,
        //                 &autoCtrlTaskHandle_);
        //     return APP_OK;
        // }

        // case EAutoCtrlProcess::DOGHOLE: {
        //     currentAutoCtrlProcess_ = EAutoCtrlProcess::DOGHOLE;
        //     xTaskCreate(StartDogHoleTask, "Dog Hole Task",
        //                 512, this, proc_ModuleTaskPriority,
        //                 &autoCtrlTaskHandle_);
        //     return APP_OK;
        // }

        // case EAutoCtrlProcess::GROUND_ORE: {
        //     currentAutoCtrlProcess_ = EAutoCtrlProcess::GROUND_ORE;
        //     xTaskCreate(StartGroundOreTask, "Ground Ore Task",
        //                 512, this, proc_ModuleTaskPriority,
        //                 &autoCtrlTaskHandle_);
        //     return APP_OK;
        // }

        // case EAutoCtrlProcess::GOLD_ORE: {
        //     currentAutoCtrlProcess_ = EAutoCtrlProcess::GOLD_ORE;
        //     xTaskCreate(StartGoldOreTask, "Gold Ore Task",
        //                 512, this, proc_ModuleTaskPriority,
        //                 &autoCtrlTaskHandle_);
        //     return APP_OK;
        // }

        // case EAutoCtrlProcess::PUSH_ORE: {
        //     currentAutoCtrlProcess_ = EAutoCtrlProcess::PUSH_ORE;
        //     xTaskCreate(StartPushOreTask, "Push Ore Task",
        //                 512, this, proc_ModuleTaskPriority,
        //                 &autoCtrlTaskHandle_);
        //     return APP_OK;
        // }

        // case EAutoCtrlProcess::POP_ORE: {
        //     currentAutoCtrlProcess_ = EAutoCtrlProcess::POP_ORE;
        //     xTaskCreate(StartPopOreTask, "Pop Ore Task",
        //                 512, this, proc_ModuleTaskPriority,
        //                 &autoCtrlTaskHandle_);
        //     return APP_OK;
        // }

        // case EAutoCtrlProcess::SILVER_ORE: {
        //     currentAutoCtrlProcess_ = EAutoCtrlProcess::SILVER_ORE;
        //     xTaskCreate(StartSilverOreTask, "Silver Ore Task",
        //                 512, this, proc_ModuleTaskPriority,
        //                 &autoCtrlTaskHandle_);
        //     return APP_OK;
        // }

        default: return APP_ERROR;
    }
}

/**
 * @brief 停止自动任务
 * 
 * @retval EAppStatus
 */
EAppStatus CSystemCore::StopAutoCtrlTask_() {
    if (autoCtrlTaskHandle_ == nullptr) return APP_ERROR;

    vTaskDelete(autoCtrlTaskHandle_);
    autoCtrlTaskHandle_ = nullptr;
    currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

    // 清除所有模块的自动控制标志（添加空指针检查）
    if (pchassis_) pchassis_->chassisCmd.isAutoCtrl = false;
    if (parm_) parm_->armCmd.isAutoCtrl = false;

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
    SysBoardLink.remoteInfo1.pack_id = 0;
    SysBoardLink.remoteInfo1.joystick_RX = SysRemote.remoteInfo.remote.joystick_RX * 220; ///< 右摇杆x
    SysBoardLink.remoteInfo1.joystick_RY = SysRemote.remoteInfo.remote.joystick_RY * 220; ///< 右摇杆y
    SysBoardLink.remoteInfo1.joystick_LX = SysRemote.remoteInfo.remote.joystick_LX * 220; ///< 左摇杆x

    // 包1数据更新
    SysBoardLink.remoteInfo2.pack_id = 1;
    SysBoardLink.remoteInfo2.joystick_LY = SysRemote.remoteInfo.remote.joystick_LY * 220; ///< 左摇杆y
    SysBoardLink.remoteInfo2.thumbWheel = SysRemote.remoteInfo.remote.thumbWheel * 220;    ///< 拨轮
    // 上面这些放大220倍是为了保留两位小数点精度，在保证不超int16_t范围的同时尽可能保证发过去的是原始遥控器数据，副板只需要*3.f再除100.f转浮点数即可获取原始遥控器数据
    SysBoardLink.remoteInfo2.switch_l = SysRemote.remoteInfo.remote.switch_L;   ///< 左拨杆
    SysBoardLink.remoteInfo2.switch_r = SysRemote.remoteInfo.remote.switch_R;   ///< 右拨杆

    // 包2数据更新
    SysBoardLink.ctrlFlags.pack_id = 2;
    SysBoardLink.ctrlFlags.rc_status = SysRemote.systemStatus;
    SysBoardLink.ctrlFlags.ctrl_mode = static_cast<uint8_t>(ctrlmode_);
    SysBoardLink.ctrlFlags.move_mode = static_cast<uint8_t>(movemode_);

    SysBoardLink.ctrlFlags.chassis_auto_ctrl = pchassis_->chassisCmd.isAutoCtrl;
    SysBoardLink.ctrlFlags.gimbal_auto_ctrl = gimbal_auto_ctrl;
    SysBoardLink.ctrlFlags.arm_auto_ctrl = parm_->armCmd.isAutoCtrl;

}


}   // namespace my_engineer
