/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-05-14 01:29:28
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-16 22:44:04
 */

#include "Core.hpp"

namespace my_engineer {

// 实例化系统核心
CSystemCore SystemCore;

/*----------- Debug: 力反馈调试 -----------*/
volatile float dbg_fb_yaw    = 0;  ///< 转换后力矩 (N·m)
volatile float dbg_fb_pitch1 = 0;
volatile float dbg_fb_pitch2 = 0;
volatile float dbg_fb_roll   = 0;
volatile float dbg_raw_yaw    = 0;  ///< 主控传来的原始值
volatile float dbg_raw_pitch1 = 0;
volatile float dbg_raw_pitch2 = 0;
volatile float dbg_raw_roll   = 0;

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

    // 启动BMI088设备
    DeviceIDMap.at(EDeviceID::DEV_MEMS_BMI088)->StartDevice();

    // 初始化所有系统
    CSystemControllerLink::SSystemInitParam_ControllerLink controllerLinkInitParam;
    controllerLinkInitParam.systemID = ESystemID::SYS_CONTROLLERLINK;
    controllerLinkInitParam.controllerLinkDevID = EDeviceID::DEV_CONTROLLER_LINK;
    SysControllerLink.InitSystem(&controllerLinkInitParam);

    // 单臂控制器
    auto it = ModuleIDMap.find(EModuleID::MOD_CONTROLLER);
    if (it != ModuleIDMap.end() && it->second != nullptr) {
        pcontroller_ = reinterpret_cast<CModController *>(it->second);
    }

    proc_waitMs(1000); // 等待系统初始化完成

    coreStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 更新处理
 * 
 */
void CSystemCore::UpdateHandler_() {

    /*------------ 单臂控制器 -----------*/
    if (pcontroller_) {
        // 启动模块
        if (!pcontroller_->ControllerInfo.isModuleAvailable
            && pcontroller_->moduleStatus == APP_OK) {
            pcontroller_->StartModule();
        }
        // 上传臂部数据 (控制器 -> 机器人)，pitch1/2减去零点标定偏移
        SysControllerLink.controllerInfo.arm.yaw       = pcontroller_->ControllerInfo.posit_yaw;
        SysControllerLink.controllerInfo.arm.pitch1     = pcontroller_->ControllerInfo.posit_pitch1 - CONTROLLER_PITCH1_ZERO_OFFSET;
        SysControllerLink.controllerInfo.arm.pitch2     = pcontroller_->ControllerInfo.posit_pitch2 - CONTROLLER_PITCH2_ZERO_OFFSET;
        SysControllerLink.controllerInfo.arm.roll       = pcontroller_->ControllerInfo.posit_roll;       // 保留原右臂符号反转
        SysControllerLink.controllerInfo.arm.pitch_end  = pcontroller_->ControllerInfo.posit_pitch_end;  // 保留原右臂符号反转
        // 接收机器人臂部位置 (机器人 -> 控制器)
        pcontroller_->ControllerCmd.cmd_yaw       = -SysControllerLink.robotInfo.arm.yaw;
        pcontroller_->ControllerCmd.cmd_pitch1    = SysControllerLink.robotInfo.arm.pitch1;
        pcontroller_->ControllerCmd.cmd_pitch2    = SysControllerLink.robotInfo.arm.pitch2;
        pcontroller_->ControllerCmd.cmd_roll      = -SysControllerLink.robotInfo.arm.roll;
        pcontroller_->ControllerCmd.cmd_pitch_end = SysControllerLink.robotInfo.arm.pitch_end;

        /* 模式切换
           controlled_by_controller = true   自定义控制器模式：控制器示教，机器人跟随
           controlled_by_controller = false  反向映射模式：控制器跟随机器人位置
        */
        if (SysControllerLink.robotInfo.robot_init_ok) {
            pcontroller_->ControllerInfo.isRobotInit = SysControllerLink.robotInfo.robot_init_ok;
        // 接收机器人力矩反馈 (机器人 -> 控制器，原始值转物理力矩 N·m)
        // KT电机: raw_iq 范围 ±2048 对应 ±33A, 扭矩常数 0.175 N·m/A (24V)，这个计算有点抽象虽然数据是正确的。后续有时间考究一下这个公式为什么有点奇怪。
        // torque = raw_iq × (33.0/2048.0) × 0.175
            constexpr float MG6012_i36v3_KT_RAW_TO_TORQUE = (33.0f / 2048.0f) / 0.175f;  
            constexpr float MG8010_i8v3_KT_RAW_TO_TORQUE = (33.0f / 2048.0f) / 1.09f;  
            constexpr float MG5010_i36v3_KT_RAW_TO_TORQUE = (33.0f / 2048.0f) * 0.3f; // 保留定义（机器人端可能用到）
            pcontroller_->ControllerCmd.fb_torque_yaw       = SysControllerLink.robotInfo.torque.yaw    * MG8010_i8v3_KT_RAW_TO_TORQUE;
            pcontroller_->ControllerCmd.fb_torque_pitch1    = SysControllerLink.robotInfo.torque.pitch1 * MG6012_i36v3_KT_RAW_TO_TORQUE;
            pcontroller_->ControllerCmd.fb_torque_pitch2    = SysControllerLink.robotInfo.torque.pitch2 * MG6012_i36v3_KT_RAW_TO_TORQUE;

            // DM4310 MIT: 12bit原始值(0~4095) -> 物理力矩(-TAU_MAX ~ +TAU_MAX), TAU_MAX=10.0 N·m
            constexpr float DM_TAU_MAX = 10.0f;
            pcontroller_->ControllerCmd.fb_torque_roll      = SysControllerLink.robotInfo.torque.roll;
            pcontroller_->ControllerCmd.fb_torque_pitch_end = 0.f;  // 末端暂不处理

            // 电流的原始值
            dbg_raw_yaw    = SysControllerLink.robotInfo.torque.yaw;
            dbg_raw_pitch1 = SysControllerLink.robotInfo.torque.pitch1;
            dbg_raw_pitch2 = SysControllerLink.robotInfo.torque.pitch2;
            dbg_raw_roll   = SysControllerLink.robotInfo.torque.roll;
            //  扭矩的转换值
            dbg_fb_yaw    = pcontroller_->ControllerCmd.fb_torque_yaw;
            dbg_fb_pitch1 = pcontroller_->ControllerCmd.fb_torque_pitch1;
            dbg_fb_pitch2 = pcontroller_->ControllerCmd.fb_torque_pitch2;
            dbg_fb_roll   = pcontroller_->ControllerCmd.fb_torque_roll;
            if(SysControllerLink.robotInfo.controlled_by_controller){
                pcontroller_->ControllerCmd.isfirstChange = true;
                pcontroller_->ControllerCmd.isFree = true;
            }
            else{
                pcontroller_->ControllerCmd.isFree = !pcontroller_->ControllerCmd.isfirstChange;//刚上电的时候不要出现反向控制的情况
            }
        } else {
            // 机器人未就绪（遥控器掉线）-> 控制器下电，回到示教模式
            pcontroller_->ControllerCmd = CModController::SControllerCmd{};//清除残余的力矩数据
            pcontroller_->ControllerCmd.isFree = true;
            pcontroller_->ControllerInfo.isRobotInit = false;
        }
    }

    /*------------ 状态汇总 -----------*/
    SysControllerLink.controllerInfo.controller_OK =
        pcontroller_ ? pcontroller_->ControllerInfo.isModuleAvailable : true;
    SysControllerLink.controllerInfo.return_success =
        pcontroller_ ? pcontroller_->ControllerInfo.isReturnSuccess : true;

    // 复位检测
    if (SysControllerLink.robotInfo.ask_reset_flag) {
        RESET_SYSTEM();
    }
}

/**
 * @brief 心跳处理
 * 
 */
void CSystemCore::HeartbeatHandler_() {
    
}

// 复位
void CSystemCore::RESET_SYSTEM() {

    
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

}   // namespace my_engineer
