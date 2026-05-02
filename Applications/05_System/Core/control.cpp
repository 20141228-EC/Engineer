/**
 * @file control.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 在这里定义遥控器和键盘的操作函数
 * @version 1.0
 * @date 2026-04-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "Core.hpp"

namespace my_engineer {

void CSystemCore::StartRobot(bool if_remote_control, bool I_dont_have_a_remote) {

    if (pgimbal_) {
        if (!pgimbal_->gimbalInfo.isModuleAvailable
            && pgimbal_->moduleStatus == APP_OK) {
            pgimbal_->StartModule();
        }
    }

    if(if_remote_control) {
        enum { HIG = 1, LOW = 2, MID = 3 };
        auto &remote = SysRemote.remoteInfo.remote;

        static uint8_t last_switch_L = LOW; // 遥控器系统层的数据更新比设备层的状态更新要慢

        if (parm_) {
            if (!parm_->armInfo.isModuleAvailable
                && parm_->moduleStatus == APP_OK
                && remote.switch_L == HIG && last_switch_L != HIG) {
                parm_->StartModule();
            }
        }
        last_switch_L = remote.switch_L;        ///<记录上一次的拨杆状态

    }

    else {
        auto &keyboard = SysRemote.remoteInfo.keyboard;

        if (parm_) {
            if (!parm_->armInfo.isModuleAvailable
                && parm_->moduleStatus == APP_OK
                && keyboard.key_Ctrl && keyboard.key_R) {
                parm_->StartModule();
            }
        }
/* 删除子龙门模块键盘启动代码
        if (psubgantry_) {
            if (!psubgantry_->subGantryInfo.isModuleAvailable
                && psubgantry_->moduleStatus == APP_OK
                && keyboard.key_Ctrl && keyboard.key_R) {
                psubgantry_->StartModule();
            }
        }
*/
    }

    if(I_dont_have_a_remote) {
        if (parm_) {
            if (!parm_->armInfo.isModuleAvailable
                && parm_->moduleStatus == APP_OK) {
                parm_->StartModule();
            }
        }
/* 删除无遥控器时启动子龙门模块代码
        if (psubgantry_) {
            if (!psubgantry_->subGantryInfo.isModuleAvailable
                && psubgantry_->moduleStatus == APP_OK) {
                psubgantry_->StartModule();
            }
        }
*/
    }
    
}

/**
 * @brief 遥控器操作
 * 
 */
void CSystemCore::ControlFromRemote_() {
    const auto freq = 1000.f; // 系统核心频率

    enum { HIG = 1, LOW = 2, MID = 3 };
    auto &remote = SysRemote.remoteInfo.remote;
    auto &remote_edge = SysRemote.remoteInfo.remote_edge;

    //将模块启动
    if (SysRemote.systemStatus == APP_OK) {
        StartRobot(true);                   ///<因为键盘的默认参数是false
    }

    //用于调试，免去遥控器上电
    StartRobot(true, true);

    if (parm_) {
        parm_->should_limit_yaw = 0;
    }

    // LOW + MID 底盘控制
    if (remote.switch_L == LOW && remote.switch_R == MID) {
        SysRemote.SetRemoteDeadZone(10.f);
            chassisCmd.speed_x = remote.joystick_LX / 2;
            chassisCmd.speed_y = remote.joystick_LY;
            chassisCmd.speed_w = remote.joystick_RX;
        
    }

    // MID + HIG 机械臂前四轴
    if (remote.switch_L == MID && remote.switch_R == HIG) {
        SysRemote.SetRemoteDeadZone(10.f);
        if (parm_) {
            // parm_->armCmd.set_angle_Yaw +=
            //     (remote.joystick_LX / 100.f) * 90.f / freq;
            // parm_->armCmd.set_angle_Pitch1 +=
            //     (remote.joystick_LY / 100.f) * 90.f / freq;
            // parm_->armCmd.set_angle_Pitch2 +=
            //     (remote.joystick_RY / 100.f) * 90.f / freq;
            // parm_->armCmd.set_angle_Roll +=
            //     (remote.joystick_RX / 100.f) * 90.f / freq;
        }
    }

    // MID + MID 机械臂后四轴
    if (remote.switch_L == MID && remote.switch_R == MID) {
        SysRemote.SetRemoteDeadZone(10.f);
        if (parm_) {
            // parm_->armCmd.set_angle_Pitch2 +=
            //     (remote.joystick_LY / 100.f) * 90.f / freq;
            // parm_->armCmd.set_angle_Roll +=
            //     (remote.joystick_LX / 100.f) * 90.f / freq;
            // parm_->armCmd.set_angle_end_pitch +=
            //     (remote.joystick_RY / 100.f) * 90.f / freq;
            // parm_->armCmd.set_angle_end_roll +=
            //     (remote.joystick_RX / 100.f) * 90.f / freq;
        }
    }

    // MID + LOW 子龙门控制
    if (remote.switch_L == MID && remote.switch_R == LOW) {
        SysRemote.SetRemoteDeadZone(10.f);

    }

    // MID + LOW 云台 + 夹爪控制
    if (remote.switch_L == MID && remote.switch_R == LOW) {
        SysRemote.SetRemoteDeadZone(10.f);
        // if (pgimbal_) {                                                             ///< 云台抬升 (左摇杆Y)
        //     pgimbal_->gimbalCmd.set_posit_lift +=
        //         (remote.joystick_LY / 100.f) * 100.f / freq;
        // }
        // if (parm_) {
        //     parm_->armCmd.set_length_grip +=                                        ///< 夹爪控制：正值张开，负值闭合
        //         (remote.thumbWheel / 100.f) * 150.f / freq;
        //     parm_->armCmd.set_length_grip =
        //         std::clamp(parm_->armCmd.set_length_grip, 0.0f, 65.0f);            ///< 限幅：0~65mm
        // }
    }
}

/**
 * @brief 键盘操作
 * 
 */
void CSystemCore::ControlFromKeyboard_() {
    const auto freq = 1000.f; // 系统核心频率

    auto &keyboard = SysRemote.remoteInfo.keyboard;
    auto &keyboard_edge = SysRemote.remoteInfo.keyboard_edge;

    static bool lastMouseStatus_L = false, lastMouseStatus_R = false;

    // 将模块启动
    if (SysRemote.systemStatus == APP_OK) {
        StartRobot(false);                  ///<转换为键盘操作
    }

    // parm_->should_limit_yaw = 1;

    /******************* 底盘控制 *******************/
    // 平滑更新角速度
        chassisCmd.speed_w = chassisCmd.speed_w +
        0.03f*(keyboard.mouse_X - chassisCmd.speed_w);

        chassisCmd.speed_x *= 0.97f;
        chassisCmd.speed_y *= 0.98f;
        if (abs(chassisCmd.speed_x) < 0.5f) chassisCmd.speed_x = 0.0f;
        if (abs(chassisCmd.speed_y) < 0.5f) chassisCmd.speed_y = 0.0f;

        if (keyboard.key_Shift) {
            chassisCmd.speed_x += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 5.0f;   ///<通过差值来实现一行代码实现左右转弯
            chassisCmd.speed_y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 5.0f;
            chassisCmd.speed_x =
            std::clamp(chassisCmd.speed_x, -50.0f, 50.0f);
            chassisCmd.speed_y =
            std::clamp(chassisCmd.speed_y, -100.0f, 100.0f);
        } else {
            chassisCmd.speed_x += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 1.0f;
            chassisCmd.speed_y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 1.0f;
            chassisCmd.speed_x =
            std::clamp(chassisCmd.speed_x, -20.0f, 20.0f);
            chassisCmd.speed_y =
            std::clamp(chassisCmd.speed_y, -50.0f, 50.0f);
        }    


    // 小陀螺  (G键)
        if (keyboard.key_G
            && currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE) {
            chassisCmd.is_spin_on = true;
        }

    /******************* 云台手动控制 *******************/
    // if (pgimbal_) {
    //     if (!keyboard.key_Ctrl &&
    //         !pgimbal_->gimbalCmd.isAutoCtrl) {
    //         // (F键)
    //         if (keyboard.key_F) {
    //             pgimbal_->gimbalCmd.set_posit_lift += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 120.0f / freq;
    //         }
    //     }
    // }

    /******************* 机械臂手动控制 *******************/
    if (parm_) {
        if (!keyboard.key_Ctrl &&
            !parm_->armCmd.isAutoCtrl) {
            // !SysBoardLink.pArm_Cmd->isAutoCtrl) {
            // yaw(Q键)
            if(keyboard.key_Q)
                parm_->armCmd.set_angle_Yaw += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 60.0f / freq;
            // pitch1(E键)
            if(keyboard.key_E)
                parm_->armCmd.set_angle_Pitch1 += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 70.0f / freq;
            // pitch2(R键)
            if(keyboard.key_R)
                parm_->armCmd.set_angle_Pitch2 += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 70.0f / freq;
            if(keyboard.key_F)
                parm_->armCmd.set_angle_Pitch3 += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 70.0f / freq;
            // roll(Z键)
            if(keyboard.key_Z)
                parm_->armCmd.set_angle_Roll += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 80.0f / freq;
            // end_pitch(X键)
            if(keyboard.key_X)
                parm_->armCmd.set_angle_end_pitch += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 90.0f / freq;
            // end_roll(C键)
            if(keyboard.key_C)
                parm_->armCmd.set_angle_end_roll += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 90.0f / freq;
            // 气泵(B键) 删除气泵控制代码
/*            if (psubgantry_) {
                if(keyboard.key_B) {
                    if (!lastMouseStatus_L && keyboard.mouse_L) {
                        psubgantry_->subGantryCmd.setPumpOn_Arm = !psubgantry_->subGantryCmd.setPumpOn_Arm;
                    }
                    if (!lastMouseStatus_R && keyboard.mouse_R) {
                        if (psubgantry_->subGantryCmd.setPumpOn_Left *
                            psubgantry_->subGantryCmd.setPumpOn_Right == 0) {
                            psubgantry_->subGantryCmd.setPumpOn_Left = true;
                            psubgantry_->subGantryCmd.setPumpOn_Right = true;
                        }
                        else {
                            psubgantry_->subGantryCmd.setPumpOn_Left = !psubgantry_->subGantryCmd.setPumpOn_Left;
                            psubgantry_->subGantryCmd.setPumpOn_Right = !psubgantry_->subGantryCmd.setPumpOn_Right;
                        }

                    }
                }
            }
*/
        }
    }

    lastMouseStatus_L = keyboard.mouse_L;
    lastMouseStatus_R = keyboard.mouse_R;


    /******************* 自动控制 *******************/
    // 删除自动控制快捷键已注释
    // if (parm_) {
    //     if (keyboard.key_Ctrl
    //     && parm_->armInfo.isModuleAvailable)
    //     {
    //         if(keyboard.key_V)
    //         {
    //             //TODO: 这个任务最好可以用于终止proc_waituntil
    //             StopAutoCtrlTask_();
    //             StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_ORIGIN);
    //         }
    //         if(keyboard.key_G)
    //         {
    //             StartAutoCtrlTask_(EAutoCtrlProcess::GOLD_ORE);
    //         }
    //         if(keyboard.key_X)
    //         {
    //             StartAutoCtrlTask_(EAutoCtrlProcess::SILVER_ORE);
    //         }
    //         if(keyboard.key_F)
    //         {
    //             StartAutoCtrlTask_(EAutoCtrlProcess::GROUND_ORE);
    //         }
    //         if(keyboard.key_R)
    //         {
    //             StartAutoCtrlTask_(EAutoCtrlProcess::PUSH_ORE);
    //         }
    //         if(keyboard.key_Q)
    //         {
    //             StartAutoCtrlTask_(EAutoCtrlProcess::POP_ORE);
    //         }
    //         // if(keyboard.key_B)
    //         // {
    //         //     StartAutoCtrlTask_(EAutoCtrlProcess::EXCHANGE);
    //         // }
    //
    //     }
    //     if(keyboard.key_Shift &&
    //         parm_->armInfo.isModuleAvailable){
    //         if(keyboard.key_Z)
    //         {
    //             StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_DRIVE);
    //         }
    //         if(keyboard.key_C)
    //         {
    //             StartAutoCtrlTask_(EAutoCtrlProcess::DOGHOLE);
    //         }
    //     }
    // }

}

/**
 * @brief 自定义控制器操作
 */
void CSystemCore::ControlFromController_() {
    const auto freq = 1000.f; // 系统核心频率
    static uint16_t last_key_F,last_key_G;

    auto &controller = SysControllerLink.controllerInfo;

    auto &keyboard = SysRemote.remoteInfo.keyboard;

    auto &robotdata = SysControllerLink.robotInfo;

    auto &keyboard_edge = SysRemote.remoteInfo.keyboard_edge;

    SysControllerLink.robotInfo.controlled_by_controller = true;

    // 夹爪控制：Core层仅传递标志位，速度渐变由组件层处理
        // 检测是否正在进行模式切换（Z+X同时按住），切换期间冻结夹爪防止意外松开
    bool mode_switching = SysRemote.remoteInfo.keyboard.key_Z
                           && SysRemote.remoteInfo.keyboard.key_X;

    // 线性插值器（25Hz数据  1000Hz控制，周期 = 40步）
    static CAlgoLinearInterp interp_yaw(40), interp_p1(40), interp_p2(40),
                             interp_roll(40), interp_end_pitch(40), interp_p3(40);
    // 上一次控制器原始数据，用于检测数据更新
    static CSystemControllerLink::SArmAngles last_arm;


    // 将模块启动
    if (SysRemote.systemStatus == APP_OK) {
        StartRobot(false);
    }

    if (parm_) {
        parm_->should_limit_yaw = 0;
    }

    /******************* 底盘控制 *******************/
    // 平滑更新角速度
        chassisCmd.speed_w = chassisCmd.speed_w +
        0.03f*(keyboard.mouse_X - chassisCmd.speed_w);

        chassisCmd.speed_x *= 0.97f;
        chassisCmd.speed_y *= 0.98f;
        if (abs(chassisCmd.speed_x) < 0.5f) chassisCmd.speed_x = 0.0f;
        if (abs(chassisCmd.speed_y) < 0.5f) chassisCmd.speed_y = 0.0f;

        if (keyboard.key_Shift) {
            chassisCmd.speed_x += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 5.0f;   ///<通过差值来实现一行代码实现左右转弯
            chassisCmd.speed_y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 5.0f;
            chassisCmd.speed_x =
            std::clamp(chassisCmd.speed_x, -50.0f, 50.0f);
            chassisCmd.speed_y =
            std::clamp(chassisCmd.speed_y, -100.0f, 100.0f);
        } else {
            chassisCmd.speed_x += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 1.0f;
            chassisCmd.speed_y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 1.0f;
            chassisCmd.speed_x =
            std::clamp(chassisCmd.speed_x, -20.0f, 20.0f);
            chassisCmd.speed_y =
            std::clamp(chassisCmd.speed_y, -50.0f, 50.0f);
        }    


    // 小陀螺  (G键)
        if (keyboard.key_G
            && currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE) {
            chassisCmd.is_spin_on = true;
        }


    /******************* 机械臂 *******************/
    // 使用控制器臂部数据控制机械臂
    if (parm_ && !parm_->armCmd.isAutoCtrl) {  ///< 自动控制期间跳过手动控制
        auto &arm = controller.arm;  // 单臂数据

        // float target_yaw = Round(arm.yaw);
        // float target_p1  = Round(arm.pitch1 * 1.102f);  // 88° 对应 97°

        // // Yaw 限位：当 P1 在危险区时，限制目标 Yaw
        // bool in_danger = (parm_->armCmd.set_angle_Pitch1 < 22.0f || target_p1 < 22.0f);
        // if (in_danger && target_yaw > 0.0f) {
        //     target_yaw = (target_yaw < 15.0f) ? 0.0f : 33.0f;
        // }

        // // P2 动态限位：上限随 P1 增大而增大（与 RestrictArmCommand_ 保持一致）
        // float target_p2 = Round(arm.pitch2 * 1.36f);  // 0°~90° 映射到 0°~122°
        // float p2_upper = std::min(24.6f + target_p1, 125.0f);
        // target_p2 = std::clamp(target_p2, 0.0f, p2_upper);

        // 设定各轴插值，并且过滤死区
        if(fabs(arm.yaw    - last_arm.yaw ) > 0.1f)interp_yaw.setTarget(parm_->armCmd.set_angle_Yaw, Round(arm.yaw));
        if(fabs(arm.pitch1 - last_arm.pitch1) > 0.1f)interp_p1.setTarget(parm_->armCmd.set_angle_Pitch1, Round(arm.pitch1));
        if(fabs(arm.pitch2 - last_arm.pitch2) > 0.1f)interp_p2.setTarget(parm_->armCmd.set_angle_Pitch2, Round(arm.pitch2));
        if(fabs(arm.pitch3 - last_arm.pitch3) > 0.1f)interp_p3.setTarget(parm_->armCmd.set_angle_Pitch3, Round(arm.pitch3));
        if(fabs(arm.roll   - last_arm.roll   ) > 0.1f)interp_roll.setTarget(parm_->armCmd.set_angle_Roll, Round(-arm.roll));
        if(fabs(arm.pitch_end - last_arm.pitch_end) > 0.1f)interp_end_pitch.setTarget(parm_->armCmd.set_angle_end_pitch, Round(arm.pitch_end));
        last_arm = arm;
        // 每个控制周期执行插值
        parm_->armCmd.set_angle_Yaw    = interp_yaw.update();
        parm_->armCmd.set_angle_Pitch1 = interp_p1.update();
        parm_->armCmd.set_angle_Pitch2 = interp_p2.update();
        parm_->armCmd.set_angle_Pitch3 = interp_p3.update();
        parm_->armCmd.set_angle_Roll   = interp_roll.update();
        parm_->armCmd.set_angle_end_pitch = interp_end_pitch.update();
        
        // const float alpha = 0.98f;  ///< 低通滤波平滑系数（越大越平滑，0.95~0.98 对应约40~80ms过渡）
        // // 低通滤波平滑控制
        // parm_->armCmd.set_angle_Yaw       = LowPassFilter(parm_->armCmd.set_angle_Yaw,       arm.yaw,        alpha);
        // parm_->armCmd.set_angle_Pitch1    = LowPassFilter(parm_->armCmd.set_angle_Pitch1,    arm.pitch1,      alpha);
        // parm_->armCmd.set_angle_Pitch2    = LowPassFilter(parm_->armCmd.set_angle_Pitch2,    arm.pitch2,      alpha);
        // parm_->armCmd.set_angle_Pitch3    = LowPassFilter(parm_->armCmd.set_angle_Pitch3,    arm.pitch3,      alpha);
        // parm_->armCmd.set_angle_Roll      = LowPassFilter(parm_->armCmd.set_angle_Roll,      -arm.roll,       alpha);
        // parm_->armCmd.set_angle_end_pitch = LowPassFilter(parm_->armCmd.set_angle_end_pitch, arm.pitch_end,   alpha);

        // Yaw 最终物理限位
        parm_->armCmd.set_angle_Yaw = std::clamp(parm_->armCmd.set_angle_Yaw,
            ARM_YAW_PHYSICAL_RANGE_MIN, ARM_YAW_PHYSICAL_RANGE_MAX);

        // 末端roll轴
        parm_->armCmd.set_angle_end_roll += static_cast<float_t>(keyboard.key_F - keyboard.key_G) * 60.0f / freq;
        //parm_->armCmd.set_angle_end_roll += (controller.rocker_X / 100.f) * 100.f / freq;  // 摇杆增量
        // parm_->armCmd.set_angle_end_roll += 110.0f*(keyboard.key_F - keyboard.key_G)/freq;
        // if( SysRemote.remoteInfo.keyboard.key_Z){
        //     if(last_key_F != SysRemote.remoteInfo.keyboard.key_F)
        //         parm_->armCmd.set_angle_end_roll += 30.0f;
        //     if(last_key_G != SysRemote.remoteInfo.keyboard.key_G)
        //         parm_->armCmd.set_angle_end_roll -= 30.0f;
        // }

        // 二次夹紧，通过 armCmd 传递 re-grip 指令给模块层
        static bool regrip_keyboardcom = false;//键盘手动控制夹爪
        if(keyboard_edge.key_V == CSystemRemote::ERemoteEdge::Rising){
            regrip_keyboardcom = true;
        }
        const bool regrip_requested = controller.gripper_regrip || regrip_keyboardcom;
        if (regrip_requested) {
            parm_->armCmd.reGripCmd = true;                                  // 传递 re-grip 指令
        }
        controller.gripper_regrip = false;  // 处理之后清除标志位
        regrip_keyboardcom = false;

        // if (controller.gripper_close) {
        //     parm_->armCmd.set_speed_grip = -grip_speed;   // 闭合
        // } else if (!mode_switching) {
        //     parm_->armCmd.set_speed_grip = grip_speed;    // 张开
        // } else {
        //     parm_->armCmd.set_speed_grip = 0;             // 模式切换冻结
        // }
        if(keyboard_edge.key_C == CSystemRemote::ERemoteEdge::Rising){
            gripKeyboardcom_ = !gripKeyboardcom_;
        }
        const bool grip_close_cmd = controller.gripper_close || gripKeyboardcom_;
        if (grip_close_cmd) {
             parm_->armCmd.gripClose = true;
             parm_->armCmd.gripOpen = false;
             // isGripped 判断由组件层 HOLD 状态自动处理
         } else if (!mode_switching) {
             parm_->armCmd.gripClose = false;
             parm_->armCmd.gripOpen = true;
         } else {
             // 模式切换期间冻结，清零标志防止残留
             parm_->armCmd.gripClose = false;
             parm_->armCmd.gripOpen = false;
         }
    }
        // LowPassFilter(parm_->armCmd.set_angle_end_roll,
        //     Round(controller.angle_roll_end), 0.5f);
        // if (controller.Rocker_Key == CSystemControllerLink::KEY_STATUS::PRESS &&
        //     last_rocker_key_status == CSystemControllerLink::KEY_STATUS::RELEASE) {
        //     psubgantry_->subGantryCmd.setPumpOn_Gantry = !psubgantry_->subGantryCmd.setPumpOn_Gantry;
        // }
        if(keyboard_edge.key_Q == CSystemRemote::ERemoteEdge::Rising){
            robotdata.p3_lock = !robotdata.p3_lock;
        }

    // 云台yaw
    if (pgimbal_) {
        if(!pgimbal_->gimbalCmd.isAutoCtrl){  // 进入自定义控制器控制时云台自动归位
            pgimbal_->gimbalCmd.set_encoder_yaw = GIMBAL_YAW_INIT_ANGLE;  // 进入自定义控制器控制时云台自动归位
        }
    }
    // }
    last_key_F = SysRemote.remoteInfo.keyboard.key_F;
    last_key_G = SysRemote.remoteInfo.keyboard.key_G;
    // last_rocker_key_status = controller.Rocker_Key

    
}

// void CSystemCore::ControlFromEsp32_() {
//     const auto freq = 1000.f; // 系统核心频率

//     auto &esp32 = SysESP32.BLEInfo;

//     if (parm_) {
//         parm_->should_limit_yaw = 0;

//         if (SysESP32.BLE_Mode_Open) {
//             parm_->armCmd.set_angle_Yaw = LowPassFilter(parm_->armCmd.set_angle_Yaw, esp32.Yaw, 0.5f);
//             parm_->armCmd.set_angle_Pitch1 = LowPassFilter(parm_->armCmd.set_angle_Pitch1, esp32.Pitch1, 0.5f);
//             parm_->armCmd.set_angle_Pitch2 = LowPassFilter(parm_->armCmd.set_angle_Pitch2, esp32.Pitch2, 0.5f);
//             parm_->armCmd.set_angle_Roll = LowPassFilter(parm_->armCmd.set_angle_Roll, esp32.Roll, 0.5f);
//             parm_->armCmd.set_angle_end_pitch = LowPassFilter(parm_->armCmd.set_angle_end_pitch, esp32.End_Pitch, 0.5f);
//             parm_->armCmd.set_angle_end_roll = LowPassFilter(parm_->armCmd.set_angle_end_roll, esp32.End_Roll, 0.5f);
//         }
//     }
// }


}   // namespace my_engineer
