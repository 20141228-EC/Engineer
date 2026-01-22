/**
 * @file control.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 在这里定义遥控器和键盘的操作函数
 * @version 1.0
 * @date 2025-12-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "Core.hpp"

namespace my_engineer {

void CSystemCore::StartRobot(bool if_remote_control, bool I_dont_have_a_remote) {

    if (pchassis_) {
        if (!pchassis_->chassisInfo.isModuleAvailable               ///<说明模块已经注册了
            && pchassis_->moduleStatus == APP_OK) {
            pchassis_->StartModule();                               ///<在创建任务的时候还会再调用一次初始化函数
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
                parm_->StartModule();       ///< 左拨杆拨到上时初始化臂
            }
        }

        last_switch_L = remote.switch_L;        ///<记录上一次的左拨杆状态

    }

    else {
        auto &keyboard = SysRemote.remoteInfo.keyboard;

        if (parm_) {
            if (!parm_->armInfo.isModuleAvailable
                && parm_->moduleStatus == APP_OK
                && keyboard.key_Ctrl && keyboard.key_R) { ///< ctrl+r 初始化臂
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
    // StartRobot(true, true);

    if (parm_) {
        parm_->should_limit_yaw = 0;
    }

    // 仅在非自动任务时根据拨杆更新运动模式(或键位 键位待设)
    if (currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE)
    {
        if (remote.switch_L == HIG && remote.switch_R == MID) {
            movemode_ = EMoveMode::CLIMBING;
            pchassis_->MovMode = CModChassis::EmovMode::CLIMBING;
        }
        else {
            movemode_ = EMoveMode::NORMAL;
            pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
        }
    }
    else
    {
        // 如果在自动任务里面，则运动模式由对应任务决定
    }

    // LOW + MID 底盘控制(轮毂+髋) + 云台抬升
    if (remote.switch_L == LOW && remote.switch_R == MID) {
        SysRemote.SetRemoteDeadZone(10.f);
        // 底盘控制
        if (pchassis_) {
            if(!pchassis_->chassisCmd.isAutoCtrl){
                pchassis_->chassisCmd.speed_X = remote.joystick_LX / 2;             ///<摇杆的x方向控制车的左右移动，为了保证操作手的手感减小左右方向的速度
                pchassis_->chassisCmd.speed_Y = remote.joystick_LY;
                pchassis_->chassisCmd.speed_W = remote.joystick_RX;
                pchassis_->chassisCmd.L_length += (remote.joystick_RY / 100.f) * 90.f / freq; ///< 腿长采用增量式控制
                pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
                
            static uint8_t thumbwheel_count = 0;

            if(remote_edge.thumbWheel == CSystemRemote::ERemoteEdge::Falling){
                pchassis_->reset_hip = !pchassis_->reset_hip;   ///< 要求复位腿
            }

            }    
        }
        // 云台的抬升逻辑此处也没写，在副板，用拨轮控
    }

    // MID + HIG 主臂关节四轴 + 夹爪
    else if (remote.switch_L == MID && remote.switch_R == HIG) {
        SysRemote.SetRemoteDeadZone(10.f);
        if (parm_) {
            if(!parm_->armCmd.isAutoCtrl){
                parm_->armCmd.set_angle_Yaw +=
                    (remote.joystick_LX / 100.f) * 90.f / freq;
                parm_->armCmd.set_angle_Pitch1 +=
                    (remote.joystick_LY / 100.f) * 90.f / freq;
                parm_->armCmd.set_angle_Pitch2 +=
                    (remote.joystick_RY / 100.f) * 90.f / freq;
                parm_->armCmd.set_angle_Roll +=
                    (remote.joystick_RX / 100.f) * 90.f / freq;
                parm_->armCmd.set_length_grip +=
                    (remote.thumbWheel / 100.f) * 90.f / freq; ///< 拨轮控夹爪
            }
        }
        if(pchassis_){
            pchassis_->MovMode = CModChassis::EmovMode::NORMAL; 
        }
    }

    // MID + MID 副臂关节四轴 + 夹爪
    if (remote.switch_L == MID && remote.switch_R == MID) {
        SysRemote.SetRemoteDeadZone(10.f);
        ///< 此处不执行任何操作，由板间通信将整个遥控器数据传给副板，副板自己执行控制逻辑
        if(pchassis_){
            if(!pchassis_->chassisCmd.isAutoCtrl){
                pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
            } 
        }
    }

    // MID + LOW 左摇杆主臂末端，右摇杆副臂末端
    else if (remote.switch_L == MID && remote.switch_R == LOW) {
        SysRemote.SetRemoteDeadZone(10.f);
        if (parm_) {
            if(!parm_->armCmd.isAutoCtrl){
                parm_->armCmd.set_angle_end_pitch +=
                    (remote.joystick_LY / 100.f) * 300.f / freq;
                parm_->armCmd.set_angle_end_roll +=
                    (remote.joystick_LX / 100.f) * 120.f / freq;
            }
                // 只写了主臂末端的控制，副臂的目标设置在副板代码中
        }
        if(pchassis_){
            if(!pchassis_->chassisCmd.isAutoCtrl){
                pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
            } 
        }
    }

    // HIG + MID 自动上台阶 利用陀螺仪数据控腿长
    else if(remote.switch_L == HIG && remote.switch_R == MID)
    {
        SysRemote.SetRemoteDeadZone(10.f);
        // 底盘控制
        if (pchassis_) {
            if(!pchassis_->chassisCmd.isAutoCtrl){
                pchassis_->chassisCmd.speed_X = remote.joystick_LX / 2;             ///<摇杆的x方向控制车的左右移动，为了保证操作手的手感减小左右方向的速度
                pchassis_->chassisCmd.speed_Y = remote.joystick_LY;
                pchassis_->chassisCmd.speed_W = remote.joystick_RX;
                pchassis_->MovMode = CModChassis::EmovMode::CLIMBING;               ///< 更新模块运动模式标志位
            }

            static uint8_t thumbwheel_count = 0;

            if(remote_edge.thumbWheel == CSystemRemote::ERemoteEdge::Falling){
                pchassis_->reset_hip = !pchassis_->reset_hip;   ///< 要求复位腿
            }

            ///< 右摇杆y控云台pitch，逻辑在副板
        }
    }
    // HIG + LOW 云台全控制
    else if(remote.switch_L == HIG && remote.switch_R == LOW)
    {
        if(pchassis_){
            if(!pchassis_->chassisCmd.isAutoCtrl){
                pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
            } 
        }
        ///< 云台控制逻辑均在副板
    }
}

/**
 * @brief 键盘操作
 * 
 */
void CSystemCore::ControlFromKeyboard_() {
    const auto freq = 1000.f; // 系统核心频率

    auto &keyboard = SysRemote.remoteInfo.keyboard;

    static bool lastMouseStatus_L = false, lastMouseStatus_R = false;

    // 将模块启动
    if (SysRemote.systemStatus == APP_OK) {
        StartRobot(false);                  ///<转换为键盘操作
    }

    // parm_->should_limit_yaw = 1;

    /******************* 底盘控制 *******************/

    // 平滑更新角速度
    if (pchassis_) {
        pchassis_->chassisCmd.speed_W = pchassis_->chassisCmd.speed_W +
            0.03f*(keyboard.mouse_X - pchassis_->chassisCmd.speed_W);

        if (!pchassis_->chassisCmd.isAutoCtrl)
        {
            pchassis_->chassisCmd.speed_X *= 0.97f;
            pchassis_->chassisCmd.speed_Y *= 0.98f;
            if (abs(pchassis_->chassisCmd.speed_X) < 0.5f) pchassis_->chassisCmd.speed_X = 0.0f;
            if (abs(pchassis_->chassisCmd.speed_Y) < 0.5f) pchassis_->chassisCmd.speed_Y = 0.0f;

            if (keyboard.key_Shift) {
                pchassis_->chassisCmd.speed_X += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 5.0f;   ///<通过差值来实现一行代码实现左右转弯
                pchassis_->chassisCmd.speed_Y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 5.0f;
                pchassis_->chassisCmd.speed_X =
                std::clamp(pchassis_->chassisCmd.speed_X, -50.0f, 50.0f);
                pchassis_->chassisCmd.speed_Y =
                std::clamp(pchassis_->chassisCmd.speed_Y, -100.0f, 100.0f);
            } else {
                pchassis_->chassisCmd.speed_X += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 1.0f;
                pchassis_->chassisCmd.speed_Y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 1.0f;
                pchassis_->chassisCmd.speed_X =
                std::clamp(pchassis_->chassisCmd.speed_X, -20.0f, 20.0f);
                pchassis_->chassisCmd.speed_Y =
                std::clamp(pchassis_->chassisCmd.speed_Y, -30.0f, 30.0f);
            }
            if(keyboard.key_B){
                pchassis_->chassisCmd.L_length += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 0.3f;
            }
            if(keyboard.key_Ctrl
                && keyboard.key_B
                && keyboard.key_Shift
                && currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE) {
                pchassis_->reset_hip = !pchassis_->reset_hip;
        }
    }
    

    // 小陀螺  (G键)
    if (pchassis_) {
        if (!keyboard.key_Ctrl
            && keyboard.key_G
            && currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE) {
            pchassis_->chassisCmd.speed_W = static_cast<float_t>(keyboard.mouse_R - keyboard.mouse_L) * 100.f;
        }
    }

    /******************* 云台手动控制 *******************/

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
            // roll(Z键)
            if(keyboard.key_Z)
                parm_->armCmd.set_angle_Roll += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 80.0f / freq;
            // end_pitch(X键)
            if(keyboard.key_X)
                parm_->armCmd.set_angle_end_pitch += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 90.0f / freq;
            // end_roll(C键)
            if(keyboard.key_C)
                parm_->armCmd.set_angle_end_roll += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 90.0f / freq;
        }
    }

    lastMouseStatus_L = keyboard.mouse_L;
    lastMouseStatus_R = keyboard.mouse_R;


    /******************* 自动控制 *******************/
    if (parm_ && pchassis_) {
        if (keyboard.key_Ctrl && parm_->armInfo.isModuleAvailable)
        {
            // Ctrl + V: 停止所有自动任务
            if(keyboard.key_V)
            {
                StopAutoCtrlTask_();
            }

            // Ctrl + C: 启动上台阶任务
            if(keyboard.key_C)
            {
                StartAutoCtrlTask_(EAutoCtrlProcess::CLIMBING);
            }

            /* --- 其他旧的自动任务快捷键已被移除 ---
            if(keyboard.key_G) { StartAutoCtrlTask_(EAutoCtrlProcess::GOLD_ORE); }
            if(keyboard.key_X) { StartAutoCtrlTask_(EAutoCtrlProcess::SILVER_ORE); }
            if(keyboard.key_F) { StartAutoCtrlTask_(EAutoCtrlProcess::GROUND_ORE); }
            if(keyboard.key_R) { StartAutoCtrlTask_(EAutoCtrlProcess::PUSH_ORE); }
            if(keyboard.key_Q) { StartAutoCtrlTask_(EAutoCtrlProcess::POP_ORE); }
            */
        }
        /* --- 其他旧的自动任务快捷键已被移除 ---
        if(keyboard.key_Shift && parm_->armInfo.isModuleAvailable){
            if(keyboard.key_Z) { StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_DRIVE); }
            if(keyboard.key_C) { StartAutoCtrlTask_(EAutoCtrlProcess::DOGHOLE); }
        }
        */
    }
    }
}

/**
 * @brief 自定义控制器操作
 */
void CSystemCore::ControlFromController_() {
    const auto freq = 1000.f; // 系统核心频率
    static uint16_t last_key_F,last_key_G;

    auto &controller = SysControllerLink.controllerInfo;

    auto &keyboard = SysRemote.remoteInfo.keyboard;

    SysControllerLink.robotInfo.controlled_by_controller = true;
    SysControllerLink.robotInfo.ask_return_flag = true;

//     static CSystemControllerLink::KEY_STATUS last_rocker_key_status;

//     // 将模块启动
//     if (SysRemote.systemStatus == APP_OK) {
//         StartRobot(false);
//     }

//     if (parm_) {
//         parm_->should_limit_yaw = 0;
//     }

//     /******************* 底盘控制 *******************/
//     if (pchassis_) {
//         if (!pchassis_->chassisCmd.isAutoCtrl)
//         {
//             pchassis_->chassisCmd.speed_X *= 0.97f;
//             pchassis_->chassisCmd.speed_Y *= 0.98f;
//             pchassis_->chassisCmd.speed_W *= 0.98f;
//             if (abs(pchassis_->chassisCmd.speed_X) < 0.5f) pchassis_->chassisCmd.speed_X = 0.0f;
//             if (abs(pchassis_->chassisCmd.speed_Y) < 0.5f) pchassis_->chassisCmd.speed_Y = 0.0f;
//             if (abs(pchassis_->chassisCmd.speed_W) < 0.3f) pchassis_->chassisCmd.speed_W = 0.0f;

//             if (keyboard.key_Shift) {
//                 pchassis_->chassisCmd.speed_X += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 5.0f;
//                 pchassis_->chassisCmd.speed_Y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 5.0f;
//                 pchassis_->chassisCmd.speed_W += static_cast<float_t>(keyboard.key_E - keyboard.key_Q) * 5.0f;
//                 pchassis_->chassisCmd.speed_X =
//                 std::clamp(pchassis_->chassisCmd.speed_X, -50.0f, 50.0f);
//                 pchassis_->chassisCmd.speed_Y =
//                 std::clamp(pchassis_->chassisCmd.speed_Y, -100.0f, 100.0f);
//                 pchassis_->chassisCmd.speed_W =
//                 std::clamp(pchassis_->chassisCmd.speed_W, -50.0f, 50.0f);
//             } else {
//                 pchassis_->chassisCmd.speed_X += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 0.8f;
//                 pchassis_->chassisCmd.speed_Y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 0.8f;
//                 pchassis_->chassisCmd.speed_W += static_cast<float_t>(keyboard.key_E - keyboard.key_Q) * 0.4f;
//                 pchassis_->chassisCmd.speed_X =
//                 std::clamp(pchassis_->chassisCmd.speed_X, -20.0f, 20.0f);
//                 pchassis_->chassisCmd.speed_Y =
//                 std::clamp(pchassis_->chassisCmd.speed_Y, -30.0f, 30.0f);
//                 pchassis_->chassisCmd.speed_W =
//                 std::clamp(pchassis_->chassisCmd.speed_W, -20.0f, 20.0f);
//             }
//         }
//     }

//     /******************* 机械臂 *******************/
//     //  if (controller.return_success) {
//     if (parm_) {
//         parm_->armCmd.set_angle_Yaw =
//             LowPassFilter(parm_->armCmd.set_angle_Yaw,
//                 Round(controller.), 0.5f);
//         parm_->armCmd.set_angle_Pitch1 =
//             LowPassFilter(parm_->armCmd.set_angle_Pitch1,
//                 Round(controller.angle_pitch1 + 20.0f), 0.5f);
//         parm_->armCmd.set_angle_Pitch2 =
//             LowPassFilter(parm_->armCmd.set_angle_Pitch2,
//                 Round(controller.angle_pitch2 + 20.0f), 0.5f);
//         parm_->armCmd.set_angle_Roll =
//             LowPassFilter(parm_->armCmd.set_angle_Roll,
//                 Round(-controller.angle_roll), 0.5f);
//         if (controller.angle_pitch1 < 38.0f) {
//             parm_->armCmd.set_angle_end_pitch =
//                 std::clamp(parm_->armCmd.set_angle_end_pitch, 0.0f, 40.0f);
//         }
//         else {
//             parm_->armCmd.set_angle_end_pitch =
//             LowPassFilter(parm_->armCmd.set_angle_end_pitch,
//                 Round(controller.angle_pitch_end), 0.5f);
//         }
//         // 只有末端的roll轴是增量式控制
//         parm_->armCmd.set_angle_end_roll += 110.0f*(keyboard.key_F - keyboard.key_G)/freq;
//         if( SysRemote.remoteInfo.keyboard.key_Z){
//             if(last_key_F != SysRemote.remoteInfo.keyboard.key_F)
//                 parm_->armCmd.set_angle_end_roll += 30.0f;
//             if(last_key_G != SysRemote.remoteInfo.keyboard.key_G)
//                 parm_->armCmd.set_angle_end_roll -= 30.0f;
//         }
//     }
//         // LowPassFilter(parm_->armCmd.set_angle_end_roll,
//         //     Round(controller.angle_roll_end), 0.5f);
//         // if (controller.Rocker_Key == CSystemControllerLink::KEY_STATUS::PRESS &&
//         //     last_rocker_key_status == CSystemControllerLink::KEY_STATUS::RELEASE) {
//         //     psubgantry_->subGantryCmd.setPumpOn_Gantry = !psubgantry_->subGantryCmd.setPumpOn_Gantry;
//         // }

//     // 云台抬升

// /*删除自定义控制器对应的兑矿操作
//     if (psubgantry_) {
//         psubgantry_->subGantryCmd.setLiftPosit_L = 88.5f;           ///<兑矿时必须保证刺雷抬到最高点
//         psubgantry_->subGantryCmd.setLiftPosit_R = 88.5f;

//         // 气泵控制
//         static uint8_t vb_count = 0;
//         static bool vb_flag = false;
//         if (SysRemote.remoteInfo.keyboard.key_V && SysRemote.remoteInfo.keyboard.key_B) {
//             vb_count++;
//         }
//         // 全部松开之后才清零计数器
//         else if (SysRemote.remoteInfo.keyboard.key_V || SysRemote.remoteInfo.keyboard.key_B == false) {
//             vb_count = 0;
//             vb_flag = false;
//         }
//         if (vb_count > 20 && vb_flag == false) {
//             vb_flag = true;
//             vb_count = 0;
//             psubgantry_->subGantryCmd.setPumpOn_Arm = !psubgantry_->subGantryCmd.setPumpOn_Arm;
//         }
//     }
// */
//     // }
//     last_key_F = SysRemote.remoteInfo.keyboard.key_F;
//     last_key_G = SysRemote.remoteInfo.keyboard.key_G;
//     // last_rocker_key_status = controller.Rocker_Key

    
}

void CSystemCore::ControlFromEsp32_() {
    const auto freq = 1000.f; // 系统核心频率

    auto &esp32 = SysESP32.BLEInfo;

    if (parm_) {
        parm_->should_limit_yaw = 0;

        if (SysESP32.BLE_Mode_Open) {
            parm_->armCmd.set_angle_Yaw = LowPassFilter(parm_->armCmd.set_angle_Yaw, esp32.Yaw, 0.5f);
            parm_->armCmd.set_angle_Pitch1 = LowPassFilter(parm_->armCmd.set_angle_Pitch1, esp32.Pitch1, 0.5f);
            parm_->armCmd.set_angle_Pitch2 = LowPassFilter(parm_->armCmd.set_angle_Pitch2, esp32.Pitch2, 0.5f);
            parm_->armCmd.set_angle_Roll = LowPassFilter(parm_->armCmd.set_angle_Roll, esp32.Roll, 0.5f);
            parm_->armCmd.set_angle_end_pitch = LowPassFilter(parm_->armCmd.set_angle_end_pitch, esp32.End_Pitch, 0.5f);
            parm_->armCmd.set_angle_end_roll = LowPassFilter(parm_->armCmd.set_angle_end_roll, esp32.End_Roll, 0.5f);
        }
    }
}


}   // namespace my_engineer
