/**
 * @file control.cpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 在这里定义遥控器和键盘的操作函数
 * @version 1.0
 * @date 2024-11-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "Core.hpp"

namespace my_engineer {

void CSystemCore::StartRobot(bool if_remote_control, bool I_dont_have_a_remote) {

    if (!pchassis_->chassisInfo.isModuleAvailable
        && pchassis_->moduleStatus == APP_OK) {
        pchassis_->StartModule();
    }

    if (!pgimbal_->gimbalInfo.isModuleAvailable
        && pgimbal_->moduleStatus == APP_OK) {
        pgimbal_->StartModule();
    }

    if(if_remote_control) {
        enum { HIG = 1, LOW = 2, MID = 3 };
        auto &remote = SysRemote.remoteInfo.remote;

        static uint8_t last_switch_L = LOW; // 遥控器系统层的数据更新比设备层的状态更新要慢

        if (!parm_->armInfo.isModuleAvailable
            && parm_->moduleStatus == APP_OK
            && remote.switch_L == HIG && last_switch_L != HIG) {
            parm_->StartModule();
        }

        if (!psubgantry_->subGantryInfo.isModuleAvailable
            && psubgantry_->moduleStatus == APP_OK
            && remote.switch_L == HIG && last_switch_L != HIG) {
            psubgantry_->StartModule();
        }

        last_switch_L = remote.switch_L;

    }

    else {
        auto &keyboard = SysRemote.remoteInfo.keyboard;

        if (!parm_->armInfo.isModuleAvailable
            && parm_->moduleStatus == APP_OK
            && keyboard.key_Ctrl && keyboard.key_R) {
            parm_->StartModule();
        }

        if (!psubgantry_->subGantryInfo.isModuleAvailable
            && psubgantry_->moduleStatus == APP_OK
            && keyboard.key_Ctrl && keyboard.key_R) {
            psubgantry_->StartModule();
        }
    }

    if(I_dont_have_a_remote) {
        if (!parm_->armInfo.isModuleAvailable
            && parm_->moduleStatus == APP_OK) {
            parm_->StartModule();
        }

        if (!psubgantry_->subGantryInfo.isModuleAvailable
            && psubgantry_->moduleStatus == APP_OK) {
            psubgantry_->StartModule();
        }
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

    //将模块启动
    if (SysRemote.systemStatus == APP_OK) {
        StartRobot(true);
    }

    //用于调试，免去遥控器上电
    // StartRobot(true, true);

    parm_->should_limit_yaw = 0;

    // LOW + MID 底盘控制 和 云台抬升
    if (remote.switch_L == LOW && remote.switch_R == MID) {
        SysRemote.SetRemoteDeadZone(10.f);
        // 底盘控制
        pchassis_->chassisCmd.speed_X = remote.joystick_LX / 2;
        pchassis_->chassisCmd.speed_Y = remote.joystick_LY;
        pchassis_->chassisCmd.speed_W = remote.joystick_RX;
        // 云台抬升
        pgimbal_->gimbalCmd.set_posit_lift += 
            (remote.joystick_RY / 100.f) * 100.f / freq;
        
    }

    // MID + HIG 机械臂前四轴
    if (remote.switch_L == MID && remote.switch_R == HIG) {
        SysRemote.SetRemoteDeadZone(10.f);
        parm_->armCmd.set_angle_Yaw += 
            (remote.joystick_LX / 100.f) * 90.f / freq;
        parm_->armCmd.set_angle_Pitch1 +=
            (remote.joystick_LY / 100.f) * 90.f / freq;
        parm_->armCmd.set_angle_Pitch2 +=
            (remote.joystick_RY / 100.f) * 90.f / freq;
        parm_->armCmd.set_angle_Roll +=
            (remote.joystick_RX / 100.f) * 90.f / freq;
    }

    // MID + MID 机械臂后四轴
    if (remote.switch_L == MID && remote.switch_R == MID) {
        SysRemote.SetRemoteDeadZone(10.f);
        parm_->armCmd.set_angle_Pitch2 +=
            (remote.joystick_LY / 100.f) * 90.f / freq;
        parm_->armCmd.set_angle_Roll +=
            (remote.joystick_LX / 100.f) * 90.f / freq;
        parm_->armCmd.set_angle_end_pitch +=
            (remote.joystick_RY / 100.f) * 90.f / freq;
        parm_->armCmd.set_angle_end_roll +=
            (remote.joystick_RX / 100.f) * 90.f / freq;
    }

    // MID + LOW 子龙门控制
    if (remote.switch_L == MID && remote.switch_R == LOW) {
        SysRemote.SetRemoteDeadZone(10.f);
        psubgantry_->subGantryCmd.setStretchPosit_L +=
            (remote.joystick_LY / 100.f) * 300.f / freq;
        psubgantry_->subGantryCmd.setStretchPosit_R +=
            (remote.joystick_RY / 100.f) * 300.f / freq;
        psubgantry_->subGantryCmd.setLiftPosit_L +=
            (remote.joystick_LX / 100.f) * 120.f / freq;
        psubgantry_->subGantryCmd.setLiftPosit_R +=
            (remote.joystick_RX / 100.f) * 120.f / freq;
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
        StartRobot(false);
    }

    // parm_->should_limit_yaw = 1;

    /******************* 底盘控制 *******************/

    // 平滑更新角速度

    pchassis_->chassisCmd.speed_W = pchassis_->chassisCmd.speed_W +
        0.03f*(keyboard.mouse_X - pchassis_->chassisCmd.speed_W);

    if (!pchassis_->chassisCmd.isAutoCtrl)
    {
        pchassis_->chassisCmd.speed_X *= 0.97f;
        pchassis_->chassisCmd.speed_Y *= 0.98f;
        if (abs(pchassis_->chassisCmd.speed_X) < 0.5f) pchassis_->chassisCmd.speed_X = 0.0f;
        if (abs(pchassis_->chassisCmd.speed_Y) < 0.5f) pchassis_->chassisCmd.speed_Y = 0.0f;

        if (keyboard.key_Shift) {
            pchassis_->chassisCmd.speed_X += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 5.0f;
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
    }
    

    // 小陀螺  (G键)
    if (!keyboard.key_Ctrl
        && keyboard.key_G
        && currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE) {
        pchassis_->chassisCmd.speed_W = static_cast<float_t>(keyboard.mouse_R - keyboard.mouse_L) * 100.f;
    }

    /******************* 云台手动控制 *******************/
    if (!keyboard.key_Ctrl && 
        !pgimbal_->gimbalCmd.isAutoCtrl) {
        // (F键)
        if (keyboard.key_F) {
            pgimbal_->gimbalCmd.set_posit_lift += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 120.0f / freq;
        }
    }

    /******************* 机械臂手动控制 *******************/
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
        // 气泵(B键)
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

    lastMouseStatus_L = keyboard.mouse_L;
    lastMouseStatus_R = keyboard.mouse_R;


    /******************* 自动控制 *******************/
    if (keyboard.key_Ctrl
    && parm_->armInfo.isModuleAvailable)
    {
        if(keyboard.key_V)
        {
            //TODO: 这个任务最好可以用于终止proc_waituntil
            StopAutoCtrlTask_();
            StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_ORIGIN);
        }
        if(keyboard.key_G)
        {
            StartAutoCtrlTask_(EAutoCtrlProcess::GOLD_ORE);
        }
        if(keyboard.key_X)
        {
            StartAutoCtrlTask_(EAutoCtrlProcess::SILVER_ORE);
        }
        if(keyboard.key_F)
        {
            StartAutoCtrlTask_(EAutoCtrlProcess::GROUND_ORE);
        }
        if(keyboard.key_R)
        {
            StartAutoCtrlTask_(EAutoCtrlProcess::PUSH_ORE);
        }
        if(keyboard.key_Q)
        {
            StartAutoCtrlTask_(EAutoCtrlProcess::POP_ORE);
        }
        // if(keyboard.key_B)
        // {
        //     StartAutoCtrlTask_(EAutoCtrlProcess::EXCHANGE);
        // }

    }
    if(keyboard.key_Shift &&
        parm_->armInfo.isModuleAvailable){
        if(keyboard.key_Z)
        {
            StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_DRIVE);
        }
        if(keyboard.key_C)
        {
            StartAutoCtrlTask_(EAutoCtrlProcess::DOGHOLE);
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

    static CSystemControllerLink::KEY_STATUS last_rocker_key_status;

    // 将模块启动
    if (SysRemote.systemStatus == APP_OK) {
        StartRobot(false);
    }
	
    parm_->should_limit_yaw = 0;

    /******************* 底盘控制 *******************/
    if (!pchassis_->chassisCmd.isAutoCtrl)
    {
        pchassis_->chassisCmd.speed_X *= 0.97f;
        pchassis_->chassisCmd.speed_Y *= 0.98f;
        pchassis_->chassisCmd.speed_W *= 0.98f;
        if (abs(pchassis_->chassisCmd.speed_X) < 0.5f) pchassis_->chassisCmd.speed_X = 0.0f;
        if (abs(pchassis_->chassisCmd.speed_Y) < 0.5f) pchassis_->chassisCmd.speed_Y = 0.0f;
        if (abs(pchassis_->chassisCmd.speed_W) < 0.3f) pchassis_->chassisCmd.speed_W = 0.0f;

        if (keyboard.key_Shift) {
            pchassis_->chassisCmd.speed_X += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 5.0f;
            pchassis_->chassisCmd.speed_Y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 5.0f;
            pchassis_->chassisCmd.speed_W += static_cast<float_t>(keyboard.key_E - keyboard.key_Q) * 5.0f;
            pchassis_->chassisCmd.speed_X =
            std::clamp(pchassis_->chassisCmd.speed_X, -50.0f, 50.0f);
            pchassis_->chassisCmd.speed_Y =
            std::clamp(pchassis_->chassisCmd.speed_Y, -100.0f, 100.0f);
            pchassis_->chassisCmd.speed_W =
            std::clamp(pchassis_->chassisCmd.speed_W, -50.0f, 50.0f);
        } else {
            pchassis_->chassisCmd.speed_X += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 0.8f;
            pchassis_->chassisCmd.speed_Y += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 0.8f;
            pchassis_->chassisCmd.speed_W += static_cast<float_t>(keyboard.key_E - keyboard.key_Q) * 0.4f;
            pchassis_->chassisCmd.speed_X =
            std::clamp(pchassis_->chassisCmd.speed_X, -20.0f, 20.0f);
            pchassis_->chassisCmd.speed_Y =
            std::clamp(pchassis_->chassisCmd.speed_Y, -30.0f, 30.0f);
            pchassis_->chassisCmd.speed_W =
            std::clamp(pchassis_->chassisCmd.speed_W, -20.0f, 20.0f);
        }
    }

    /******************* 机械臂 *******************/
    //  if (controller.return_success) {
        parm_->armCmd.set_angle_Yaw =
            LowPassFilter(parm_->armCmd.set_angle_Yaw,
                Round(controller.angle_yaw), 0.5f);
        parm_->armCmd.set_angle_Pitch1 =
            LowPassFilter(parm_->armCmd.set_angle_Pitch1,
                Round(controller.angle_pitch1 + 20.0f), 0.5f);
        parm_->armCmd.set_angle_Pitch2 =
            LowPassFilter(parm_->armCmd.set_angle_Pitch2,
                Round(controller.angle_pitch2 + 20.0f), 0.5f);
        parm_->armCmd.set_angle_Roll =
            LowPassFilter(parm_->armCmd.set_angle_Roll,
                Round(-controller.angle_roll), 0.5f);
        if (controller.angle_pitch1 < 38.0f) {
            parm_->armCmd.set_angle_end_pitch = 
                std::clamp(parm_->armCmd.set_angle_end_pitch, 0.0f, 40.0f);
        }
        else {
            parm_->armCmd.set_angle_end_pitch =
            LowPassFilter(parm_->armCmd.set_angle_end_pitch,
                Round(controller.angle_pitch_end), 0.5f);
        }
        // 只有末端的roll轴是增量式控制
        parm_->armCmd.set_angle_end_roll += 110.0f*(keyboard.key_F - keyboard.key_G)/freq;
        if( SysRemote.remoteInfo.keyboard.key_Z){
            if(last_key_F != SysRemote.remoteInfo.keyboard.key_F)
                parm_->armCmd.set_angle_end_roll += 30.0f;
            if(last_key_G != SysRemote.remoteInfo.keyboard.key_G)
                parm_->armCmd.set_angle_end_roll -= 30.0f;
        }
            // LowPassFilter(parm_->armCmd.set_angle_end_roll,
            //     Round(controller.angle_roll_end), 0.5f);
        // if (controller.Rocker_Key == CSystemControllerLink::KEY_STATUS::PRESS &&
        //     last_rocker_key_status == CSystemControllerLink::KEY_STATUS::RELEASE) {
        //     psubgantry_->subGantryCmd.setPumpOn_Gantry = !psubgantry_->subGantryCmd.setPumpOn_Gantry;
        // }

        // 云台抬升
        pgimbal_->gimbalCmd.set_posit_lift += static_cast<float_t>(keyboard.key_Z - keyboard.key_X) * 60.0f / freq;
        psubgantry_->subGantryCmd.setLiftPosit_L = 88.5f;           ///<兑矿时必须保证刺雷抬到最高点
        psubgantry_->subGantryCmd.setLiftPosit_R = 88.5f;

        // 气泵控制
        static uint8_t vb_count = 0;
        static bool vb_flag = false;
        if (SysRemote.remoteInfo.keyboard.key_V && SysRemote.remoteInfo.keyboard.key_B) {
            vb_count++;
        }
        // 全部松开之后才清零计数器
        else if (SysRemote.remoteInfo.keyboard.key_V || SysRemote.remoteInfo.keyboard.key_B == false) {
            vb_count = 0;
            vb_flag = false;
        }
        if (vb_count > 20 && vb_flag == false) {
            vb_flag = true;
            vb_count = 0;
            psubgantry_->subGantryCmd.setPumpOn_Arm = !psubgantry_->subGantryCmd.setPumpOn_Arm;
        }

    // }
        last_key_F = SysRemote.remoteInfo.keyboard.key_F;
        last_key_G = SysRemote.remoteInfo.keyboard.key_G;
    // last_rocker_key_status = controller.Rocker_Key

    
}

void CSystemCore::ControlFromEsp32_() {
    const auto freq = 1000.f; // 系统核心频率

    auto &esp32 = SysESP32.BLEInfo;

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


}   // namespace my_engineer
