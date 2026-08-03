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
                && keyboard.key_Ctrl && keyboard.key_Shift && keyboard.key_F) { ///< ctrl+shift+f 初始化臂
                parm_->StartModule();
            }
        }
    }

    if(I_dont_have_a_remote) {
        if (parm_) {
            if (!parm_->armInfo.isModuleAvailable
                && parm_->moduleStatus == APP_OK) {
                parm_->StartModule();
            }
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

    // LOW + MID 底盘控制(轮毂+髋)
    if (remote.switch_L == LOW && remote.switch_R == MID) {
        SysRemote.SetRemoteDeadZone(10.f);
        // 底盘控制
        if (pchassis_) {
            if(!pchassis_->chassisCmd.isAutoCtrl){
                pchassis_->chassisCmd.speed_X = remote.joystick_LX / 2;             ///<摇杆的x方向控制车的左右移动，为了保证操作手的手感减小左右方向的速度
                pchassis_->chassisCmd.speed_Y = remote.joystick_LY;
                pchassis_->chassisCmd.speed_W = remote.joystick_RX;
                pchassis_->chassisCmd.L_length += (remote.joystick_RY / 250.f) * 90.f / freq; ///< 腿长采用增量式控制
                pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
                
            static uint8_t thumbwheel_count = 0;

            if(remote_edge.thumbWheel == CSystemRemote::ERemoteEdge::Falling){
                pchassis_->reset_hip = !pchassis_->reset_hip;   ///< 要求复位腿
                }
            if(remote_edge.thumbWheel == CSystemRemote::ERemoteEdge::Rising){
                pchassis_->chassisInfo.crawler_on = !pchassis_->chassisInfo.crawler_on; ///< 启动履带电机
                }
            // if(pchassis_->filter->Imu_Ave_Info.imu_ave_pitch >= 23.f){
            //     parm_->armCmd.set_angle_Pitch1 = 4.f;
            //     parm_->armCmd.set_angle_Pitch2 = 11.f;  
            // }

            }    
        }
    }

    // MID + HIG 臂关节四轴 + 夹爪
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
				parm_->armCmd.set_speed_grip =
					(remote.thumbWheel / 100.f) * 6000.f;
            }
        }
        if(pchassis_){
            pchassis_->MovMode = CModChassis::EmovMode::NORMAL; 
        }
    }

    // MID + MID 副臂关节四轴 + 夹爪
    else if (remote.switch_L == MID && remote.switch_R == MID) {
        SysRemote.SetRemoteDeadZone(10.f);
        ///< 此处不执行任何操作，由板间通信将整个遥控器数据传给副板，副板自己执行控制逻辑
        if(pchassis_){
            if(!pchassis_->chassisCmd.isAutoCtrl){
                pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
            }
        if(parm_){
            parm_->armCmd.set_angle_Roll +=
                    (remote.joystick_RY / 100.f) * 90.f / freq;
            parm_->armCmd.set_angle_end_pitch +=
                    (remote.joystick_LY / 100.f) * 300.f / freq;
            parm_->armCmd.set_angle_end_roll +=
                    (remote.joystick_LX / 100.f) * 120.f / freq;
        }
        if(pgimbal_){
            pgimbal_->gimbalCmd.set_visualyaw +=
                    (remote.joystick_RX / 100.f) * 200.f / freq;
            pgimbal_->gimbalCmd.set_pitch +=
                    (remote.thumbWheel / 100.f) * 60.f / freq;
        } 
        }
    }

    // MID + LOW 左摇杆主臂末端，右摇杆副臂末端
    else if (remote.switch_L == MID && remote.switch_R == LOW) {
        SysRemote.SetRemoteDeadZone(10.f);
        if (parm_) {
            if(!parm_->armCmd.isAutoCtrl){

            }
        }
        if(pchassis_){
            if(!pchassis_->chassisCmd.isAutoCtrl){
                
            } 
        }
    }
    else{   // 未定义模式直接锁底盘
        if(pchassis_){
            if(!pchassis_->chassisCmd.isAutoCtrl){
                pchassis_->chassisCmd.speed_X = 0.f;
                pchassis_->chassisCmd.speed_Y = 0.f;
                pchassis_->chassisCmd.speed_W = 0.f;
            } 
        }
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
    auto &controller = SysControllerLink.controllerInfo;

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
            0.1f*(keyboard.mouse_X - pchassis_->chassisCmd.speed_W);

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
                pchassis_->chassisCmd.L_length += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 0.006f;
                if(keyboard.mouse_L != keyboard.mouse_R){
                    pgimbal_->gimbalCmd.set_pitch = GIMBAL_PITCH_INIT_ANGLE + pchassis_->chassisInfo.L_Length * 0.52f;// 同步抬升
                }
            }
            if(keyboard.key_Ctrl
                && keyboard.key_B
                && keyboard.key_Shift
                && currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE) {
                pchassis_->reset_hip = !pchassis_->reset_hip;
                pgimbal_->gimbalCmd.set_pitch = 0.f;
                pgimbal_->gimbalCmd.set_visualyaw = 0.f;
            }
            if(keyboard_edge.key_F == CSystemRemote::ERemoteEdge::Rising
             &&keyboard_edge.key_G == CSystemRemote::ERemoteEdge::Rising) {
                pchassis_->chassisInfo.crawler_on = !pchassis_->chassisInfo.crawler_on;
            }
        }
    }

    /******************* 云台手动控制 *******************/
    // (G键: yaw, F键: pitch)
    if (pgimbal_) {
        if (!pgimbal_->gimbalCmd.isAutoCtrl) {
            if (keyboard.key_G) {
                pgimbal_->gimbalCmd.set_visualyaw += static_cast<float_t>(keyboard.mouse_R - keyboard.mouse_L) * 70.f / freq;
            }
            if (keyboard.key_F) {
                pgimbal_->gimbalCmd.set_pitch += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 60.f / freq;
            }
        }
    }

    /******************* 机械臂手动控制 *******************/
    if (parm_) {
        if (!parm_->armCmd.isAutoCtrl) {
            // !SysBoardLink.pArm_Cmd->isAutoCtrl) {
            // yaw(Q键)
            if(keyboard.key_Q)
                parm_->armCmd.set_angle_Yaw += static_cast<float_t>(keyboard.mouse_R - keyboard.mouse_L) * 60.0f / freq;
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

            parm_->armCmd.gripClose = (gripKeyboardCmd_ == EGripKeyboardCmd::CLOSE);
            parm_->armCmd.gripOpen = (gripKeyboardCmd_ == EGripKeyboardCmd::OPEN);
        }
    }

    lastMouseStatus_L = keyboard.mouse_L;
    lastMouseStatus_R = keyboard.mouse_R;


    /******************* 自动控制 *******************/
    if (parm_ && pchassis_) {
        if (keyboard.key_Ctrl && !keyboard.key_Shift && pchassis_->chassisInfo.isModuleAvailable)
        {
            // Ctrl + Z: 停止所有自动任务
            if(keyboard.key_Z)
            {
                StopAutoCtrlTask_();
            }

            // Ctrl + C: 启动上台阶任务
            if(keyboard.key_C)
            {
                StartAutoCtrlTask_(EAutoCtrlProcess::CLIMBING);
            }

            // Ctrl + V: 启动下台阶任务
            if(keyboard.key_V){
                StartAutoCtrlTask_(EAutoCtrlProcess::DOWN_STAIR);
            }

            // Ctrl + X: 启动存矿任务
            if(keyboard.key_X){
                exchange_side_ = EExchangeSide::NONE;
                StartAutoCtrlTask_(EAutoCtrlProcess::STORE_ORE);
            }

            // Ctrl + B: 启动取矿任务
            if(keyboard.key_B){
                exchange_side_ = EExchangeSide::NONE;
                StartAutoCtrlTask_(EAutoCtrlProcess::EXCHANGE_ORE);
            }
            // Ctrl + R: 启动全部复位任务
            if(keyboard.key_R){
                StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_ORIGIN);
            }

            /* --- 其他旧的自动任务快捷键已被移除 ---
            if(keyboard.key_G) { StartAutoCtrlTask_(EAutoCtrlProcess::GOLD_ORE); }
            if(keyboard.key_X) { StartAutoCtrlTask_(EAutoCtrlProcess::SILVER_ORE); }
            if(keyboard.key_F) { StartAutoCtrlTask_(EAutoCtrlProcess::GROUND_ORE); }
            if(keyboard.key_R) { StartAutoCtrlTask_(EAutoCtrlProcess::PUSH_ORE); }
            if(keyboard.key_Q) { StartAutoCtrlTask_(EAutoCtrlProcess::POP_ORE); }
            */
        }
        if(keyboard.key_Shift && parm_->armInfo.isModuleAvailable){
            //if(keyboard.key_Z) { StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_ORIGIN); }   // Shift + Z 能量单元任务 
            // if(keyboard.key_C) { StartAutoCtrlTask_(EAutoCtrlProcess::DOGHOLE); }
            if(keyboard.key_C) { StartAutoCtrlTask_(EAutoCtrlProcess::CYCLE);}
        }
        /******************* 功能按键 *******************/
        if (parm_ && pchassis_) {
            // 左取矿
            if (controller.left_exchange) {
                exchange_side_ = EExchangeSide::LEFT;
                if (StartAutoCtrlTask_(EAutoCtrlProcess::EXCHANGE_ORE) != APP_OK) {
                    exchange_side_ = EExchangeSide::NONE;   ///< 启动失败回收预选，避免残留污染下次
                }
                controller.left_exchange = false;
            }
            // 右取矿
            if (controller.right_exchange) {
                exchange_side_ = EExchangeSide::RIGHT;
                if (StartAutoCtrlTask_(EAutoCtrlProcess::EXCHANGE_ORE) != APP_OK) {
                    exchange_side_ = EExchangeSide::NONE;
                }
                controller.right_exchange = false;
            }
            // 自动兑矿
            if (controller.auto_exchange) {
                exchange_side_ = EExchangeSide::AUTO;
                if (StartAutoCtrlTask_(EAutoCtrlProcess::STORE_ORE) != APP_OK) {
                    exchange_side_ = EExchangeSide::NONE;
                }
                controller.auto_exchange = false;
            }
            // 大陀螺模式
            static bool last_cycle = false;
            bool cycle_rising = controller.cycle && !last_cycle;
            last_cycle = controller.cycle;
            if (cycle_rising) {
                if (isCycleActive_) {
                    isCycleActive_ = false;             ///< 退出
                } else if (currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE) {
                    isCycleActive_ = true;              ///< 启动
                    StartAutoCtrlTask_(EAutoCtrlProcess::CYCLE);
                }
            }
            controller.cycle = false;
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

    auto &robotdata = SysControllerLink.robotInfo;

    auto &keyboard_edge = SysRemote.remoteInfo.keyboard_edge;

    SysControllerLink.robotInfo.controlled_by_controller = true;

    // 夹爪控制：Core层仅传递标志位，速度渐变由组件层处理
        // 检测是否正在进行模式切换（Z+X同时按住），切换期间冻结夹爪防止意外松开
    bool mode_switching = SysRemote.remoteInfo.keyboard.key_Z
                           && SysRemote.remoteInfo.keyboard.key_X;

    // 线性插值器（25Hz数据  1000Hz控制，周期 = 40步）
    static CAlgoLinearInterp interp_yaw(40), interp_p1(40), interp_p2(40),
                             interp_roll(40), interp_end_pitch(40);
    // 上一次控制器原始数据，用于检测数据更新
    static CSystemControllerLink::SArmAngles last_arm;
    static bool last_isCustomCtrl = false;
    const bool cur_isCustomCtrl = parm_ ? parm_->armCmd.isCustomCtrl : false;
    if (parm_ && cur_isCustomCtrl && !last_isCustomCtrl) {
        interp_yaw.setTarget(parm_->armCmd.set_angle_Yaw, Round(controller.arm.yaw));
        interp_p1.setTarget(parm_->armCmd.set_angle_Pitch1, Round(controller.arm.pitch1));
        interp_p2.setTarget(parm_->armCmd.set_angle_Pitch2, Round(controller.arm.pitch2));
        interp_roll.setTarget(parm_->armCmd.set_angle_Roll, Round(-controller.arm.roll));
        interp_end_pitch.setTarget(parm_->armCmd.set_angle_end_pitch, Round(controller.arm.pitch_end));
        last_arm = controller.arm;
    }
    last_isCustomCtrl = cur_isCustomCtrl;


    // 将模块启动
    if (SysRemote.systemStatus == APP_OK) {
        StartRobot(false);
    }

    if (parm_) {
        parm_->should_limit_yaw = 0;
    }

    /******************* 底盘控制 *******************/

    // 平滑更新角速度
    if (pchassis_) {
        pchassis_->chassisCmd.speed_W *= 0.92f;
        if (abs(pchassis_->chassisCmd.speed_W) < 0.3f) pchassis_->chassisCmd.speed_W = 0.0f;//处理旋转前停留的w速度避免自旋

        pchassis_->chassisCmd.speed_W += static_cast<float_t>(keyboard.key_E - keyboard.key_Q) * 15.0f;
        pchassis_->chassisCmd.speed_W = std::clamp(pchassis_->chassisCmd.speed_W, -20.0f, 20.0f);
        
        // 鼠标移动过快则认为是误操作，切换回键盘控制
        if(abs(keyboard.mouse_X) > 350) {
            use_Controller_ = false;
            SysControllerLink.robotInfo.controlled_by_controller = false;
            return;
        }

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
            if(keyboard.key_B
             &&!keyboard.key_Ctrl){
                pchassis_->chassisCmd.L_length += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 0.01f;
                pgimbal_->gimbalCmd.set_pitch = GIMBAL_PITCH_INIT_ANGLE + pchassis_->chassisInfo.L_Length * 0.52f;// 同步抬升
            }
            if(keyboard.key_Ctrl
                && keyboard.key_B
                && keyboard.key_Shift
                && currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE) {
                pchassis_->reset_hip = !pchassis_->reset_hip;
                pgimbal_->gimbalCmd.set_pitch = 0.f;
                pgimbal_->gimbalCmd.set_visualyaw = 0.f;
            }
            if(keyboard_edge.key_F == CSystemRemote::ERemoteEdge::Rising
             &&keyboard_edge.key_G == CSystemRemote::ERemoteEdge::Rising) {
                pchassis_->chassisInfo.crawler_on = !pchassis_->chassisInfo.crawler_on;
            }
        }
    }


    /******************* 机械臂 *******************/
    // 使用控制器臂部数据控制机械臂
    if (parm_ && !parm_->armCmd.isAutoCtrl) {  ///< 自动控制期间跳过手动控制
        auto &arm = controller.arm;  // 单臂数据

        // 设定各轴插值，并且过滤死区
        if(fabs(arm.yaw    - last_arm.yaw ) > 0.1f)interp_yaw.setTarget(parm_->armCmd.set_angle_Yaw, Round(arm.yaw));
        if(fabs(arm.pitch1 - last_arm.pitch1) > 0.1f)interp_p1.setTarget(parm_->armCmd.set_angle_Pitch1, Round(arm.pitch1));
        if(fabs(arm.pitch2 - last_arm.pitch2) > 0.1f)interp_p2.setTarget(parm_->armCmd.set_angle_Pitch2, Round(arm.pitch2));
        if(fabs(arm.roll   - last_arm.roll   ) > 0.1f)interp_roll.setTarget(parm_->armCmd.set_angle_Roll, Round(-arm.roll));
        if(fabs(arm.pitch_end - last_arm.pitch_end) > 0.1f)interp_end_pitch.setTarget(parm_->armCmd.set_angle_end_pitch, Round(arm.pitch_end));
        last_arm = arm;
        // 每个控制周期执行插值
        parm_->armCmd.set_angle_Yaw    = interp_yaw.update();
        parm_->armCmd.set_angle_Pitch1 = interp_p1.update();
        parm_->armCmd.set_angle_Pitch2 = interp_p2.update();
        parm_->armCmd.set_angle_Roll   = interp_roll.update();
        parm_->armCmd.set_angle_end_pitch = interp_end_pitch.update();
        
        // const float alpha = 0.98f;  
        // parm_->armCmd.set_angle_Yaw       = LowPassFilter(parm_->armCmd.set_angle_Yaw,       arm.yaw,        alpha);
        // parm_->armCmd.set_angle_Pitch1    = LowPassFilter(parm_->armCmd.set_angle_Pitch1,    arm.pitch1,      alpha);
        // parm_->armCmd.set_angle_Pitch2    = LowPassFilter(parm_->armCmd.set_angle_Pitch2,    arm.pitch2,      alpha);
        // parm_->armCmd.set_angle_Roll      = LowPassFilter(parm_->armCmd.set_angle_Roll,      -arm.roll,       alpha);
        // parm_->armCmd.set_angle_end_pitch = LowPassFilter(parm_->armCmd.set_angle_end_pitch, arm.pitch_end,   alpha);

        // Yaw 最终物理限位
        parm_->armCmd.set_angle_Yaw = std::clamp(parm_->armCmd.set_angle_Yaw,
            ARM_YAW_PHYSICAL_RANGE_MIN, ARM_YAW_PHYSICAL_RANGE_MAX);

        // 末端roll轴
        parm_->armCmd.set_angle_end_roll += static_cast<float_t>(keyboard.key_G - keyboard.key_F) * 180.0f / freq;
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
        const bool regrip_requested = regrip_keyboardcom;
        if (regrip_requested) {
            parm_->armCmd.reGripCmd = true;                                  // 传递 re-grip 指令
        }
        //controller.gripper_regrip = false;  // 处理之后清除标志位
        regrip_keyboardcom = false;

        if(!mode_switching && keyboard_edge.key_C == CSystemRemote::ERemoteEdge::Rising){
            gripKeyboardCmd_ = (gripKeyboardCmd_ == EGripKeyboardCmd::CLOSE)
                ? EGripKeyboardCmd::OPEN
                : EGripKeyboardCmd::CLOSE;
        }
        const bool grip_close_cmd = gripKeyboardCmd_ == EGripKeyboardCmd::CLOSE;
        const bool grip_open_cmd = gripKeyboardCmd_ == EGripKeyboardCmd::OPEN;
        if (mode_switching) {
             // 模式切换期间冻结夹爪，避免 Z+X 切换被解释成张开
             parm_->armCmd.gripClose = false;
             parm_->armCmd.gripOpen = false;
        } else if (grip_close_cmd) {
             parm_->armCmd.gripClose = true;
             parm_->armCmd.gripOpen = false;
             // isGripped 判断由组件层 HOLD 状态自动处理
        } else if (grip_open_cmd) {
             parm_->armCmd.gripClose = false;
             parm_->armCmd.gripOpen = true;
        } else {
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
        // if((keyboard_edge.key_Z == CSystemRemote::ERemoteEdge::Rising 
        //     && keyboard_edge.key_Ctrl== CSystemRemote::ERemoteEdge::Rising 
        //     //&& keyboard_edge.key_Shift== CSystemRemote::ERemoteEdge::Rising 
        //     &&currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE)){
        //     parm_->armCmd.resetEndPitch = true;
        // }
        // if((keyboard.key_Z
        //     && keyboard.key_Ctrl
        //     &&currentAutoCtrlProcess_ == EAutoCtrlProcess::NONE)){
        //     parm_->armCmd.resetEndPitch = true;
        // }

    // 云台yaw
    if (pgimbal_) {
        if(!pgimbal_->gimbalCmd.isAutoCtrl && !pgimbal_->gimbalInfo.isIntoControll){  // 进入自定义控制器控制时云台自动归位
            pgimbal_->gimbalCmd.set_visualyaw = GIMBAL_VISUAL_MOTOR_INIT_ANGLE;  // 进入自定义控制器控制时云台自动归位
            pgimbal_->gimbalInfo.isIntoControll = true;
        }
        //pgimbal_->gimbalCmd.set_visualyaw += static_cast<float_t>(keyboard.mouse_Y - keyboard.mouse_X) * 1.0f / freq;
    }



/*删除自定义控制器对应的兑矿操作
    if (psubgantry_) {
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
    }
*/
    // }
    last_key_F = SysRemote.remoteInfo.keyboard.key_F;
    last_key_G = SysRemote.remoteInfo.keyboard.key_G;
    // last_rocker_key_status = controller.Rocker_Key

    
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
