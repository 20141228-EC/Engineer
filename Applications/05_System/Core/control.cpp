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

// int16_t 
namespace my_engineer {


namespace {

struct SPlanarVector {
    float x;
    float y;
};

enum class ESpinState {
    OFF = 0,
    SPIN,
};

struct SGyroSpinRuntimeState {
    float gyroFollowWFiltered = 0.0f;        // 陀螺仪跟随角速度滤波值
    ESpinState spinState = ESpinState::OFF;  // 小陀螺状态
    bool lastSpinHotkeyPressed = false;      // 上次小陀螺热键状态
    bool lastModeActive = false;             // 上次小陀螺模式状态（用于检测模式切换）
    float virtualGimbalAbsYawDeg = 0.0f;     // 虚拟云台绝对偏航角（度）
    float spinTranslationCompDeg = 0.0f;     // 小陀螺运动补偿角（度）
    float spinYawRateDegFiltered = 0.0f;     // 小陀螺yaw角速度滤波值（度/s）
};

inline SPlanarVector RotateVectorRad(float x, float y, float angleRad) {
    return {
        x * cosf(angleRad) - y * sinf(angleRad),
        x * sinf(angleRad) + y * cosf(angleRad)
    };
}

// 旋转向量，angleDeg为正时逆时针旋转，为负时顺时针旋转
inline SPlanarVector RotateVectorDeg(float x, float y, float angleDeg) {
    constexpr float kDegToRad = 3.14159265f / 180.0f;
    return RotateVectorRad(x, y, angleDeg * kDegToRad);
}

inline float WrapDeg180(float deg) {
    while (deg > 180.f) deg -= 360.f;
    while (deg < -180.f) deg += 360.f;
    return deg;
}

inline float kGyroFollowLimited(float speed){
    if(speed <= 80.0f)return 1.0f;
    else {
        float scale = 1.0f - (speed - 80.0f) / 20.0f * (1.0 - 0.9f);
        return (scale > 0.0f) ? scale : 0.0f;
    }
}
// 小陀螺模式下根据陀螺仪角速度对遥控器输入进行补偿，减少旋转时的控制死区
inline SPlanarVector ApplySpinMoveBias(const SPlanarVector &gimbalVec, ESpinState spinState,
                                       float spinMoveBiasDeg, float spinSpeedW) {
    if (spinState != ESpinState::SPIN) {
        return gimbalVec;
    }

    const float spinBiasDeg = spinMoveBiasDeg * (spinSpeedW >= 0.0f ? 1.0f : -1.0f);
    return RotateVectorDeg(gimbalVec.x, gimbalVec.y, spinBiasDeg);
}

// 获取小陀螺模式下的陀螺仪yaw角速度，若IMU不可用则使用后备值
inline float GetSpinYawRateDeg(CAlgo_IMU_Ave *filter, float fallbackYawRateDeg) {
    if (filter && filter->mems && filter->mems->memsStatus == CMemsBase::EMemsStatus::NORMAL) {
        constexpr float kRadToDeg = 57.2957795f; // 180 / pi
        return filter->mems->memsData[CMemsBase::DATA_GYRO_Z] * kRadToDeg;
    }
    return fallbackYawRateDeg;
}

inline float UpdateSpinMoveCompDeg(float spinTranslationCompDeg, float &spinYawRateDegFiltered,
                                   bool useVirtualGimbal, ESpinState spinState,
                                   CAlgo_IMU_Ave *filter, float fallbackYawRateDeg, float freq) {
    if (!(useVirtualGimbal && spinState == ESpinState::SPIN)) {
        spinTranslationCompDeg = 0.0f;
        spinYawRateDegFiltered = 0.0f;
        return spinTranslationCompDeg;
    }

    const float spinYawRateDeg = GetSpinYawRateDeg(filter, fallbackYawRateDeg);
    spinYawRateDegFiltered = 0.35f * spinYawRateDeg + 0.65f * spinYawRateDegFiltered;
    return WrapDeg180(spinTranslationCompDeg - spinYawRateDegFiltered / freq);
}

inline void ResetGyroSpinRuntimeState(SGyroSpinRuntimeState &state, bool resetModeFlag) {
    state.gyroFollowWFiltered = 0.0f;
    state.spinState = ESpinState::OFF;
    state.lastSpinHotkeyPressed = false;
    state.spinTranslationCompDeg = 0.0f;
    state.spinYawRateDegFiltered = 0.0f;
    if (resetModeFlag) {
        state.lastModeActive = false;
    }
}

inline void ApplyGyroSpinChassisControl(CModChassis *chassis,
                                        const SPlanarVector &gimbalVec,
                                        float yawCmd,
                                        bool gyroModeActive,
                                        bool spinToggleEvent,
                                        bool spinHotkeyPressed,
                                        SGyroSpinRuntimeState &state,
                                        float freq) {
    if (!chassis) {
        return;
    }

    float kGyroFollowKp = 5.0f; // 跟随kp
    constexpr float kDeadZoneDeg = 1.0f;  // yaw死区，避免陀螺仪数据不稳定时小幅度抖动
    constexpr float kWAlpha = 0.20f;      // 陀螺跟随角速度滤波系数，越大响应越快但抖动越明显
    constexpr float kSpinSpeedW = 80.0f;  // 小陀螺旋转角速度（度/s）
    constexpr float kSpinMoveBiasDeg = 10.0f;// 小陀螺运动补偿角度（度），用于抵消旋转时的控制死区，提升小陀螺状态下的操控性

    const bool imuYawValid = (chassis->filter && chassis->filter->Imu_Ave_Info.is_initialized);
    if (!gyroModeActive) {
        ResetGyroSpinRuntimeState(state, true);
        chassis->spin_on = false;
        return;
    }

    if (!state.lastModeActive && imuYawValid) {
        state.virtualGimbalAbsYawDeg = chassis->filter->Imu_Ave_Info.imu_ave_yaw;
    }

    const bool gimbalDataValid = false;
    const bool shouldUseVirtualGimbal = (!gimbalDataValid && imuYawValid);

    float gimbalYawDeg = 0.0f;
    bool gimbalYawUsable = false;

    if (shouldUseVirtualGimbal) {
        const float chassisYawDeg = chassis->filter->Imu_Ave_Info.imu_ave_yaw;
        state.virtualGimbalAbsYawDeg = WrapDeg180(state.virtualGimbalAbsYawDeg + yawCmd / freq);
        gimbalYawDeg = WrapDeg180(state.virtualGimbalAbsYawDeg - chassisYawDeg);
        gimbalYawUsable = true;
    }

    if (spinToggleEvent || (spinHotkeyPressed && !state.lastSpinHotkeyPressed)) {
        state.spinState = (state.spinState == ESpinState::SPIN) ? ESpinState::OFF : ESpinState::SPIN;
        state.gyroFollowWFiltered = 0.0f;
        if (state.spinState != ESpinState::SPIN) {
            state.spinTranslationCompDeg = 0.0f;
            state.spinYawRateDegFiltered = 0.0f;
        }
    }
    state.lastSpinHotkeyPressed = spinHotkeyPressed;
    chassis->spin_on = (state.spinState == ESpinState::SPIN);

    const auto biasedGimbalVec = ApplySpinMoveBias(gimbalVec, state.spinState, kSpinMoveBiasDeg, kSpinSpeedW);

    if (gimbalYawUsable) {
        state.spinTranslationCompDeg = UpdateSpinMoveCompDeg(state.spinTranslationCompDeg,
                                                             state.spinYawRateDegFiltered,
                                                             shouldUseVirtualGimbal,
                                                             state.spinState,
                                                             chassis->filter,
                                                             kSpinSpeedW,
                                                             freq);

        const float moveYawDeg = WrapDeg180(gimbalYawDeg + state.spinTranslationCompDeg);
        const auto chassisVec = RotateVectorDeg(biasedGimbalVec.x, biasedGimbalVec.y, moveYawDeg);
        chassis->chassisCmd.speed_X = chassisVec.x;
        chassis->chassisCmd.speed_Y = chassisVec.y;
    } else {
        chassis->chassisCmd.speed_X = biasedGimbalVec.x;
        chassis->chassisCmd.speed_Y = biasedGimbalVec.y;
    }

    if (state.spinState == ESpinState::SPIN) {
        chassis->chassisCmd.speed_W = kSpinSpeedW;
    } else if (gimbalYawUsable) {
        float yawErrorDeg = WrapDeg180(-gimbalYawDeg);

        if (fabsf(yawErrorDeg) < kDeadZoneDeg) {
            yawErrorDeg = 0.0f;
        }
        kGyroFollowKp *= kGyroFollowLimited(sqrtf(chassis->chassisCmd.speed_X * chassis->chassisCmd.speed_X + chassis->chassisCmd.speed_Y * chassis->chassisCmd.speed_Y));
        float gyroFollowW = kGyroFollowKp * yawErrorDeg;
        gyroFollowW = std::clamp(gyroFollowW, -100.f, 100.f);
        state.gyroFollowWFiltered = kWAlpha * gyroFollowW + (1 - kWAlpha) * state.gyroFollowWFiltered;
        chassis->chassisCmd.speed_W = state.gyroFollowWFiltered;
    } else {
        state.gyroFollowWFiltered = 0.0f;
        chassis->chassisCmd.speed_W = yawCmd;
    }

    chassis->MovMode = CModChassis::EmovMode::NORMAL;
    state.lastModeActive = true;
}

} // namespace

void CSystemCore::StartRobot(bool if_remote_control, bool I_dont_have_a_remote) {

    if (pchassis_) {
        if (!pchassis_->chassisInfo.isModuleAvailable               ///<说明模块已经注册了
            && pchassis_->moduleStatus == APP_OK) {
            pchassis_->StartModule();                               ///<在创建任务的时候还会再调用一次初始化函数
        }
    }

    // if(if_remote_control) {
    //     enum { HIG = 1, LOW = 2, MID = 3 };
    //     auto &remote = SysRemote.remoteInfo.remote;

    //     static uint8_t last_switch_L = LOW; // 遥控器系统层的数据更新比设备层的状态更新要慢

    //     if (parm_) {
    //         if (!parm_->armInfo.isModuleAvailable
    //             && parm_->moduleStatus == APP_OK
    //             && remote.switch_L == HIG && last_switch_L != HIG) { 
    //             parm_->StartModule();       ///< 左拨杆拨到上时初始化臂
    //         }
    //     }

    //     last_switch_L = remote.switch_L;        ///<记录上一次的左拨杆状态

    // }

    // else {
    //     auto &keyboard = SysRemote.remoteInfo.keyboard;

    //     if (parm_) {
    //         if (!parm_->armInfo.isModuleAvailable
    //             && parm_->moduleStatus == APP_OK
    //             && keyboard.key_Ctrl && keyboard.key_Shift && keyboard.key_F) { ///< ctrl+shift+f 初始化臂
    //             parm_->StartModule();
    //         }
    //     }
    // }

    // if(I_dont_have_a_remote) {
    //     if (parm_) {
    //         if (!parm_->armInfo.isModuleAvailable
    //             && parm_->moduleStatus == APP_OK) {
    //             parm_->StartModule();
    //         }
    //     }
    // }
    
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
    auto &keyboard = SysRemote.remoteInfo.keyboard;
    auto getRemoteYawCmd = [&remote]() -> float {
        constexpr float kYawCmdDeadZone = 6.0f;
        const float yawCmd = remote.joystick_RX;
        return (fabsf(yawCmd) < kYawCmdDeadZone) ? 0.0f : yawCmd;
    };
    static SGyroSpinRuntimeState gyroSpinState;
    const bool remoteOnline = (SysRemote.systemStatus == APP_OK);
    const bool isGyroSpinMode = (remote.switch_L == HIG && remote.switch_R == LOW);
    static bool lastRemoteOnline = false;

    // 遥控恢复在线时，强制退出小陀螺（spin），回到陀螺仪跟随状态。
    if (!lastRemoteOnline && remoteOnline) {
        ResetGyroSpinRuntimeState(gyroSpinState, true);
    }
    lastRemoteOnline = remoteOnline;

    if (!remoteOnline) {
        ResetGyroSpinRuntimeState(gyroSpinState, true);
        return;
    }

    if (remoteWasOffline_) {
        remoteWasOffline_ = false;
        ResetGyroSpinRuntimeState(gyroSpinState, true);
    }

    if (!isGyroSpinMode) {
        ResetGyroSpinRuntimeState(gyroSpinState, true);
    }
    //将模块启动
    if (remoteOnline) {
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

    // LOW + MID 底盘控制(轮毂+髋)
    if (remote.switch_L == LOW && remote.switch_R == MID) {
        SysRemote.SetRemoteDeadZone(10.f);
        // 底盘控制
        if (pchassis_) {
            if(!pchassis_->chassisCmd.isAutoCtrl){
                pchassis_->chassisCmd.speed_X = remote.joystick_LX / 2;             ///<摇杆的x方向控制车的左右移动，为了保证操作手的手感减小左右方向的速度
                pchassis_->chassisCmd.speed_Y = remote.joystick_LY;
                pchassis_->chassisCmd.speed_W = getRemoteYawCmd();
                pchassis_->chassisCmd.L_length += (remote.joystick_RY / 250.f) * 90.f / freq; ///< 腿长采用增量式控制
                pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
                
            if(remote_edge.thumbWheel == CSystemRemote::ERemoteEdge::Falling){
                pchassis_->reset_hip = !pchassis_->reset_hip;   ///< 要求复位腿
                }
            if(remote_edge.thumbWheel == CSystemRemote::ERemoteEdge::Rising){
                pchassis_->crawler_on = !pchassis_->crawler_on; ///< 启动履带电机
                }
            // if(pchassis_->filter->Imu_Ave_Info.imu_ave_pitch >= 23.f){
            //     parm_->armCmd.set_angle_Pitch1 = 4.f;
            //     parm_->armCmd.set_angle_Pitch2 = 11.f;  
            // }

            }    
        }
    }

    // MID + HIG 主臂关节四轴 + 夹爪
    // else if (remote.switch_L == MID && remote.switch_R == HIG) {
    //     SysRemote.SetRemoteDeadZone(10.f);
    //     if (parm_) {
    //         if(!parm_->armCmd.isAutoCtrl){
    //             parm_->armCmd.set_angle_Yaw +=
    //                 (remote.joystick_LX / 100.f) * 90.f / freq;
    //             parm_->armCmd.set_angle_Pitch1 +=
    //                 (remote.joystick_LY / 100.f) * 90.f / freq;
    //             parm_->armCmd.set_angle_Pitch2 +=
    //                 (remote.joystick_RY / 100.f) * 90.f / freq;
    //             parm_->armCmd.set_angle_Roll +=
    //                 (remote.joystick_RX / 100.f) * 90.f / freq;
    //             parm_->armCmd.set_length_grip +=
    //                 (remote.thumbWheel / 100.f) * 60.f / freq; ///< 拨轮控夹爪
    //         }
    //     }
    //     if(pchassis_){
    //         pchassis_->MovMode = CModChassis::EmovMode::NORMAL; 
    //     }
    // }

    // MID + MID 副臂关节四轴 + 夹爪
    // if (remote.switch_L == MID && remote.switch_R == MID) {
    //     SysRemote.SetRemoteDeadZone(10.f);
    //     ///< 此处不执行任何操作，由板间通信将整个遥控器数据传给副板，副板自己执行控制逻辑
    //     if(pchassis_){
    //         if(!pchassis_->chassisCmd.isAutoCtrl){
    //             pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
    //         } 
    //     }
    // }

    // MID + LOW 左摇杆主臂末端，右摇杆副臂末端
    // else if (remote.switch_L == MID && remote.switch_R == LOW) {
    //     SysRemote.SetRemoteDeadZone(10.f);
    //     if (parm_) {
    //         if(!parm_->armCmd.isAutoCtrl){
    //             parm_->armCmd.set_angle_end_pitch +=
    //                 (remote.joystick_LY / 100.f) * 300.f / freq;
    //             parm_->armCmd.set_angle_end_roll +=
    //                 (remote.joystick_LX / 100.f) * 120.f / freq;
    //         }
    //             // 只写了主臂末端的控制，副臂的目标设置在副板代码中
    //     }
    //     if(pchassis_){
    //         if(!pchassis_->chassisCmd.isAutoCtrl){
    //             pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
    //         } 
    //     }
    // }

    // HIG + MID 自动上台阶 利用陀螺仪数据控腿长
    else if(remote.switch_L == HIG && remote.switch_R == MID)
    {
        SysRemote.SetRemoteDeadZone(10.f);
        // 底盘控制
        if (pchassis_) {
            if(!pchassis_->chassisCmd.isAutoCtrl){
                pchassis_->chassisCmd.speed_X = remote.joystick_LX / 2;             ///<摇杆的x方向控制车的左右移动，为了保证操作手的手感减小左右方向的速度
                pchassis_->chassisCmd.speed_Y = remote.joystick_LY;
                pchassis_->chassisCmd.speed_W = getRemoteYawCmd();
                pchassis_->MovMode = CModChassis::EmovMode::CLIMBING;               ///< 更新模块运动模式标志位
            }

            static uint8_t thumbwheel_count = 0;

            if(remote_edge.thumbWheel == CSystemRemote::ERemoteEdge::Falling){
                pchassis_->reset_hip = !pchassis_->reset_hip;   ///< 要求复位腿
            }
            if(remote_edge.thumbWheel == CSystemRemote::ERemoteEdge::Rising){
                pchassis_->crawler_on = !pchassis_->crawler_on; ///< 启动履带电机
            }

            ///< 右摇杆y控云台pitch，逻辑在副板
        }
    }
    // HIG + LOW 由板间通信下发底盘速度
    else if(remote.switch_L == HIG && remote.switch_R == LOW)
    {
        if(pchassis_){
            if(!pchassis_->chassisCmd.isAutoCtrl){
                const bool boardLinkCtrlValid = (SysBoardLink.ctrlInfos.pack_id == CDevBoardLink::PKT_CTRL_INFOS
                    && SysBoardLink.ctrlInfos.remote_is_online == 1);
                if (boardLinkCtrlValid) {
                    pchassis_->chassisCmd.speed_X = static_cast<float>(SysBoardLink.ctrlInfos.speed_x);
                    pchassis_->chassisCmd.speed_Y = static_cast<float>(SysBoardLink.ctrlInfos.speed_y);
                    pchassis_->chassisCmd.speed_W = static_cast<float>(SysBoardLink.ctrlInfos.speed_w);
                    pchassis_->spin_on = (SysBoardLink.otherInfo.pack_id == CDevBoardLink::PKT_OTHER_INFOS
                        && SysBoardLink.otherInfo.is_spin_on != 0);
                    pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
                    ResetGyroSpinRuntimeState(gyroSpinState, false);
                } else {
                    const float remoteYawCmd = -getRemoteYawCmd();
                    ApplyGyroSpinChassisControl(pchassis_,
                                                {remote.joystick_LX / 2, remote.joystick_LY},
                                                remoteYawCmd,
                                                true,
                                                remote_edge.thumbWheel == CSystemRemote::ERemoteEdge::Rising,
                                                keyboard.key_Ctrl && keyboard.key_F,
                                                gyroSpinState,
                                                freq);
                }
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
    static SGyroSpinRuntimeState keyboardGyroSpinState;
    static float keyboardGimbalSpeedX = 0.0f;
    static float keyboardGimbalSpeedY = 0.0f;
    static float keyboardYawCmdFiltered = 0.0f;
    static uint8_t lastMouseStatus_L = 0;
    static uint8_t lastMouseStatus_R = 0;
    static uint32_t lastKeyboardCtrlTick = 0U;

    const uint32_t now = HAL_GetTick();
    if (lastKeyboardCtrlTick != 0U && (now - lastKeyboardCtrlTick) > 50U) {
        ResetGyroSpinRuntimeState(keyboardGyroSpinState, true);
        keyboardGimbalSpeedX = 0.0f;
        keyboardGimbalSpeedY = 0.0f;
        keyboardYawCmdFiltered = 0.0f;
    }
    lastKeyboardCtrlTick = now;

    // 将模块启动
    if (SysRemote.systemStatus == APP_OK) {
        StartRobot(false);                  ///<转换为键盘操作
    }

    // parm_->should_limit_yaw = 1;

    /******************* 底盘控制 *******************/

    // 平滑更新角速度
    if (pchassis_) {
        keyboardYawCmdFiltered = keyboardYawCmdFiltered +
            0.03f * (keyboard.mouse_X - keyboardYawCmdFiltered);

        if (!pchassis_->chassisCmd.isAutoCtrl)
        {
            keyboardGimbalSpeedX *= 0.97f;
            keyboardGimbalSpeedY *= 0.98f;
            if (abs(keyboardGimbalSpeedX) < 0.5f) keyboardGimbalSpeedX = 0.0f;
            if (abs(keyboardGimbalSpeedY) < 0.5f) keyboardGimbalSpeedY = 0.0f;

            if (keyboard.key_Shift) {
                keyboardGimbalSpeedX += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 5.0f;   ///<通过差值来实现一行代码实现左右转弯
                keyboardGimbalSpeedY += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 5.0f;
                keyboardGimbalSpeedX = std::clamp(keyboardGimbalSpeedX, -50.0f, 50.0f);
                keyboardGimbalSpeedY = std::clamp(keyboardGimbalSpeedY, -100.0f, 100.0f);
            } else {
                keyboardGimbalSpeedX += static_cast<float_t>(keyboard.key_D - keyboard.key_A) * 1.0f;
                keyboardGimbalSpeedY += static_cast<float_t>(keyboard.key_W - keyboard.key_S) * 1.0f;
                keyboardGimbalSpeedX = std::clamp(keyboardGimbalSpeedX, -20.0f, 20.0f);
                keyboardGimbalSpeedY = std::clamp(keyboardGimbalSpeedY, -30.0f, 30.0f);
            }

            ApplyGyroSpinChassisControl(pchassis_,
                                        {keyboardGimbalSpeedX, keyboardGimbalSpeedY},
                                        -keyboardYawCmdFiltered,
                                        true,
                                        false,
                                        keyboard.key_F,
                                        keyboardGyroSpinState,
                                        freq);

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

    // /******************* 云台手动控制 *******************/
    // // 通过G键+鼠标左右键控抬升
    // // 通过鼠标y轴速度控pitch

    // /******************* 机械臂手动控制 *******************/
    // // 按下ctrl时控副臂，否则控主臂
    // if (parm_) {
    //     if (!parm_->armCmd.isAutoCtrl) {
    //         // !SysBoardLink.pArm_Cmd->isAutoCtrl) {
    //         // yaw(Q键)
    //         if(keyboard.key_Q)
    //             parm_->armCmd.set_angle_Yaw += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 60.0f / freq;
    //         // pitch1(E键)
    //         if(keyboard.key_E)
    //             parm_->armCmd.set_angle_Pitch1 += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 70.0f / freq;
    //         // pitch2(R键)
    //         if(keyboard.key_R)
    //             parm_->armCmd.set_angle_Pitch2 += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 70.0f / freq;
    //         // roll(Z键)
    //         if(keyboard.key_Z)
    //             parm_->armCmd.set_angle_Roll += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 80.0f / freq;
    //         // end_pitch(X键)
    //         if(keyboard.key_X)
    //             parm_->armCmd.set_angle_end_pitch += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 90.0f / freq;
    //         // end_roll(C键)
    //         if(keyboard.key_C)
    //             parm_->armCmd.set_angle_end_roll += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 90.0f / freq;
    //     }
    // }

    lastMouseStatus_L = keyboard.mouse_L;
    lastMouseStatus_R = keyboard.mouse_R;


    // /******************* 自动控制 *******************/
    // if (parm_ && pchassis_) {
    //     if (keyboard.key_Ctrl && parm_->armInfo.isModuleAvailable)
    //     {
    //         // Ctrl + V: 停止所有自动任务
    //         if(keyboard.key_V)
    //         {
    //             StopAutoCtrlTask_();
    //         }

    //         // Ctrl + C: 启动上台阶任务
    //         if(keyboard.key_C)
    //         {
    //             StartAutoCtrlTask_(EAutoCtrlProcess::CLIMBING);
    //         }

    //         /* --- 其他旧的自动任务快捷键已被移除 ---
    //         if(keyboard.key_G) { StartAutoCtrlTask_(EAutoCtrlProcess::GOLD_ORE); }
    //         if(keyboard.key_X) { StartAutoCtrlTask_(EAutoCtrlProcess::SILVER_ORE); }
    //         if(keyboard.key_F) { StartAutoCtrlTask_(EAutoCtrlProcess::GROUND_ORE); }
    //         if(keyboard.key_R) { StartAutoCtrlTask_(EAutoCtrlProcess::PUSH_ORE); }
    //         if(keyboard.key_Q) { StartAutoCtrlTask_(EAutoCtrlProcess::POP_ORE); }
    //         */
    //     }
    //     /* --- 其他旧的自动任务快捷键已被移除 ---
    //     if(keyboard.key_Shift && parm_->armInfo.isModuleAvailable){
    //         if(keyboard.key_Z) { StartAutoCtrlTask_(EAutoCtrlProcess::RETURN_DRIVE); }
    //         if(keyboard.key_C) { StartAutoCtrlTask_(EAutoCtrlProcess::DOGHOLE); }
    //     }
    //     */
    // }
    }
}

/**
 * @brief 自定义控制器操作（当前仅保留底盘相关依赖）
 */
void CSystemCore::ControlFromController_() {
    if (pchassis_ && !pchassis_->chassisCmd.isAutoCtrl) {
        pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
    }
}

void CSystemCore::ControlFromEsp32_() {
    if (pchassis_ && !pchassis_->chassisCmd.isAutoCtrl) {
        pchassis_->MovMode = CModChassis::EmovMode::NORMAL;
    }
}


}   // namespace my_engineer
