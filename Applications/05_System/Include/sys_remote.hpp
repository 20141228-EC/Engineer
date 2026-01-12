/**
 * @file sys_remote.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 定义遥控器系统
 * @version 1.0
 * @date 2026-01-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef SYS_REMOTE_HPP
#define SYS_REMOTE_HPP

#include "sys_common.hpp"
#include "Device.hpp"

#define GET_SIGN(value)  ((value > 0) - (value < 0))

namespace my_engineer {

/**
 * @brief 遥控器系统类
 * 
 */
class CSystemRemote final: public CSystemBase{
public:

    // 定义遥控器系统初始化参数结构体
    struct SSystemInitParam_Remote: public SSystemInitParam_Base{
        EDeviceID remoteDevID = EDeviceID::DEV_NULL; ///< 遥控器设备ID
        //EDeviceID refereeDevID = EDeviceID::DEV_NULL; ///< 裁判系统设备ID
    };

    // Remote信息结构体
    struct SRemoteInfo {
        float_t joystick_LX = 0;    ///< 左摇杆x轴
        float_t joystick_LY = 0;    ///< 左摇杆y轴
        float_t joystick_RX = 0;    ///< 右摇杆x轴
        float_t joystick_RY = 0;    ///< 右摇杆y轴
        float_t thumbWheel = 0;     ///< 拨轮（向上负，向下正）
        uint8_t switch_L = 0;       ///< 左拨杆
        uint8_t switch_R = 0;       ///< 右拨杆
    };

    // 边沿信息枚举变量
    enum class ERemoteEdge{
        RESET = -1, ///< 重置
        NONE,       ///< 无边沿
        Rising,     ///< 上升沿(0-1)
        Falling,    ///< 下降沿(1-0)
    };
    
    // Remote边沿信息结构体
    struct SRemoteEdge{
        ERemoteEdge joystick_LX = ERemoteEdge::RESET;   ///< 左摇杆x轴
        ERemoteEdge joystick_LY = ERemoteEdge::RESET;   ///< 左摇杆y轴
        ERemoteEdge joystick_RX = ERemoteEdge::RESET;   ///< 右摇杆x轴
        ERemoteEdge joystick_RY = ERemoteEdge::RESET;   ///< 右摇杆y轴
        ERemoteEdge thumbWheel = ERemoteEdge::RESET;    ///< 拨轮
        ERemoteEdge switch_L = ERemoteEdge::RESET;    ///< 左摇杆
        ERemoteEdge switch_R = ERemoteEdge::RESET;    ///< 右摇杆
    };

    // Keyboard信息结构体
    struct SKeyboardInfo {
        int16_t mouse_X = 0;
        int16_t mouse_Y = 0;
        int16_t mouse_Thumb = 0;
        bool mouse_L = false;   ///< 鼠标左键
        bool mouse_R = false;   ///< 鼠标右键
        bool key_W = false;     ///< W键
        bool key_A = false;     ///< A键
        bool key_S = false;     ///< S键
        bool key_D = false;     ///< D键
        bool key_Q = false;     ///< Q键
        bool key_E = false;     ///< E键
        bool key_R = false;     ///< R键
        bool key_F = false;     ///< F键
        bool key_G = false;     ///< G键
        bool key_Z = false;     ///< Z键
        bool key_X = false;     ///< X键
        bool key_C = false;     ///< C键
        bool key_V = false;     ///< V键
        bool key_B = false;     ///< B键
        bool key_Ctrl = false;  ///< ctrl键
        bool key_Shift = false; ///< shift键
    };

    // 定义遥控器信息包结构体并实例化
    struct SremoteInfoPackage {
        SRemoteInfo remote;
        SRemoteEdge remote_edge;
        SKeyboardInfo keyboard;
    } remoteInfo;

    // 初始化系统
    EAppStatus InitSystem(SSystemInitParam_Base *pStruct) final;

    // 设置遥控器死区
    void SetRemoteDeadZone(float_t deadZone);

    // 复位标志
    bool ResetFlag = false;

private:

    // 遥控器设备指针
    CRcBase *pRemoteDev_ = nullptr;

    // 裁判系统设备指针
    // CDevReferee *pRefereeDev_ = nullptr;

    // 更新处理
    void UpdateHandler_() final;

    // 心跳处理
    void HeartbeatHandler_() final;

    // 更新遥控器
    EAppStatus UpdateRemote_();

    // 更新键盘
    EAppStatus UpdateKeyboard_();

    // 更新裁判系统
    // EAppStatus UpdateReferee_();

    // 遥控器死区
    float_t remoteDeadZone_ = 0;

    EAppStatus UpdateRemote_with_deadzone_();

    EAppStatus UpdateRemote_Edge_();

};

extern CSystemRemote SysRemote;

}   // namespace my_engineer

#endif // SYS_REMOTE_HPP
