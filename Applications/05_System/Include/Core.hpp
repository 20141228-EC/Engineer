/**
 * @file Core.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 系统核心头文件
 * @version 1.0
 * @date 2025-12-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef CORE_HPP
#define CORE_HPP

#include "Interface.hpp"
#include "Device.hpp"
#include "Module.hpp"
#include "System.hpp"
#include "algo_other.hpp"

#define I_AM_CONTROLLER 0 // 当前板子是控制器

/*-------------------------------------AUTO_PROCESS_SET----------------------------------------------------------*/

// 上台阶
#define CLIMBING_YAW_ANGLE        ARM_YAW_INIT_ANGLE
#define CLIMBING_PITCH1_ANGLE     ARM_PITCH1_INIT_ANGLE
#define CLIMBING_PITCH2_ANGLE     ARM_PITCH2_INIT_ANGLE
#define CLIMBING_ROLL_ANGLE       ARM_ROLL_INIT_ANGLE
#define CLIMBING_END_PITCH_ANGLE  ARM_END_PITCH_INIT_ANGLE
#define CLIMBING_END_ROLL_ANGLE   ARM_END_ROLL_INIT_ANGLE
#define CLIMBING_GRIP_LENGTH      ARM_GRIP_INIT_LENGTH
#define CLIMBING_SPEED            80.f      ///< 全速的80%

// 抓能量单元
#define GRAB_ENERGY_UNIT_YAW_ANGLE        1.0f
#define GRAB_ENERGY_UNIT_PITCH1_ANGLE     1.0f
#define GRAB_ENERGY_UNIT_PITCH2_ANGLE     1.0f
#define GRAB_ENERGY_UNIT_ROLL_ANGLE       1.0f
#define GRAB_ENERGY_UNIT_END_PITCH_ANGLE  1.0f
#define GRAB_ENERGY_UNIT_END_ROLL_ANGLE   1.0f
#define GRAB_ENERGY_UNIT_GRIP_LENGTH      1.0f
// 待改

// 兑换矿石
#define EXCHANGE_ORE_YAW_ANGLE        1.0f
#define EXCHANGE_ORE_PITCH1_ANGLE     1.0f
#define EXCHANGE_ORE_PITCH2_ANGLE     1.0f
#define EXCHANGE_ORE_ROLL_ANGLE       1.0f
#define EXCHANGE_ORE_END_PITCH_ANGLE  1.0f
#define EXCHANGE_ORE_END_ROLL_ANGLE   1.0f
#define EXCHANGE_ORE_GRIP_LENGTH      1.0f
// 待改

// 存矿
#define SAVE_ORE_YAW_ANGLE        1.0f
#define SAVE_ORE_PITCH1_ANGLE     1.0f
#define SAVE_ORE_PITCH2_ANGLE     1.0f
#define SAVE_ORE_ROLL_ANGLE       1.0f
#define SAVE_ORE_END_PITCH_ANGLE  1.0f
#define SAVE_ORE_END_ROLL_ANGLE   1.0f
#define SAVE_ORE_GRIP_LENGTH      1.0f
// 待改

namespace my_engineer {


/**
 * @brief 定义系统核心类
 * 
 */
class CSystemCore final {
    // 友元函数
    friend void StartUpdateTask(void *argument);
    friend void StartHeartbeatTask(void *argument);

public:
    // 定义自动操作的任务类型并实例化表示当前任务类型
    enum class EAutoCtrlProcess {
        NONE,
        RETURN_ORIGIN,      ///< 复位
        CLIMBING,           ///< 上台阶
        ENERGY_UNIT,        ///< 抓取能量单元
        EXCHANGE_ORE,       ///< 兑矿
        SAVE_ORE,           ///< 存矿

        // 下面这些是待删的，由于和别的模块比如视觉耦合所以暂时不删
        RETURN_DRIVE,
        DOGHOLE,
        GROUND_ORE,
        SILVER_ORE,
        GOLD_ORE,
        EXCHANGE,
        PUSH_ORE,
        POP_ORE,

    } currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;

    // 面向系统层的控制模式枚举
    enum class ECtrlMode {
        NONE,
        RC_CTRL,            ///< 遥控器控制
        KEY_CTRL,           ///< 键鼠控制
        CONTROLLER_CTRL,    ///< 自定义控制器控制
    } ctrlmode_ = ECtrlMode::NONE;

    // 面向系统层的运动模式枚举
    enum class EMoveMode {
        NONE,
        NORMAL,             ///< 普通
        CLIMBING,           ///< 上台阶
        // ...to be updated...
    } movemode_ = EMoveMode::NONE;

    EVarStatus use_Controller_ = false; ///< 是否使用控制器

    EVarStatus gimbal_auto_ctrl = false;   ///< 云台是否自动控制

    // 初始化系统核心
    EAppStatus InitSystemCore();

private:
    // 定义系统核心的状态
    EAppStatus coreStatus = APP_RESET;

    // 定义系统核心响应频率
    const float_t freq = 1000.f;

    // 模块指针
    CModChassis *pchassis_ = nullptr;
    CModArm *parm_ = nullptr;

    // 自动任务句柄
    TaskHandle_t autoCtrlTaskHandle_ = nullptr;

    // 定义系统核心的更新处理
    void UpdateHandler_();

    // 定义系统核心的心跳处理
    void HeartbeatHandler_();

    // 系统操作方式
    void ControlFromRemote_();
    void ControlFromKeyboard_();
    void ControlFromController_();
    void ControlFromEsp32_();

    // 更新板间通信包
    void BoardLink_Info_Update_();

    // 自动操作(启动与停止)
    EAppStatus StartAutoCtrlTask_(EAutoCtrlProcess process);
    EAppStatus StopAutoCtrlTask_();


    void StartRobot(bool if_remote_control, bool I_dont_have_a_remote = false);

    // 软件复位
    void RESET_SYSTEM();

    // 声明自动操作的任务函数
    static void StartClimbingTask(void *arg);
    static void StartSaveOreTask(void *arg);
    
};

void JointAngleToEulerAngle(const float_t *jointAngle, float_t *eulerAngle);

extern CSystemCore SystemCore;

} // namespace my_engineer

#endif // CORE_HPP
