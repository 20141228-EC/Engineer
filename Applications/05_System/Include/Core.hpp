/**
 * @file Core.hpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 系统核心头文件
 * @version 1.0
 * @date 2024-11-10
 * 
 * @copyright Copyright (c) 2024
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

namespace my_engineer {


// 前向声明模块类以减少头文件依赖
class CModChassis;
class CModGimbal;
class CModArm;


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
        NONE = 0,
        RETURN_ORIGIN = 1,      // 全部复位
        STORE_ORE = 2,          // 存矿
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
        DOWNSTAIR,          ///< 下台阶
        // ...to be updated...
    } movemode_ = EMoveMode::NONE;

    enum class EArmMode {
        NONE,
        NORMAL,             ///< 普通
        STORE_L_ORE,           ///< 左边存矿
        STORE_R_ORE,          ///< 右边存矿
        GET_L_ORE,           ///< 左边取矿
        GET_R_ORE,          ///< 右边取矿
        // ...to be updated...
    } armmode_ = EArmMode::NONE;

    EVarStatus use_Controller_ = false; ///< 是否使用控制器

    // 初始化系统核心
    EAppStatus InitSystemCore();

private:
    // 定义系统核心的状态
    EAppStatus coreStatus = APP_RESET;

    // 定义系统核心响应频率
    const float_t freq = 1000.f;

    // 模块指针
    CModChassis *pchassis_ = nullptr;

    // 系统指针
    CSystemBoardLink *pboardlink_ = nullptr;  ///< 板间通信系统指针

    TaskHandle_t autoCtrlTaskHandle_ = nullptr;

    // 自定义控制器模式下的夹爪键盘翻转状态
    bool gripKeyboardcom_ = false;

    // 底盘控制指令(由操作手决定)
    struct SChassisCmd {
        float_t speed_x = 0.f;  // 横向速度
        float_t speed_y = 0.f;  // 前进速度
        float_t speed_w = 0.f;  // 旋转速度

        EVarStatus is_spin_on = false;  // 开启小陀螺
    }chassisCmd;

    // 底盘控制指令(根据转系处理之后最终发给下板)
    struct Core
    {
        float_t speed_x_ = 0.f;
        float_t speed_y_ = 0.f;
        float_t speed_w_ = 0.f;
    }chassisCmd_;

    // 上层发来的软件复位标志
    EVarStatus ResetFlag = false;
    

    // 定义系统核心的更新处理
    void UpdateHandler_();

    // 定义系统核心的心跳处理
    void HeartbeatHandler_();

    // 系统操作方式
    void ControlFromRemote_();
    void ControlFromKeyboard_();
    void ControlFromController_();
    void ControlFromEsp32_();

    // 自动操作(启动与停止)
    EAppStatus StartAutoCtrlTask_(EAutoCtrlProcess process);
    EAppStatus StopAutoCtrlTask_();

    void StartRobot(bool if_remote_control, bool I_dont_have_a_remote = false);

    // 软件复位
    void RESET_SYSTEM();

    // 获取板通信息
    void BoardLink_Info_Update_();

    // 底盘指令更新
    void Chassis_UpdateHandler_();

    // 限制底盘命令
    void RestrictChassisCmd_();

    // 声明自动操作的任务函数（实现位于各 process_* 源文件）
    static void StartReturnOriginTask(void *arg);
    static void StartReturnDriveTask(void *arg);
    static void StartDogHoleTask(void *arg);
    static void StartGroundOreTask(void *arg);
    static void StartSilverOreTask(void *arg);
    static void StartGoldOreTask(void *arg);
    static void StartExchangeTask(void *arg);
    static void StartExchangeOreTask(void *arg);
    static void StartSaveOreTask(void *arg);
    static void StartPopOreTask(void *arg);
    static void StartPushOreTask(void *arg);
    static void StartVisionExchangeTask(void *arg);
    static void StartTurnoverTask(void *arg);
    static void StartClimbingTask(void *arg);
    static void StartEnergyUnitTask(void *arg);

};

void JointAngleToEulerAngle(const float_t *jointAngle, float_t *eulerAngle);

extern CSystemCore SystemCore;

} // namespace my_engineer

#endif // CORE_HPP