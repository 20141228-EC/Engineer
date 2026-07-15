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
#include "algo_ave_filter.hpp"
#include "algo_imu_ekf.hpp"
#include "algo_kf_filter.hpp"
#include "algo_traj_playback.hpp"

#define I_AM_CONTROLLER 0 // 当前板子是控制器

/*-------------------------------------AUTO_PROCESS_SET----------------------------------------------------------*/

/* ----------------------上台阶------------------- */
#define CLIMBING_YAW_ANGLE        ARM_YAW_INIT_ANGLE
#define CLIMBING_PITCH1_ANGLE     ARM_PITCH1_INIT_ANGLE
#define CLIMBING_PITCH2_ANGLE     ARM_PITCH2_INIT_ANGLE
#define CLIMBING_ROLL_ANGLE       ARM_ROLL_INIT_ANGLE
#define CLIMBING_END_PITCH_ANGLE  80
#define CLIMBING_END_ROLL_ANGLE   ARM_END_ROLL_INIT_ANGLE
#define CLIMBING_GRIP_LENGTH      ARM_GRIP_INIT_LENGTH
#define CLIMBING_SPEED            20.f      ///< 给一个较小的速度
#define SAVING_SPEED              -40.f     ///< 回退
#define SAVING_HIP_ANGLE          6.3f      ///< 自救腿长
#define CLIMBING_HIP_ANGLE        2.5f      ///< 抬一点腿

/* -----------------------下台阶--------------------*/
#define DOWNSTAIR_YAW_ANGLE        ARM_YAW_INIT_ANGLE
#define DOWNSTAIR_PITCH1_ANGLE     ARM_PITCH1_INIT_ANGLE
#define DOWNSTAIR_PITCH2_ANGLE     ARM_PITCH2_INIT_ANGLE
#define DOWNSTAIR_ROLL_ANGLE       -1
#define DOWNSTAIR_END_PITCH_ANGLE  90
#define DOWNSTAIR_END_ROLL_ANGLE   ARM_END_ROLL_INIT_ANGLE
#define DOWNSTAIR_GRIP_LENGTH      ARM_GRIP_INIT_LENGTH
#define DOWNSTAIR_SPEED            -50.f      ///< 全速的80%
#define DOWNSTAIR_HIP_ANGLE        5.3f      ///< 腿抬高
#define DOWNSTAIR_GIMBAL_ANGLE     179.f

/* --------------------抓能量单元------------------- */
#define GRAB_ENERGY_UNIT_YAW_ANGLE        1.0f
#define GRAB_ENERGY_UNIT_PITCH1_ANGLE     1.0f
#define GRAB_ENERGY_UNIT_PITCH2_ANGLE     1.0f
#define GRAB_ENERGY_UNIT_ROLL_ANGLE       1.0f
#define GRAB_ENERGY_UNIT_END_PITCH_ANGLE  1.0f
#define GRAB_ENERGY_UNIT_END_ROLL_ANGLE   1.0f
#define GRAB_ENERGY_UNIT_GRIP_LENGTH      1.0f
// 待改


/* --------------------兑换矿石-------------------- */
#define EXCHANGE_ORE_YAW_ANGLE        1.0f
#define EXCHANGE_ORE_PITCH1_ANGLE     1.0f
#define EXCHANGE_ORE_PITCH2_ANGLE     1.0f
#define EXCHANGE_ORE_ROLL_ANGLE       1.0f
#define EXCHANGE_ORE_END_PITCH_ANGLE  1.0f
#define EXCHANGE_ORE_END_ROLL_ANGLE   1.0f
#define EXCHANGE_ORE_GRIP_LENGTH      1.0f
// 待改

/* ----------------------存矿------------------------*/
#define SAVE_ORE_YAW_ANGLE        1.0f
#define SAVE_ORE_PITCH1_ANGLE     1.0f
#define SAVE_ORE_PITCH2_ANGLE     1.0f
#define SAVE_ORE_ROLL_ANGLE       1.0f
#define SAVE_ORE_END_PITCH_ANGLE  1.0f
#define SAVE_ORE_END_ROLL_ANGLE   1.0f
#define SAVE_ORE_GRIP_LENGTH      1.0f
// 待改


/* -----------------------全部复位----------------------- */
#define RETURN_ORIGIN_YAW_ANGLE        ARM_YAW_INIT_ANGLE
#define RETURN_ORIGIN_PITCH1_ANGLE     ARM_PITCH1_INIT_ANGLE
#define RETURN_ORIGIN_PITCH2_ANGLE     ARM_PITCH2_INIT_ANGLE
#define RETURN_ORIGIN_ROLL_ANGLE       ARM_ROLL_INIT_ANGLE
#define RETURN_ORIGIN_END_PITCH_ANGLE  ARM_END_PITCH_INIT_ANGLE
#define RETURN_ORIGIN_END_ROLL_ANGLE   ARM_END_ROLL_INIT_ANGLE
#define RETURN_ORIGIN_GRIP_LENGTH      ARM_GRIP_INIT_LENGTH


/* ------------------------捡地矿------------------------- */
#define GROUND_ORE_YAW_ANGLE        1.0f
#define GROUND_ORE_PITCH1_ANGLE     1.0f
#define GROUND_ORE_PITCH2_ANGLE     1.0f
#define GROUND_ORE_ROLL_ANGLE       1.0f
#define GROUND_ORE_END_PITCH_ANGLE  1.0f
#define GROUND_ORE_END_ROLL_ANGLE   1.0f
#define GROUND_ORE_GRIP_LENGTH      1.0f
#define GROUND_ORE_HIP_LENGTH       1.0f
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
    friend class CStoreOreTaskRunner;  ///< 允许 Runner 访问 oreTaskStep_ / oreGetDone_ 等私有成员

public:
    // 定义自动操作的任务类型并实例化表示当前任务类型
    enum class EAutoCtrlProcess {
        NONE,
        RETURN_ORIGIN,      ///< 所有模块复位
        CLIMBING,           ///< 上台阶
        DOWN_STAIR,         ///< 下台阶
        ENERGY_UNIT,        ///< 抓取能量单元
        EXCHANGE_ORE,       ///< 兑矿
        STORE_ORE,           ///< 存矿
        GROUND_ORE,         ///< 地矿
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
        EXCHANGE_L_ORE,           ///< 左边取矿
        EXCHANGE_R_ORE,          ///< 右边取矿
        AUTO,                   ///< 自动六矿
        // ...to be updated...
    } armmode_ = EArmMode::NONE;

    enum class EGripKeyboardCmd : uint8_t {
        HOLD,
        CLOSE,
        OPEN,
    };

    // 存矿时末端 roll 的朝向（上/下），手动存矿前由 R 键切换
    enum class EStoreEndRollPose : uint8_t {
        DOWN,
        UP,
    } storeEndRollPose_ = EStoreEndRollPose::DOWN;

    EVarStatus use_Controller_ = false; ///< 是否使用控制器

    EVarStatus gimbal_auto_ctrl = false;   ///< 云台是否自动控制

    EVarStatus arm_init_fail = false;          ///< 臂初始化失败

    // 初始化系统核心
    EAppStatus InitSystemCore();

private:
    // 定义系统核心的状态
    EAppStatus coreStatus = APP_RESET;

    // 定义系统核心响应频率
    const float_t freq = 1000.f;
    // 保留上一次的存取矿石的记忆
    uint8_t oreTaskStep_ = 0; 
    bool oreGetDone_ = false;// 确定是否停下

    // 模块指针
    CModChassis *pchassis_ = nullptr;
    CModArm *parm_ = nullptr;
    CModGimbal *pgimbal_ = nullptr;

    // 自动任务句柄
    TaskHandle_t autoCtrlTaskHandle_ = nullptr;

    // 自定义控制器模式下的夹爪键盘指令状态
    EGripKeyboardCmd gripKeyboardCmd_ = EGripKeyboardCmd::HOLD;

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
    static void StartDownStairTask(void *arg);
    static void StartSaveOreTask(void *arg);
    static void StartGroundOreTask(void *arg);
    static void StartExchangeOreTask(void *arg);
    static void StartReturnOriginTask(void *arg);
    static void StartEnergyUnitTask(void *arg);

    static void StartStoreTask(void *arg);
    static void StartExchangeGetTask(void *arg);
    
};

void JointAngleToEulerAngle(const float_t *jointAngle, float_t *eulerAngle);    // 未实现

extern CSystemCore SystemCore;

} // namespace my_engineer

#endif // CORE_HPP
