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
#include "algo_quintic.hpp"

#define I_AM_CONTROLLER 0 // 当前板子是控制器

/* --------------------末端 roll 一键翻转------------------- */
#define END_ROLL_FLIP_SPEED       700.f         ///< 翻转限速

/*-------------------------------------AUTO_PROCESS_SET----------------------------------------------------------*/

/* ----------------------上台阶------------------- */
#define CLIMBING_YAW_ANGLE        ARM_YAW_INIT_ANGLE
#define CLIMBING_PITCH1_ANGLE     45.f
#define CLIMBING_PITCH2_ANGLE     20.f
#define CLIMBING_ROLL_ANGLE       ARM_ROLL_INIT_ANGLE
#define CLIMBING_END_PITCH_ANGLE  80
#define CLIMBING_END_ROLL_ANGLE   ARM_END_ROLL_INIT_ANGLE
#define CLIMBING_GRIP_LENGTH      ARM_GRIP_INIT_LENGTH
#define CLIMBING_SPEED            20.f      ///< 给一个较小的速度
#define SAVING_SPEED              -60.f     ///< 回退
#define SAVING_HIP_ANGLE          6.3f      ///< 自救腿长
#define CLIMBING_HIP_ANGLE        0.f      ///< 抬一点腿

/* -----------------------下台阶--------------------*/
#define DOWNSTAIR_YAW_ANGLE        ARM_YAW_INIT_ANGLE
#define DOWNSTAIR_PITCH1_ANGLE     ARM_PITCH1_INIT_ANGLE
#define DOWNSTAIR_PITCH2_ANGLE     ARM_PITCH2_INIT_ANGLE
#define DOWNSTAIR_ROLL_ANGLE       -1
#define DOWNSTAIR_END_PITCH_ANGLE  90
#define DOWNSTAIR_END_ROLL_ANGLE   ARM_END_ROLL_INIT_ANGLE
#define DOWNSTAIR_GRIP_LENGTH      ARM_GRIP_INIT_LENGTH
#define DOWNSTAIR_SPEED            -80.f      ///< 全速的80%
#define DOWNSTAIR_HIP_ANGLE        7.8f      ///< 腿抬高
#define DOWNSTAIR_GIMBAL_ANGLE     -185.f

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
#define EXCHANGE_ORE_GIMBLE_PITCH_ANGLE      -55.0f
#define EXCHANGE_ORE_GIMBLE_YAW_ANGLE      0.f
#define EXCHANGE_ORE_GIMBLE_INIT_ANGLE      55.f
#define EXCHANGE_ORE_GIMBLE_ANGLE      55.f
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

/* ------------------------大陀螺------------------------- */
#define CYCLE_YAW_ANGLE        -4.0f
#define CYCLE_PITCH1_ANGLE     91.0f
#define CYCLE_PITCH2_ANGLE     31.0f
#define CYCLE_ROLL_ANGLE       9.0f
#define CYCLE_END_PITCH_ANGLE  -14.0f
#define CYCLE_END_ROLL_ANGLE   -1.5f
#define CYCLE_GRIP_LENGTH      1.0f
#define CYCLE_HIP_LENGTH       1.0f
#define CYCLE_GIMBAL_YAW_ANGLE -2.0f
#define CYCLE_GIMBAL_PITCH_ANGLE    -16.0f



namespace my_engineer {

struct SArmPresetPose{
     float yaw = 0.f, pitch1 = 0.f, pitch2 = 0.f, pitch3 = 0.f, roll = 0.f, end_pitch = 0.f, end_roll = 0.f;
};

// preset 预设位姿
const SArmPresetPose PresetPose_Level[3] = {
    {/*LEVEL_1:*/ -2.7f, 17.1f, 20.2f, 0.f, 102.f, 86.f, 0.f},
    {/*LEVEL_2:*/ -2.7f, 14.9f, 20.4f, 0.f, -1.f,  65.f, 0.f},
    {/*LEVEL_3:*/ -2.7f, 14.9f, 20.4f, 0.f, -1.f,  65.f, 0.f},
};

class CStoreOreTaskRunner;

/**
 * @brief 定义系统核心类
 * 
 */
class CSystemCore final {
    // 友元函数
    friend void StartUpdateTask(void *argument);
    friend void StartHeartbeatTask(void *argument);
    friend class CStoreOreTaskRunner;

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
        CYCLE,              ///< 大陀螺
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
        CYCLE,              ///< 大陀螺
        // ...to be updated...
    } movemode_ = EMoveMode::NONE;

    enum class EArmMode {
        NONE,
        NORMAL,             ///< 普通
        STORE_L_ORE,           ///< 左边存矿
        STORE_R_ORE,          ///< 右边存矿
        EXCHANGE_L_ORE,           ///< 左边取矿
        EXCHANGE_R_ORE,          ///< 右边取矿
        AUTO
        // ...to be updated...
    } armmode_ = EArmMode::NONE;

    // 取矿左右预选
    enum class EExchangeSide { NONE, LEFT, RIGHT, AUTO } exchange_side_ = EExchangeSide::NONE;

    enum class EGripKeyboardCmd : uint8_t {
        HOLD,
        CLOSE,
        OPEN,
    };

    enum class EStoreEndRollPose : uint8_t {
        DOWN,
        UP,
    } storeEndRollPose_ = EStoreEndRollPose::DOWN;

    EVarStatus use_Controller_ = false; ///< 是否使用控制器

    bool isCycleActive_ = false;

    EVarStatus gimbal_auto_ctrl = false;   ///< 云台是否自动控制

    EVarStatus arm_init_fail = false;          ///< 臂初始化失败

    // 初始化系统核心
    EAppStatus InitSystemCore();

private:
    CAlgoQuintic quinticPlayer_;
    CAlgoRamp endRollRamp_;            ///< 末端 roll 一键翻转的限速斜坡
    // 定义系统核心的状态
    EAppStatus coreStatus = APP_RESET;

    uint8_t oreTaskStep_ = 0;
    bool oreGetDone_ = false;// 确定是否停下

    // 定义系统核心响应频率
    const float_t freq = 1000.f;

    // 选择难度等级时，初始化臂的动作
    uint32_t presetStartTime_ = 0;     // 记录当前的时间戳
    bool presetActive_ = false;        // preset 进行中标志
    uint8_t presetLevel_ = 0;          // 当前 preset 等级 (1/2/3)
    bool presetHolding_ = false;       // preset 到位后等待切换中
    uint32_t presetHoldStart_ = 0;     // holding 开始时刻

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
    static void StartCycleTask(void *arg);
    
};

void JointAngleToEulerAngle(const float_t *jointAngle, float_t *eulerAngle);    // 未实现

extern CSystemCore SystemCore;

} // namespace my_engineer

#endif // CORE_HPP
