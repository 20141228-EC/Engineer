/******************************************************************************
 * @brief
 *
 * @file         proc_cycle.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2026-7-18
 *
 * @copyright    Copyright (c) 2026
 *
 ******************************************************************************/

#include "Core.hpp"
#include "utils.h"
#include <cmath>

namespace my_engineer {

/******************************************************************************
 * @brief    底盘随机角度往复旋转任务
 *
 * 运动过程：
 * 进入任务时记录当前角度为基准角
 * -> 在基准角逆时针侧随机选择一个目标角度
 * -> 到达目标并稳定后
 * -> 在基准角顺时针侧随机选择一个目标角度
 * -> 左右交替，每次转动角度随机
 *
 * 设计说明：
 * 1. 方向不是完全随机，而是强制左右交替，保证呈现往复效果；
 * 2. 每次转动角度在 25°～85° 之间随机；
 * 3. 最大单侧角度小于 90°，左右目标之间不会达到 180°；
 * 4. 根据剩余角度动态调节速度，接近目标时自动减速；
 * 5. 到达目标后稳定一段时间才切换目标，减少抖动。
 ******************************************************************************/
void CSystemCore::StartCycleTask(void *arg)
{
    if (arg == nullptr)
    {
        proc_return();
    }

    // 获取 SystemCore 句柄
    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;

    core.pchassis_->chassisCmd.isAutoCtrl = true;
    core.pgimbal_->gimbalCmd.isAutoCtrl = true;
    core.parm_->armCmd.isAutoCtrl = true;

    /* Set Chassis */
    core.pchassis_->chassisCmd.L_length = CLIMBING_HIP_ANGLE;

    /* Set Arm */
    // core.parm_->armCmd.gripClose = true;
    // core.parm_->armCmd.set_angle_Yaw = CYCLE_YAW_ANGLE;
    // core.parm_->armCmd.set_angle_Pitch1 = CYCLE_PITCH1_ANGLE;
    // core.parm_->armCmd.set_angle_Pitch2 = CYCLE_PITCH2_ANGLE;
    // core.parm_->armCmd.set_angle_Roll = CYCLE_ROLL_ANGLE;
    // core.parm_->armCmd.set_angle_end_pitch = CYCLE_END_PITCH_ANGLE;
    // core.parm_->armCmd.set_angle_end_roll = CYCLE_END_ROLL_ANGLE;
    // core.parm_->armCmd.set_length_grip = CYCLE_GRIP_LENGTH;

    /* Set Gimbal */
    core.pgimbal_->gimbalCmd.set_visualyaw = CYCLE_GIMBAL_YAW_ANGLE;
    core.pgimbal_->gimbalCmd.set_pitch = CYCLE_GIMBAL_PITCH_ANGLE;

    /**************************************************************************
     * 可调参数
     **************************************************************************/

    // 每次相对于基准角至少转动多少度
    constexpr float_t kMinRandomAngle = 25.0f;

    // 每次相对于基准角最多转动多少度
    // 建议保持小于 90°，避免两个随机端点相差达到或超过 180°
    constexpr float_t kMaxRandomAngle = 85.0f;

    // 当前角度与目标角度的误差小于该值时，视为到达目标
    constexpr float_t kArrivalThreshold = 5.0f;

    // 接近目标时的最低旋转速度
    constexpr float_t kMinRotateSpeed = 60.0f;

    // 最大旋转速度
    constexpr float_t kMaxRotateSpeed = 100.0f;

    // 角度误差转换为旋转速度的比例系数
    constexpr float_t kRotateSpeedKp = 2.0f;

    // 到达目标后连续稳定的循环次数
    // 当前循环周期约为 1 ms，因此 80 次约等于 80 ms
    constexpr uint16_t kStableLoopCount = 80U;

    /**************************************************************************
     * 初始化基准角和随机数
     **************************************************************************/

    // 进入任务时的当前 yaw 角度作为固定基准角
    const float_t baseYawAngle = core.pchassis_->filter->Imu_Ekf_Info.yaw;

    /*
     * 随机数状态。
     *
     * 使用 static 可以避免每次进入任务时都从完全相同的序列开始。
     * 如果工程中存在系统 Tick、硬件定时器或硬件随机数，
     * 建议将其混合到 randomState 中，随机效果会更好。
     */
    static uint32_t randomState = 0xA341316CU;

    // 使用当前 yaw 角度对随机状态进行扰动
    const int32_t yawSeed = static_cast<int32_t>((baseYawAngle + 180.0f) * 1000.0f);

    randomState ^= static_cast<uint32_t>(yawSeed) + 0x9E3779B9U;

    // xorshift32 的状态不能为 0
    if (randomState == 0U)
    {
        randomState = 0xA341316CU;
    }

    // 先运行一次，进一步打散随机状态
    math::NextRandom(randomState);

    /**
     * @brief 生成某一侧的随机目标角度
     *
     * @param clockwiseSide
     * true：基准角顺时针侧
     * false：基准角逆时针侧
     */
    auto makeRandomTarget =
        [&](bool clockwiseSide) -> float_t
    {
        const float_t randomAngle =
            math::RandomRange(
                randomState,
                kMinRandomAngle,
                kMaxRandomAngle);

        const float_t signedOffset =
            clockwiseSide ? randomAngle : -randomAngle;

        return math::loopLimit(
            baseYawAngle + signedOffset,
            -180.0f,
            180.0f);
    };

    /**************************************************************************
     * 初始化第一个旋转目标
     **************************************************************************/

    // false：目标位于逆时针侧
    // true：目标位于顺时针侧
    bool movingToClockwiseSide = false;

    // 第一个目标先放在基准角逆时针侧
    float_t targetYawAngle = makeRandomTarget(movingToClockwiseSide);

    // 到达目标后的稳定计数
    uint16_t stableLoopCount = 0U;

    proc_waitMs(300);

    // 更新系统层标志位
    core.movemode_ = EMoveMode::CYCLE;

    // 更新模块层标志位
    core.pchassis_->MovMode =
        CModChassis::EmovMode::CYCLE;

    /**************************************************************************
     * 主控制循环
     **************************************************************************/

    // 按住 Shift 或 按键激活时持续执行
    while (keyboard.key_Shift || core.isCycleActive_)
    {
        const float_t currentYaw =
            core.pchassis_->filter->Imu_Ekf_Info.yaw;

        // 限制误差角度
        const float_t yawError = math::loopLimit( targetYawAngle - currentYaw, -180.0f, 180.0f);

        const float_t absYawError = std::fabs(yawError);

        if (absYawError > kArrivalThreshold)
        {
            // 尚未到达目标，清除稳定计数
            stableLoopCount = 0U;

            /*
             * 根据角度误差动态计算旋转速度：
             *
             * 角度误差较大时使用较高速度；
             * 接近目标时自动降低速度；
             * 最终限制在最低速度和最高速度之间。
             */
            const float_t rotateSpeed = std::fmin(kMaxRotateSpeed, std::fmax(kMinRotateSpeed, absYawError * kRotateSpeedKp));

            core.pchassis_->chassisCmd.speed_W =
                math::sign(yawError) * -rotateSpeed;
        }
        else
        {
            // 已进入目标角度误差范围，停止旋转
            core.pchassis_->chassisCmd.speed_W = 0.0f;

            /*
             * 连续稳定一段时间后才切换目标 防止在到达阈值附近反复切换
             */
            ++stableLoopCount;

            if (stableLoopCount >= kStableLoopCount)
            {
                stableLoopCount = 0;

                // 强制切换到另一侧
                movingToClockwiseSide = !movingToClockwiseSide;

                // 在另一侧重新随机生成目标角度
                targetYawAngle = makeRandomTarget( movingToClockwiseSide);
            }
        }

        // 允许抬腿
        if (keyboard.key_B)
        {
            core.pchassis_->chassisCmd.L_length += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 0.015f;
        }

        proc_waitMs(1);
    }

proc_exit:

    // 退出任务前停止底盘旋转
    core.pchassis_->chassisCmd.speed_W = 0.0f;
    // 恢复各模块控制状态
    core.pchassis_->chassisCmd.isAutoCtrl = false;
    core.pgimbal_->gimbalCmd.isAutoCtrl = false;
    core.parm_->armCmd.isAutoCtrl = false;
    core.isCycleActive_ = false;
    core.autoCtrlTaskHandle_ = nullptr;
    core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
    proc_return();
}

} // namespace my_engineer