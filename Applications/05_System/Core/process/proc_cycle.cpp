/******************************************************************************\
 * @brief
 *
 * @file         proc_cycle.cpp
 * @version      V1.0
 *
 ******************************************************************************/

#include "Core.hpp"
#include "utils.h"
#include <cmath>

namespace my_engineer {

void CSystemCore::StartCycleTask(void *arg)
{
    if (arg == nullptr)
    {
        proc_return();
    }

    auto &core = *reinterpret_cast<CSystemCore *>(arg);
    auto &keyboard = SysRemote.remoteInfo.keyboard;

    core.pchassis_->chassisCmd.isAutoCtrl = true;
    if (core.pgimbal_) core.pgimbal_->gimbalCmd.isAutoCtrl = true;
    core.parm_->armCmd.isAutoCtrl = true;

    core.pchassis_->chassisCmd.L_length = CLIMBING_HIP_ANGLE;

    if (core.pgimbal_) core.pgimbal_->gimbalCmd.set_visualyaw = CYCLE_GIMBAL_YAW_ANGLE;

    constexpr float_t kMinRandomAngle = 25.0f;
    constexpr float_t kMaxRandomAngle = 85.0f;
    constexpr float_t kArrivalThreshold = 5.0f;
    constexpr float_t kMinRotateSpeed = 60.0f;
    constexpr float_t kMaxRotateSpeed = 100.0f;
    constexpr float_t kRotateSpeedKp = 2.0f;
    constexpr uint16_t kStableLoopCount = 80U;

    const float_t baseYawAngle = core.pchassis_->filter->Imu_Ave_Info.imu_ave_yaw;

    static uint32_t randomState = 0xA341316CU;
    const int32_t yawSeed = static_cast<int32_t>((baseYawAngle + 180.0f) * 1000.0f);
    randomState ^= static_cast<uint32_t>(yawSeed) + 0x9E3779B9U;
    if (randomState == 0U)
    {
        randomState = 0xA341316CU;
    }
    math::NextRandom(randomState);

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

    bool movingToClockwiseSide = false;
    float_t targetYawAngle = makeRandomTarget(movingToClockwiseSide);
    uint16_t stableLoopCount = 0U;

    proc_waitMs(300);

    core.movemode_ = EMoveMode::CYCLE;
    core.pchassis_->MovMode = CModChassis::EmovMode::CYCLE;

    while (keyboard.key_Shift || core.isCycleActive_)
    {
        const float_t currentYaw =
            core.pchassis_->filter->Imu_Ave_Info.imu_ave_yaw;

        const float_t yawError = math::loopLimit( targetYawAngle - currentYaw, -180.0f, 180.0f);
        const float_t absYawError = std::fabs(yawError);

        if (absYawError > kArrivalThreshold)
        {
            stableLoopCount = 0U;
            const float_t rotateSpeed = std::fmin(kMaxRotateSpeed, std::fmax(kMinRotateSpeed, absYawError * kRotateSpeedKp));
            core.pchassis_->chassisCmd.speed_W =
                math::sign(yawError) * -rotateSpeed;
        }
        else
        {
            core.pchassis_->chassisCmd.speed_W = 0.0f;
            ++stableLoopCount;
            if (stableLoopCount >= kStableLoopCount)
            {
                stableLoopCount = 0;
                movingToClockwiseSide = !movingToClockwiseSide;
                targetYawAngle = makeRandomTarget( movingToClockwiseSide);
            }
        }

        if (keyboard.key_B)
        {
            core.pchassis_->chassisCmd.L_length += static_cast<float_t>(keyboard.mouse_L - keyboard.mouse_R) * 0.015f;
        }

        proc_waitMs(1);
    }

proc_exit:

    core.pchassis_->chassisCmd.speed_W = 0.0f;
    core.pchassis_->chassisCmd.isAutoCtrl = false;
    if (core.pgimbal_) core.pgimbal_->gimbalCmd.isAutoCtrl = false;
    core.parm_->armCmd.isAutoCtrl = false;
    core.isCycleActive_ = false;
    core.autoCtrlTaskHandle_ = nullptr;
    core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
    proc_return();
}

} // namespace my_engineer
