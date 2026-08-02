/********************************************************************************
 * @brief        五次点到点轨迹规划器实现
 * @file         algo_quintic.cpp
 * @author       ciallo
 * @version      V1.0
 * @date         2026-6-13
 ********************************************************************************/

#include "algo_quintic.hpp"
#include <cmath>

namespace my_engineer {

// 0.5是因为tau在一次导的时候取极大值
#define Q_VEL_FACTOR    1.875f          /* s'(0.5) = 1.875 */
#define Q_ACC_FACTOR    5.7735027f      /* |s''(0.2113)| ≈ 10√3/3 */

void CAlgoQuintic::PlanPointToPoint(const float_t current[COUNT],
                                    const float_t target[COUNT],
                                    float_t minTime) {
    float_t Tj = 0.0f;

    for (int i = 0; i < COUNT; i++) {
        p0_[i] = current[i];
        const float_t dp = target[i] - current[i];
        dp_[i] = dp;

        const float_t absDp = std::fabs(dp);
        if (absDp < 1e-4f) continue;

        const float_t v = config_.jointParams[i].velMax * speedScale;
        const float_t a = config_.jointParams[i].accMax * speedScale;
        if (v < 1e-6f || a < 1e-6f) continue;

        const float_t Tvel = absDp * Q_VEL_FACTOR / v;
        const float_t Tacc = std::sqrt(absDp * Q_ACC_FACTOR / a);
        const float_t Tij  = (Tvel > Tacc) ? Tvel : Tacc;
        if (Tij > Tj) {
            Tj = Tij;
        }
    }

    T_ = (minTime > Tj) ? minTime : Tj;
}

void CAlgoQuintic::Evaluate(float_t t, float_t output[COUNT]) const {
    if (T_ < 1e-6f) {
        for (int i = 0; i < COUNT; i++) output[i] = p0_[i] + dp_[i];
        return;
    }
    if (t <= 0.0f) {
        for (int i = 0; i < COUNT; i++) output[i] = p0_[i];
        return;
    }
    if (t >= T_) {
        for (int i = 0; i < COUNT; i++) output[i] = p0_[i] + dp_[i];
        return;
    }

    // s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
    const float_t tau = t / T_;
    const float_t t2 = tau * tau;
    const float_t t3 = t2 * tau;
    const float_t t4 = t3 * tau;
    const float_t t5 = t4 * tau;
    const float_t s  = 10.0f * t3 - 15.0f * t4 + 6.0f * t5;

    for (int i = 0; i < COUNT; i++) {
        output[i] = p0_[i] + dp_[i] * s;
    }
}

} // namespace my_engineer
