/********************************************************************************
 * @brief        五次点到点轨迹规划器（零边界条件）
 *
 *   s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
 *   峰值速度系数: 1.875    (τ=0.5)
 *   峰值加速度系数: 5.7735  (τ≈0.2113 / 0.7887)
 *
 * @file         algo_quintic.hpp
 * @author       ciallo
 * @version      V1.0
 * @date         2026-6-13
 ********************************************************************************/

#ifndef ALGO_QUINTIC_HPP
#define ALGO_QUINTIC_HPP

#include "Configuration.hpp"
#include "algo_traj_playback.hpp"

namespace my_engineer {

class CAlgoQuintic {
public:
    static const int COUNT = CAlgoTrajPlayback::JointId::COUNT;

    CAlgoTrajPlayback::SConfig config_;

    float_t speedScale = 1.0f;  // >1 加速，<1 减速

    /// 规划从 current 到 target 的同步五次轨迹
    void PlanPointToPoint(const float_t current[COUNT],
                          const float_t target[COUNT],
                          float_t minTime = 0.0f);

    /// 查询 t 秒时的关节角度
    void Evaluate(float_t t, float_t output[COUNT]) const;

    bool    IsFinished(float_t t) const { return t >= T_; }
    float_t GetMaxTime() const { return T_; }

private:
    float_t p0_[COUNT] = {0};   // 起点位置
    float_t dp_[COUNT] = {0};   // 位移量
    float_t T_ = 0.0f;          // 同步时长（秒）
};

} // namespace my_engineer

#endif // ALGO_QUINTIC_HPP
