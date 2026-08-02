/********************************************************************************
 * @brief        五次 Hermite 插值器
 *
 * @file         algo_quintic_spline.hpp
 * @author       ciallo
 * @version      V1.0
 * @date         2026-6-10
 * @copyright    Copyright (c) 2026
 ********************************************************************************/

#ifndef ALGO_QUINTIC_SPLINE_HPP
#define ALGO_QUINTIC_SPLINE_HPP

#include "Configuration.hpp"

namespace my_engineer {

class CAlgoQuinticSpline {
public:

    static const int MAX_FRAMES = 20;   ///< 最大关键帧数
    static const int AXES       = 7;    ///< 关节数 
    /// 每个关节的最大速度限幅
    float_t velLimit[AXES] = {
        90.0f,   // yaw
        90.0f,   // p1
        80.0f,   // p2
        90.0f,   // p3
        120.0f,  // roll
        150.0f,  // endP
        120.0f,   // endR
    };

    /// 每个关节的最大加速度限幅
    float_t accLimit[AXES] = {
        180.0f,  // yaw
        180.0f,  // p1
        160.0f,  // p2
        180.0f,  // p3
        300.0f,  // roll
        300.0f,  // endP
        300.0f,  // endR
    };

    /**
     * @brief 轨迹关键帧数组
     * @param traj     关键帧数组
     * @param count    关键帧行总数
     * @param segFrom  起始帧索引 
     * @param segTo    结束帧索引
     * @param rollOff  endRoll 偏移量
     */
    void Build(const float_t traj[][10], int count,
               int segFrom = 0, int segTo = -1,
               float_t rollOff = 0.0f);

    /**
     * @brief 在时刻 t_ms 毫秒处，计算 7 个关节角度
     * @param t_ms   从轨迹起始算起的毫秒数
     * @param output 输出 7 个关节角度
     */
    void Evaluate(float_t t_ms, float_t output[AXES]) const;

    /// 轨迹总时长 (ms)
    float_t GetTotalDurationMs() const { return totalTimeMs_; }

    /// 轨迹是否已结束
    bool IsFinished(float_t t_ms) const { return t_ms >= totalTimeMs_; }

    /**
     * @brief 获取 t_ms 时刻所在的段号
     * @return 段索引
     */
    int GetSegmentIndex(float_t t_ms) const;

    /// 获取段数
    int GetSegmentCount() const { return segCount_; }

    ~CAlgoQuinticSpline() = default;

private:
    /// 每段的 Hermite 边界条件
    struct SSegCoeff {
        float_t tStartMs;           ///< 段起始时间
        float_t T;                  ///< 段时长
        float_t p0[AXES], p1[AXES]; ///< 起始/终止位置
        float_t v0[AXES], v1[AXES]; ///< 起始/终止速度
        float_t a0[AXES], a1[AXES]; ///< 起始/终止加速度
    };

    // 找段索引
    int FindSegmentRaw(float_t t_ms) const;

    // 固定成员数组
    SSegCoeff seg_[MAX_FRAMES - 1];// 每一帧
    int       segCount_    = 0;
    float_t   totalTimeMs_ = 0.0f;
    
    // 数组缓冲
    struct SBuildBuffer {
        float_t pos[MAX_FRAMES][AXES];
        float_t timeS[MAX_FRAMES];
        float_t vel[MAX_FRAMES][AXES];
        float_t acc[MAX_FRAMES][AXES];
    };
    SBuildBuffer buf_;
};

} // namespace my_engineer

#endif // ALGO_QUINTIC_SPLINE_HPP
