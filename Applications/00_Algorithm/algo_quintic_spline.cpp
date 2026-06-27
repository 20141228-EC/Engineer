/********************************************************************************
 * @brief        五次 Hermite 插值器实现
 *
 *   核心公式
 *   s(τ) = p0·h00 + v0·T·h10 + a0·T²·h20 + p1·h01 + v1·T·h11 + a1·T²·h21
 *
 * @file         algo_quintic_spline.cpp
 * @author       ciallo
 * @version      V1.1
 * @date         2026-6-14
 * @copyright    Copyright (c) 2026
 ********************************************************************************/

#include "algo_quintic_spline.hpp"
#include <cmath>

namespace my_engineer {

/*-------------------------------------初始化插值器----------------------------------------*/
void CAlgoQuinticSpline::Build(const float_t traj[][9], int count,
                               int segFrom, int segTo,
                               float_t rollOff) {
    segCount_    = 0;
    totalTimeMs_ = 0.0f;// 初始化清零

    if (segTo < 0) segTo = count - 1;// 对没有处理的值进行规范
    const int n = segTo - segFrom + 1;
    if (n < 2 || n > MAX_FRAMES || segFrom < 0 || segTo >= count) return;// 失败调用退出

    segCount_ = n - 1;// 段数 = 帧数 - 1，中间的连线

    // 使用成员缓冲区
    auto &pos   = buf_.pos;
    auto &timeS = buf_.timeS;
    auto &vel   = buf_.vel;
    auto &acc   = buf_.acc;

    // 提取位置 + rollOff
    for (int i = 0; i < n; i++) {
        const int src = segFrom + i;
        for (int j = 0; j < AXES; j++) {
            pos[i][j] = traj[src][j + 1];
        }
        pos[i][AXES - 1] += rollOff;
    }

    // 自动时间参数化
    timeS[0] = 0.0f;
    for (int i = 0; i < n - 1; i++) {
        float_t Tseg = 0.0f;
        for (int j = 0; j < AXES; j++) {
            const float_t absDp = std::fabs(pos[i + 1][j] - pos[i][j]);
            if (absDp < 1e-4f) continue;

            const float_t v = velLimit[j];
            const float_t a = accLimit[j];
            if (v < 1e-6f || a < 1e-6f) continue;

            const float_t Tvel = absDp * 1.875f / v;
            const float_t Tacc = std::sqrt(absDp * 5.7735027f / a);
            const float_t Tij  = (Tvel > Tacc) ? Tvel : Tacc;
            if (Tij > Tseg) Tseg = Tij;
        }
        if (Tseg < 0.1f) Tseg = 0.1f;
        timeS[i + 1] = timeS[i] + Tseg * 1.3f;
    }
    totalTimeMs_ = timeS[n - 1] * 1000.0f;

    /*-------------------------------------速度估算的核心代码----------------------------------------*/
    for (int j = 0; j < AXES; j++) {

        // 第 0 帧: 速度与加速度归零
        vel[0][j] = 0.0f;
        acc[0][j] = 0.0f;

        // 中间帧
        for (int i = 1; i < n - 1; i++) { //跳过首帧
            float_t dtPrev = timeS[i] - timeS[i - 1];// 前一段时间
            float_t dtNext = timeS[i + 1] - timeS[i];// 后一段时间
            float_t dtSum  = dtPrev + dtNext;// 总的时间的跨度

            if (dtSum > 1e-6f) {
                float_t slopePrev = (dtPrev > 1e-6f) ? (pos[i][j] - pos[i - 1][j]) / dtPrev : 0.0f;
                float_t slopeNext = (dtNext > 1e-6f) ? (pos[i + 1][j] - pos[i][j]) / dtNext : 0.0f;

                // 前后斜率反号则运动方向反转, 速度归零
                if (slopePrev * slopeNext < 0.0f) {
                    vel[i][j] = 0.0f;
                } else {
                    vel[i][j] = (pos[i + 1][j] - pos[i - 1][j]) / dtSum;
                }

                // 速度限幅
                if (vel[i][j] >  velLimit[j]) vel[i][j] =  velLimit[j];
                if (vel[i][j] < -velLimit[j]) vel[i][j] = -velLimit[j];

                // 加速度估算
                float_t accPrev = (dtPrev > 1e-6f) ? (vel[i][j] - vel[i - 1][j]) / dtPrev : 0.0f;
                float_t velNext = slopeNext;  // 用前向差分作为 next 速度参考
                float_t accNext = (dtNext > 1e-6f) ? (velNext - vel[i][j]) / dtNext : 0.0f;
                acc[i][j] = 0.5f * (accPrev + accNext);
            } else {
                vel[i][j] = 0.0f;
                acc[i][j] = 0.0f;
            }
        }

        // 最后一帧: 速度归零
        vel[n - 1][j] = 0.0f;
        acc[n - 1][j] = 0.0f;
    }

    // 存储每段的 Hermite 边界条件
    for (int s = 0; s < segCount_; s++) {
        SSegCoeff &c   = seg_[s];   // 引用别名
        c.tStartMs     = timeS[s] * 1000.0f; // 段起始时间(ms)
        c.T            = timeS[s + 1] - timeS[s]; // 段时长(s)

        for (int j = 0; j < AXES; j++) {
            c.p0[j] = pos[s][j];
            c.p1[j] = pos[s + 1][j];
            c.v0[j] = vel[s][j];
            c.v1[j] = vel[s + 1][j];
            c.a0[j] = acc[s][j];
            c.a1[j] = acc[s + 1][j];
        }
    }
}

/*----------------------------------查表计算过程---------------------------------*/
void CAlgoQuinticSpline::Evaluate(float_t t_ms, float_t output[AXES]) const {

    // 边界值
    if (t_ms <= seg_[0].tStartMs) {
        for (int j = 0; j < AXES; j++) output[j] = seg_[0].p0[j];
        return;
    }
    if (t_ms >= totalTimeMs_) {
        const SSegCoeff &last = seg_[segCount_ - 1];
        for (int j = 0; j < AXES; j++) output[j] = last.p1[j];
        return;
    }// 将超出限位的帧约束到第一帧和最后一帧

    // 找段
    const int s = FindSegmentRaw(t_ms);

    const SSegCoeff &c = seg_[s];
    const float_t dt = c.T;
    if (dt < 1e-6f) {
        for (int j = 0; j < AXES; j++) output[j] = c.p0[j];
        return;
    }

    // 归一化时间
    const float_t tau = (t_ms - c.tStartMs) * 0.001f / dt;

    // 五次 Hermite 基函数
    const float_t t2 = tau * tau;
    const float_t t3 = t2 * tau;
    const float_t t4 = t3 * tau;
    const float_t t5 = t4 * tau;

    const float_t h00 = 1.0f - 10.0f * t3 + 15.0f * t4 - 6.0f * t5;//控制起点的位置
    const float_t h10 = tau   -  6.0f * t3 +  8.0f * t4 - 3.0f * t5;//控制终点的位置
    const float_t h20 = 0.5f * t2 - 1.5f * t3 + 1.5f * t4 - 0.5f * t5;// 起点的加速度
    const float_t h01 =        10.0f * t3 - 15.0f * t4 + 6.0f * t5;//终点位置
    const float_t h11 =       - 4.0f * t3 +  7.0f * t4 - 3.0f * t5;//终点的速度
    const float_t h21 = 0.5f * t3 - t4 + 0.5f * t5;//终点的加速度

    // Hermite 插值: s = p0·h00 + v0·T·h10 + a0·T^2·h20 + p1·h01 + v1·T·h11 + a1·T^2·h21

    const float_t T  = dt;          // 秒
    const float_t T2 = T * T;

    for (int j = 0; j < AXES; j++) {
        output[j] = c.p0[j] * h00
                  + c.v0[j] * T  * h10
                  + c.a0[j] * T2 * h20
                  + c.p1[j] * h01
                  + c.v1[j] * T  * h11
                  + c.a1[j] * T2 * h21;
    }
}

/*----------------------------内部找段工具函数--------------------------------*/
int CAlgoQuinticSpline::FindSegmentRaw(float_t t_ms) const {
    int s = 0;
    for (int i = 0; i < segCount_; i++) {
        if (t_ms >= seg_[i].tStartMs) s = i;
        else break;
    }
    return s;
}

/*---------------------------------段号计算函数----------------------------------*/
int CAlgoQuinticSpline::GetSegmentIndex(float_t t_ms) const {
    if (t_ms <= 0.0f) return 0;
    if (t_ms >= totalTimeMs_) return segCount_ - 1;
    return FindSegmentRaw(t_ms);
}

} // namespace my_engineer
