/********************************************************************************
 * @brief        单轴梯形速度曲线规划器
 * @file         algo_traj_playback.hpp
 * @author       ciallo (1002046597@qq.com)
 * @version      V1.0
 * @date         2026-4-19
 * @copyright    Copyright (c) 2026
 ********************************************************************************/

#include "algo_traj_playback.hpp"
#include <cmath>

namespace my_engineer{

    /**
     * @brief 规划梯形角速度曲线
     * @param _startpoint  起始位置 (度)
     * @param _endpoint  目标位置
     * @param _vMax 最大速度 (度/s)
     * @param _acc  加速度   (度/s²)
     */
    void CAlgoTrajPlayback::STrapezoidalSpeed::TrapezoidalSpeedPlanner(float_t _startpoint, float_t _endpoint, float_t _vMax, float_t _acc) {
        startpoint = _startpoint;
        endpoint = _endpoint;
        totalDist = endpoint - startpoint;
        absDist = fabsf(totalDist);

        vMax = _vMax;
        aMax = _acc;

        if(absDist <1e-3f){            // 距离过短，直接设置为目标点
            tAcc = tDec = tConst = 0.0f;
            sAcc = sDec = sConst = 0.0f;   //加减速的时间都设置为0
            vPeak = 0.0f;
            tTotal = 0.0f;
            return;
        }

        float_t sAccDec = vMax * vMax / _acc; //加减速的总距离v^2 = 2ax 然后加减速的距离相加

        if(sAccDec <= absDist){     //包含了退化的梯形
            vPeak  = vMax;
            sAcc   = vMax * vMax / (2.0f * _acc);  // v^2/(2a)
            sDec   = sAcc;
            sConst = absDist - sAcc - sDec;
        } else{
            vPeak  = sqrtf(absDist * _acc);    // 这里是一半的距离      
            sAcc   = absDist * 0.5f;
            sDec   = sAcc;
            sConst = 0;
        } 
        
        tAcc = vPeak / aMax;
        tDec   = tAcc;
        tConst = (sConst > 0.001f) ? sConst / vPeak : 0;
        tTotal = tAcc + tConst + tDec;
    };

    /**
     * @brief 给定 (tAcc, tConst, tDec)，反推 vPeak / aMax
     */
    void CAlgoTrajPlayback::STrapezoidalSpeed::SetSynchronized(
        float_t _startpoint, float_t _endpoint,
        float_t _tAcc, float_t _tConst, float_t _tDec) {

        startpoint = _startpoint;
        endpoint   = _endpoint;
        totalDist  = endpoint - startpoint;
        absDist    = fabsf(totalDist);

        tAcc   = _tAcc;
        tConst = _tConst;
        tDec   = _tDec;
        tTotal = tAcc + tConst + tDec;

        // 静止关节或时长为零：所有运动量清零，但保留 tTotal
        // 让 GetPosition 在 t<=0 / t>=tTotal 分支正确返回 start/endpoint
        if (absDist < 1e-3f || tTotal < 1e-6f) {
            vMax = aMax = vPeak = 0.0f;
            sAcc = sDec = sConst = 0.0f;
            return;
        }

        const float_t denom = 0.5f * tAcc + tConst + 0.5f * tDec;
        vPeak  = (denom > 1e-6f) ? (absDist / denom) : 0.0f;
        aMax   = (tAcc  > 1e-6f) ? (vPeak / tAcc)    : 0.0f;
        vMax   = vPeak;   // 兼容

        sAcc   = 0.5f * vPeak * tAcc;
        sConst = vPeak * tConst;
        sDec   = 0.5f * vPeak * tDec;
    }

    /**
     * @brief 位置查询函数
     * @param t 从运动开始经过的时间 (s)
     * @return 当前位置
     */
    float_t CAlgoTrajPlayback::STrapezoidalSpeed::GetPosition(float_t t) const {
        if (t <= 0)     return startpoint;
        if (t >= tTotal) return endpoint;

        float_t s = 0.f;
        float_t dir = (totalDist >= 0) ? 1.0f : -1.0f;

        // 纯物理计算
        if(t <= tAcc){ //加速阶段
            s = 0.5f * aMax * t * t;
        }else if(t <= tAcc + tConst){ //匀速阶段
            s = sAcc + vPeak * (t - tAcc);
        }else if(t <= tAcc + tConst + tDec){ //减速阶段
            float_t td = t - tAcc - tConst;
            s = sAcc + sConst + vPeak * td - 0.5f * aMax * td * td;
        }
        return startpoint + dir * s;
    }

    /**
     * @brief 多轴协调轨迹播放器
     *
     * minTime: 默认 0 = 纯按物理参数规划（正常播放都用这个）
     *          > 0    = 强制最小总时长，把整段拖慢到 minTime（仅用于安全慢速/调试）
     *          这个函数的作用就是通过规划各个关节的运动时间然后找到运动时间最长的那个关节，将时间赋值到每个关节上面重新规划
     */
    void CAlgoTrajPlayback::PlanMultiAxisTraj(const float_t current[JointId::COUNT],
                                              const float_t target[JointId::COUNT],
                                              float_t minTime) {
        // ---------- 第一遍：用各关节物理参数算各自的梯形 tTotal ----------
        STrapezoidalSpeed tmp[JointId::COUNT];
        int     leadIdx = -1;
        float_t leadT   = 0.0f;

        for (int i = 0; i < JointId::COUNT; i++) {
            const float_t vel = TrajConfig.jointParams[i].velMax * speedScale;
            const float_t acc = TrajConfig.jointParams[i].accMax * speedScale;
            tmp[i].TrapezoidalSpeedPlanner(current[i], target[i], vel, acc);

            if (tmp[i].tTotal > leadT) {
                leadT   = tmp[i].tTotal;
                leadIdx = i;
            }// 选择排序记录最大时长
        }

        // ---------- 决定整段总时长：max(物理 leadT, minTime) ----------
        maxTime_ = (minTime > leadT) ? minTime : leadT;

        // 全部静止（包括 minTime <= 0 的极端情况）：直接拷贝原规划，所有关节都是零运动
        if (maxTime_ < 1e-6f || leadIdx < 0) {
            for (int i = 0; i < JointId::COUNT; i++) {
                jointTrajs[i] = tmp[i];
                jointTrajs[i].tTotal = maxTime_;   
            }
            return;
        }

        // ---------- 取最开头的 (tAcc, tConst, tDec) 作为同相位模板 ----------
        float_t leadTAcc, leadTConst, leadTDec;
        if (tmp[leadIdx].tTotal > 1e-6f) {
            // 若 minTime 拉伸了 maxTime_，把领头的三段等比例放大
            const float_t k = (minTime > tmp[leadIdx].tTotal)
                              ? (maxTime_ / tmp[leadIdx].tTotal)
                              : 1.0f;
            leadTAcc   = tmp[leadIdx].tAcc   * k;
            leadTConst = tmp[leadIdx].tConst * k;
            leadTDec   = tmp[leadIdx].tDec   * k;
        } else {
            // 极端情况：所有关节几乎不动但 minTime > 0，构造 1:8:1 的对称梯形作为兜底
            leadTAcc   = maxTime_ * 0.1f;
            leadTDec   = maxTime_ * 0.1f;
            leadTConst = maxTime_ - leadTAcc - leadTDec;
        }

        // ---------- 第二遍：所有关节同相位规划 ----------
        for (int i = 0; i < JointId::COUNT; i++) {
            jointTrajs[i].SetSynchronized(current[i], target[i],
                                          leadTAcc, leadTConst, leadTDec);
        }
    }

    /**
     * @brief 各关节运动距离
     */
    void CAlgoTrajPlayback::MultiAxisTrajDistance(float_t t, float_t output[JointId::COUNT]) const{
        for(int i = 0 ;i < JointId::COUNT; i++){
            output[i] = jointTrajs[i].GetPosition(t);
        }
    }




} // namespace my_engineer
