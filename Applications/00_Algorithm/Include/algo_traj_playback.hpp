/********************************************************************************
 * @brief        单轴梯形速度曲线规划器
 * @file         algo_traj_playback.hpp
 * @author       ciallo (1002046597@qq.com)
 * @version      V1.0
 * @date         2026-4-19
 * @copyright    Copyright (c) 2026
 ********************************************************************************/

#ifndef ALGO_TRAJ_PLAYBACK_HPP
#define ALGO_TRAJ_PLAYBACK_HPP

#include "Configuration.hpp"
#include <cmath>

namespace my_engineer{

class CAlgoTrajPlayback {
public:
    struct STrapezoidalSpeed {
        float_t startpoint = 0.0f;  // 轨迹起始点
        float_t endpoint = 0.0f;   // 轨迹终点位置
        float_t totalDist = 0.0f;   // 轨迹有向距离
        float_t absDist = 0.0f; // 绝对距离

        float_t vMax = 0.0f;    //设置单轴的最大速度
        float_t aMax = 0.0f;    //设置单轴的最大加速度

        float_t tAcc = 0.0f;    //加速时间
        float_t tDec = 0.0f;    //减速时间
        float_t tConst = 0.0f;  //匀速时间
        float_t tTotal = 0.0f;  //总时间

        float_t sAcc = 0.0f;  //加速度段距离
        float_t sDec = 0.0f;  //减速度段距离
        float_t sConst = 0.0f; //匀速段距离

        float_t vPeak = 0.0f; //峰值速度

    /**
     * @brief 规划梯形角速度曲线
     * @param _startpoint  起始位置 (度)
     * @param _endpoint  目标位置
     * @param _vMax 最大速度 (度/s)
     * @param _acc  加速度   (度/s²)
     */

    void TrapezoidalSpeedPlanner(float_t _startpoint, float_t _endpoint, float_t _vMax, float_t _acc) ;

    /**
     * @brief 位置查询函数
     * @param t 从运动开始经过的时间 (s)
     * @return 当前位置
     */
    float_t GetPosition(float_t t) const ;
    };

/**
 * @brief 多轴协调轨迹播放器
 */
public:
    //static const int Axes_Number = 7; // 支持7自由度

    enum JointId : uint8_t {
        J_YAW = 0,
        J_P1,
        J_P2,
        J_P3,
        J_ROLL,
        J_ENDP,
        J_ENDR,
        COUNT,
    };

    struct SJointPrarm{
        float_t velMax; //最大速度
        float_t accMax; //最大加速度
    };

    struct SConfig
    {
        SJointPrarm jointParams[JointId::COUNT] = {
        {90.0f,180.0f}, //yaw
        {60.0f,120.0f}, //pitch1
        {80.0f,160.0f}, //pitch2, 
        {90.0f,180.0f}, //pitch3
        {120.0f,300.0f}, //roll,
        {120.0f,300.0f}, //pitch_end
        {60.0f,120.0f}, //end_roll,
        }; // 每个关节的运动参数
    } TrajConfig;  //

    float_t speedScale = 1.0f; ///< 速度比例系数，>1加速，<1减速

    /**
     * @brief 规划从 current 到 target 的运动
     * @param current 7个关节当前角度
     * @param target  7个关节目标角度
     */
    void PlanMultiAxisTraj(const float_t current[JointId::COUNT], const float_t target[JointId::COUNT]) ;
    
    /**
     * @brief 在时刻 t 计算 7 个关节位置
     * @param t 从运动开始经过的时间 (s)
     * @param out 输出 7 个关节位置
     */
    void MultiAxisTrajDistance(float_t t, float_t output[JointId::COUNT]) const;

    bool IsFinished(float_t t)const {return t >= maxTime_;} // 判断轨迹是否规划完成
    float_t GetMaxTime() const { return maxTime_; }


private:
    STrapezoidalSpeed jointTrajs[JointId::COUNT]; // 每个关节各自的轨迹规划器
    float_t maxTime_ = 0; // 所有关节完成运动所需的最长时间
    
};
} // namespace my_engineer

#endif // ALGO_TRAJ_PLAYBACK_HPP