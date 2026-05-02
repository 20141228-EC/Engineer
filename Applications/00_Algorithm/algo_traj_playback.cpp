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
     */
    
    void CAlgoTrajPlayback::PlanMultiAxisTraj(const float_t current[JointId::COUNT], const float_t target[JointId::COUNT]) {
        maxTime_ = 0; //用于记录最大速度
        // 速度比例系数应用到各关节
        for (int i = 0; i < JointId::COUNT; i++) {
            float_t vel = TrajConfig.jointParams[i].velMax * speedScale;
            float_t acc = TrajConfig.jointParams[i].accMax * speedScale;
            jointTrajs[i].TrapezoidalSpeedPlanner(current[i], target[i], vel, acc);//求出相应的时间和距离
            if (jointTrajs[i].tTotal > maxTime_) {
                maxTime_ = jointTrajs[i].tTotal;//选择排序找到最大的时间
            }
        }
        // 各轴协调速度同步
        if(maxTime_ > 1e-3f){
            for(int i = 0; i < JointId::COUNT; i++){
                if(jointTrajs[i].tTotal > 1e-3f && jointTrajs[i].tTotal < maxTime_ * 0.95f ){
                    float_t scale = jointTrajs[i].tTotal / maxTime_;
                    float_t vel = TrajConfig.jointParams[i].velMax * speedScale;
                    float_t acc = TrajConfig.jointParams[i].accMax * speedScale;
                    jointTrajs[i].TrapezoidalSpeedPlanner(current[i], target[i], vel * scale, acc * scale * scale);//降速并将时间同步到maxTime_
                }
            }
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
