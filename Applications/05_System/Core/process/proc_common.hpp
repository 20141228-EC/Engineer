/******************************************************************************
 * @brief        
 * 
 * @file         proc_common.hpp
 * @author       ciallo
 * @version      V1.0
 * @date         2026-4-21
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/
#ifndef PROC_COMMON_HPP
#define PROC_COMMON_HPP

#include "Core.hpp"
#include <cmath>

#define STORE_ROLL_UP_OFFSET  -160.0f  //标定的时候出现偏差导致末端并不是水平
#define STORE_ROLL_DOWN_OFFSET 0.0f

namespace my_engineer{
    
    using J = CAlgoTrajPlayback::JointId;

    enum FrameCol : uint8_t {
            FC_TIME  = 0,
            FC_YAW   = 1,
            FC_P1    = 2,
            FC_P2    = 3,
            FC_ROLL  = 4,
            FC_ENDP  = 5,
            FC_ENDR  = 6,
            FC_GRIP  = 7,
            FC_SPEED = 8,
            FC_COUNT  = 9,    // 总列数
        };

    struct TrajClip{
        const float_t (*frame)[FC_COUNT]; // 指向关键帧的数组
        int frameCount ; //关键帧计数器
    };

    struct SOreStep {
        CModGimbal::EStorageSlot slot;          // 存矿电机槽位
        TrajClip getClip;                       // 取矿
        TrajClip storeClip;                     // 存矿
        bool enabled;                           // 是否启用
    };

    struct SArrivalCheckConfig {
        float_t toleranceDeg = 5.0f;     ///< 关节到位容差，单位：度
        uint32_t stableMs = 100;          ///< 每个关节进入容差后需要连续稳定的时间
        uint32_t timeoutMs = 1000;        ///< 本帧目标指令到达后，等待真实反馈到位的报警时间
        uint32_t hardTimeoutMs = 8000;    ///< 本帧目标指令到达后，等待真实反馈到位的硬超时时间
        uint32_t gripTimeoutMs = 3000;   ///< 本帧目标指令到达后，等待夹爪到位的超时时间
    };
    //从 armInfo 读取关节角度到 float_t[J::COUNT]
    void ReadArmjoint(const CModArm &arm, float_t output[7]);

    // 检查所有关节是否到达目标角度
    bool CheckAllJointsArrived(const CModArm &arm, const float_t target[J::COUNT],
                               const SArrivalCheckConfig &cfg);
    
    //将 float_t[7] 写入 armCmd 的7个关节 
    void WriteArmjoint(CModArm &arm, const float_t output[7]);
    
    //从轨迹二维数组提取第 row 行的7个关节数据 
    void Extrarow(const float_t traj[][FC_COUNT], int row, float_t output[J::COUNT]);
    
    //提取第 row 行的夹爪状态：返回true=夹紧，false=松开 
    bool ExtractGripClose(const float_t traj[][FC_COUNT], int row);

    // 提取第 row 行的速度比例（列9）
    float_t ExtractSpeed(const float_t traj[][FC_COUNT], int row);
    
    //梯形减速播放器
    bool PlaySegment(CModArm &arm, const float_t target[J::COUNT], float_t speedScale,
                     bool gripDuringMotion, bool gripAfter,
                     CAlgoTrajPlayback &player, bool checkctrl,
                     const float_t *startOverride = nullptr,
                     float_t minTimeS = 0.0f);

    //完整的封装
    // earlyGrip: true = 在关节运动一开始就切换夹爪，false = 关节到位后才切换
    bool PlayFrameSegment(CModArm &arm, const float_t traj[][FC_COUNT], int seg,
                          CAlgoTrajPlayback &player, bool checkctrl,
                          float_t endRollOffset = 0.0f,
                          const float_t *prevTarget = nullptr,
                          bool earlyGrip = false);
}

#endif // PROC_TRAJ_COMMON_HPP
