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
#include <map>
#include <cmath>

#define STORE_ROLL_UP_OFFSET  -160.0f  //标定的时候出现偏差导致末端并不是水平
#define STORE_ROLL_DOWN_OFFSET 0.0f

namespace my_engineer{
    
    using J = CAlgoTrajPlayback::JointId;
    
    //轨迹id
    enum ETrajID : uint8_t {
        TRAJ_GRAB_L = 0,          ///< 存矿
        TRAJ_GRAB_R,
        TRAJ_GET_L,               ///< 取矿
        TRAJ_GET_R,               ///< 取矿
        // TRAJ_GROUND,          ///< 地矿
        // TRAJ_HOME,            ///< 回零
        TRAJ_COUNT,
    };

    enum FrameCol : uint8_t {
            FC_TIME  = 0,
            FC_YAW   = 1,
            FC_P1    = 2,
            FC_P2    = 3,
            FC_P3    = 4,
            FC_ROLL  = 5,
            FC_ENDP  = 6,
            FC_ENDR  = 7,
            FC_GRIP  = 8,
            FC_SPEED = 9,   
            FC_COUNT  = 10,   // 总列数
        };

    struct TrajClip{
        const float_t (*frame)[FC_COUNT]; // 指向关键帧的数组
        int frameCount ; //关键帧计数器
    };

    struct SArrivalCheckConfig {
        float_t toleranceDeg = 3.0f;     ///< 关节到位容差，单位：度
        uint32_t stableMs = 100;          ///< 每个关节进入容差后需要连续稳定的时间
        uint32_t timeoutMs = 1000;        ///< 本帧目标指令到达后，等待真实反馈到位的报警时间
        uint32_t hardTimeoutMs = 8000;    ///< 本帧目标指令到达后，等待真实反馈到位的硬超时时间
        uint32_t gripTimeoutMs = 3000;   ///< 本帧目标指令到达后，等待夹爪到位的超时时间
    };
    //轨迹外部声明
    extern const float_t Traj_Grab[][FC_COUNT];
    extern const int     Traj_GrabLen; 
    
    //轨迹图管理所有的轨迹
    extern std::map<ETrajID,TrajClip> TrajMap;
    
    //从 armInfo 读取7个关节角度到 float_t[7] 
    void ReadArmjoint(const CModArm &arm, float_t output[7]);

    // 检查所有关节是否到达目标角度
    bool CheckAllJointsArrived(const CModArm &arm, const float_t target[J::COUNT],
                               const SArrivalCheckConfig &cfg);
    
    //将 float_t[7] 写入 armCmd 的7个关节 
    void WriteArmjoint(CModArm &arm, const float_t output[7]);
    
    //从轨迹二维数组提取第 row 行的7个关节数据 
    void Extrarow(const float_t traj[][FC_COUNT], int row, float_t output[7]);
    
    //提取第 row 行的夹爪状态：返回true=夹紧，false=松开 
    bool ExtractGripClose(const float_t traj[][FC_COUNT], int row);

    // 提取第 row 行的速度比例（列9）
    float_t ExtractSpeed(const float_t traj[][FC_COUNT], int row);
    
    //梯形减速播放器
    //  gripDuringMotion : 关节运动过程中保持的夹爪状态（true=夹紧, false=松开）
    //  gripAfter        : 关节到位之后才切换的夹爪状态（即本段目标）
    //  startOverride 非空时，使用其作为规划起点（避免每段用反馈起点导致路径漂移）
    //  minTimeS      可选最小总时长(秒)，0 = 按物理参数自由规划（默认）；
    //                 仅用于"安全慢速模式"等强制拖慢场景，正常播放不应使用，
    //                 否则物理上能更快完成的段会被无谓拖长。
    bool PlaySegment(CModArm &arm, const float_t target[J::COUNT], float_t speedScale,
                     bool gripDuringMotion, bool gripAfter,
                     CAlgoTrajPlayback &player, bool checkctrl,
                     const float_t *startOverride = nullptr,
                     float_t minTimeS = 0.0f);

    //完整的封装
    //  prevTarget 非空：用上一段 target 做起点
    //  prevTarget 为空：用实时反馈做起点
    bool PlayFrameSegment(CModArm &arm, const float_t traj[][FC_COUNT], int seg,
                          CAlgoTrajPlayback &player, bool checkctrl,
                          float_t endRollOffset = 0.0f,
                          const float_t *prevTarget = nullptr);
}

#endif // PROC_TRAJ_COMMON_HPP
