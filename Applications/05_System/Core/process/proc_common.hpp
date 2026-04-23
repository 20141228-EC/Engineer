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

    //轨迹外部声明
    extern const float_t Traj_Grab[][FC_COUNT];
    extern const int     Traj_GrabLen; 
    
    //轨迹图管理所有的轨迹
    extern std::map<ETrajID,TrajClip> TrajMap;
    
    //从 armInfo 读取7个关节角度到 float_t[7] 
    void ReadArmjoint(const CModArm &arm, float_t output[7]);
    
    //将 float_t[7] 写入 armCmd 的7个关节 
    void WriteArmjoint(CModArm &arm, const float_t output[7]);
    
    //从轨迹二维数组提取第 row 行的7个关节数据 
    void Extrarow(const float_t traj[][FC_COUNT], int row, float_t output[7]);
    
    //提取第 row 行的夹爪状态：返回true=夹紧，false=松开 
    bool ExtractGripClose(const float_t traj[][FC_COUNT], int row);

    // 提取第 row 行的速度比例（列9）
    float_t ExtractSpeed(const float_t traj[][FC_COUNT], int row);
    
    //梯形减速播放器
    bool PlaySegment(CModArm &arm, const float_t target[J::COUNT],float_t speedScale,
          bool gripClose,CAlgoTrajPlayback &player, bool checkctrl );

    //完整的封装
    bool PlayFrameSegment(CModArm &arm,const float_t traj[][FC_COUNT], int seg,
                                CAlgoTrajPlayback &player, bool checkctrl);
}

#endif // PROC_TRAJ_COMMON_HP