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
#include "algo_quintic_spline.hpp"
#include "algo_quintic.hpp"
#include <map>
#include <cmath>

#define STORE_ROLL_UP_OFFSET  -160.0f  //标定的时候出现偏差导致末端并不是水平
#define STORE_ROLL_DOWN_OFFSET 0.0f
/**
/-------------------------------------如何使用此轨迹回放库----------------------------------------------/
 * @brief 由于本文件函数较多且名字相近（PlayJointTarget/PlayTrajRow/PlayTrajRows/PlaySplineRange），
 *        因此写使用说明，帮助快速选型与理解函数关系
 * @author ciallo
 * @date 2026-6-14
 * @details 配套 proc_traj_common.cpp 实现；算法层依赖 algo_quintic.hpp / algo_quintic_spline.hpp
 *
 *
 * 1.函数调用关系
 *
 *   应用层任务函数（StartStoreTask / StartExchangeGetTask / ...）
 *        ↓
 *   高层 - 多段循环
 *     - PlayTrajRows()     按轨迹 0..segEnd-1 段连续播放
 *     - PlaySplineRange()  五次样条段范围连续播放（需要平滑衔接时用）
 *        ↓ 内部循环调用
 *   中层 - 单段播放
 *     - PlayTrajRow()      按轨迹第 seg 行播一段（输入 traj 二维数组）
 *     - PlayJointTarget()  按目标关节角播一段（输入 target[7] 数组）
 *        ↓ 内部调用
 *   底层 - 一次性工具（无循环，可独立使用）
 *     - ReadArmjoint / WriteArmjoint             关节角度读写
 *     - Extrarow / ExtractGripClose              提取轨迹行/夹爪状态
 *     - WriteGripCommand / CheckGripArrived      夹爪命令/到位检查
 *     - CheckAllJointsArrived / WaitGripArrived  关节到位检查/夹爪到位等待
 *
 *
 * 2.决策树 
 *   - 播完整段轨迹（多个关键帧）           - PlayTrajRows
 *   - 只播轨迹里的某一帧                   - PlayTrajRow（例如 Shift 确认后的单帧）
 *   - 手里已经有 target 数组（不是 traj）   - PlayJointTarget（例如 AlignEndRollToStore）
 *   - 多段需要平滑衔接（不是逐段到位）      - PlaySplineRange（例如抓取后的拔出段）
 *
 *
 * 3.典型使用示例
 *
 *   i. 播完整段轨迹
 *   auto &Traj = ExchangesingleClip_L;
 *   arm.armCmd.isAutoCtrl = true;
 *   if (!PlayTrajRows(arm, Traj.frame, Traj.frameCount)) {
 *       goto proc_exit;   // Ctrl+Z 打断 / 关节超时 / 夹爪超时
 *   }
 *
 *   ii. 分段：前段逐帧到位之后段样条平滑
 *   float_t lastTarget[J::COUNT];
 *   int gripCloseSeg = ...;    // 找夹爪闭合帧
 *
 *   // 前段：逐段到位（lastTarget 输出给后段用）
 *   if (!PlayTrajRows(arm, clip.frame, gripCloseSeg, rollOff, false, lastTarget)) return false;
 *
 *   // Shift 确认
 *
 *   // 后段：五次插值平滑拔出
 *   static CAlgoQuinticSpline spline;
 *   if (!PlaySplineRange(arm, clip, gripCloseSeg, clip.frameCount - 1, spline, rollOff)) return false;
 *
 *   iii. 临时对齐某个关节（不是播放轨迹）
 *   float_t preAlignTarget[J::COUNT];
 *   ReadArmjoint(arm, preAlignTarget);                       // 起点为当前位姿
 *   preAlignTarget[J::J_ENDR] = targetEndRoll + rollOff;     // 仅修改末端 roll
 *
 *   SPlayJointTargetOptions opt;
 *   opt.speedScale = 10.0f;
 *   opt.gripDuringMotion = gripNow;
 *   opt.gripAfter = gripNow;
 *   if (!PlayJointTarget(arm, preAlignTarget, opt)) return false;
 *
 *
 * 4.关于内部约定
 *   - 所有 Play 函数在执行期间持续监测 Ctrl+Z，操作手可随时打断
 *   - 失败（Ctrl+Z / 关节超时 / 夹爪超时）统一返回 false，由调用方 goto proc_exit 收尾
 *   - 关节 P2P 算法统一走 CAlgoQuintic 五次多项式（algo_quintic.hpp）
 *   - 多段平滑走 CAlgoQuinticSpline 五次样条（algo_quintic_spline.hpp）
 *   - 轨迹二维数组列含义见 FrameCol 枚举（FC_TIME/YAW/P1/P2/P3/ROLL/ENDP/ENDR/GRIP/SPEED）
 * 
 * 5.PlayTrajRows 和 PlaySplineRange之间的区别是一个是中间有停顿等待到位检查，一个是直接连续的插值不需要等待到位
 */

namespace my_engineer{
    
    using J = CAlgoTrajPlayback::JointId;
    
    //轨迹id
    enum ETrajID : uint8_t {
        TRAJ_EXCHANGE_L = 0,          ///< 兑左矿
        TRAJ_EXCHANGE_R,              ///< 兑右矿
        TRAJ_STORE_L,               ///< 存左矿
        TRAJ_STORE_R,               ///< 取右矿
        TRAJ_AUTO,                ///< 自动取矿
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

    // 自动取矿部分
    struct SOreStep {
        TrajClip getClip;                       // 取矿
        TrajClip storeClip;                     // 存矿
    };

    struct SArrivalCheckConfig {
        float_t toleranceDeg = 5.0f;     ///< 关节到位容差，单位：度
        uint32_t stableMs = 100;          ///< 每个关节进入容差后需要连续稳定的时间
        uint32_t hardTimeoutMs = 8000;    ///< 本帧目标指令到达后，等待真实反馈到位的硬超时时间
    };

    // PlayJointTarget 内部状态变量封装
    struct SPlayJointTargetState {
        bool arrivalCheckStarted = false;
        bool stableTiming       = false;
        bool frameJointsArrived = false;
        uint32_t arrivalStartTick = 0;
        uint32_t stableStartTick  = 0;
    };

    // PlayJointTarget 参数包
    struct SPlayJointTargetOptions {
        float_t speedScale      = 1.0f;
        bool    gripDuringMotion = true;   // 运动期间夹爪状态
        bool    gripAfter        = true;   // 到位后夹爪状态
        const float_t *startOverride = nullptr;  // 规划起点覆盖
        float_t minTimeS        = 0.0f;   // 最小时长
        bool    waitForArrival  = true;   // 是否等关节到位
    };
    //轨迹外部声明
    extern const float_t Traj_Grab[][FC_COUNT];
    extern const int     Traj_GrabLen;
    
    class CStoreOreTaskRunner final {
    public:
        explicit CStoreOreTaskRunner(CSystemCore &core)
            : core_(core),
              arm_(*core.parm_),
              keyboard_(SysRemote.remoteInfo.keyboard),
              edge_(SysRemote.remoteInfo.keyboard_edge) {}

        bool RunManualStoreTask(ETrajID trajId);
        bool RunAutoOreTask();

        bool PlayGetClip(const TrajClip &clip, float_t rollOff);
        bool AlignEndRollToStore(const TrajClip &clip, float_t rollOff);

        CSystemCore &core_;
        CModArm &arm_;
        CSystemRemote::SKeyboardInfo &keyboard_;
        CSystemRemote::SKeyboardEdge &edge_;
        float_t endRollOffset_ = STORE_ROLL_DOWN_OFFSET;
    };
    
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

    // 五次播放器
    bool PlayJointTarget(CModArm &arm,
                     const float_t target[J::COUNT],
                     const SPlayJointTargetOptions &opt);

    bool PlayTrajRow(CModArm &arm,
                          const float_t traj[][FC_COUNT], int seg,
                          float_t endRollOffset = 0.0f,
                          const float_t *prevTarget = nullptr,
                          bool earlyGrip = false,
                          bool waitForArrival = true);

    bool PlayTrajRows(CModArm &arm,
                        const float_t traj[][FC_COUNT],
                        int segEnd,
                        float_t endRollOffset = 0.0f,
                        bool earlyGrip = false,
                        float_t *lastTargetOut = nullptr);

    // 五次插值函数
    bool PlaySplineRange(CModArm &arm,
                             const TrajClip &clip,
                             int segFrom,
                             int segTo,
                             CAlgoQuinticSpline &spline,
                             float_t rollOff = 0.0f);


}

#endif // PROC_TRAJ_COMMON_HPP
