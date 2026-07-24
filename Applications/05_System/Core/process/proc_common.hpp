/******************************************************************************
 * @brief        轨迹回放公共接口与数据结构
 *
 * @file         proc_common.hpp
 * @author       ciallo
 * @version      V1.0
 * @date         2026-6-14
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

#define STORE_ROLL_UP_OFFSET    -180.0f  ///< 末端 roll 翻转偏移（朝上）
#define STORE_ROLL_DOWN_OFFSET  0.0f     ///< 末端 roll 默认偏移（朝下）

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
 *     - PlayJointTarget()  按目标关节角播一段（输入 target[J::COUNT] 数组）
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
 *   - 手里已经有 target 数组（不是 traj）   - PlayJointTarget
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
 *   ii. 分段策略（前段逐帧到位 + 后段样条平滑）
 *   float_t lastTarget[J::COUNT];
 *   int gripCloseSeg = ...;    // 找夹爪闭合帧
 *
 *   // 前段：逐段到位（lastTarget 输出给后段用）
 *   if (!PlayTrajRows(arm, clip.frame, gripCloseSeg, lastTarget)) return false;
 *
 *   // Shift 确认
 *
 *   // 后段：五次样条平滑拔出
 *   static CAlgoQuinticSpline spline;
 *   if (!PlaySplineRange(arm, clip, gripCloseSeg, clip.frameCount - 1, spline)) return false;
 *
 *
 * 4.关于内部约定
 *   - 所有 Play 函数在执行期间持续监测 Ctrl+Z，操作手可随时打断
 *   - Play 函数返回值约定：true=成功  false=失败（Ctrl+Z 打断 / 关节超时 / 夹爪超时 / 跟踪误差过大）
 *   - 关节 P2P 算法统一走 CAlgoQuintic 五次多项式（algo_quintic.hpp）
 *   - 多段平滑走 CAlgoQuinticSpline 五次样条（algo_quintic_spline.hpp）
 *   - 轨迹二维数组列含义见 FrameCol 枚举（6 轴: FC_TIME/YAW/P1/P2/ROLL/ENDP/ENDR/GRIP/SPEED）
 *
 * 5.PlayTrajRows 和 PlaySplineRange 之间的区别
 *   - PlayTrajRows：中间有停顿等待到位检查
 *   - PlaySplineRange：直接连续的插值不需要等待到位
 */

namespace my_engineer{

    using J = CAlgoTrajPlayback::JointId;

    //轨迹id
    enum ETrajID : uint8_t {
        TRAJ_EXCHANGE_L = 0,          ///< 兑左矿
        TRAJ_EXCHANGE_R,              ///< 兑右矿
        TRAJ_STORE_L,                 ///< 存左矿
        TRAJ_STORE_R,                 ///< 取右矿
        TRAJ_AUTO,                    ///< 自动取矿
        TRAJ_COUNT,
    };

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
            FC_COUNT = 9,    // 总列数
        };

    struct TrajClip{
        const float_t (*frame)[FC_COUNT]; // 指向关键帧的数组
        int frameCount ; //关键帧计数器
    };

    // 自动取矿部分
    struct SOreStep {
        TrajClip getClip;                       // 取矿
        TrajClip storeClip;                     // 存矿
        float_t  rollOff = 0.0f;                // 末端 roll 偏移（叠加在轨迹 endR 上）
    };

    struct SArrivalCheckConfig {
        float_t toleranceDeg = 5.0f;     ///< 关节到位容差，单位：度
        uint32_t stableMs = 100;          ///< 每个关节进入容差后需要连续稳定的时间
        uint32_t timeoutMs = 1000;        ///< 本帧目标指令到达后，等待真实反馈到位的报警时间
        uint32_t hardTimeoutMs = 8000;    ///< 本帧目标指令到达后，等待真实反馈到位的硬超时时间
        uint32_t gripTimeoutMs = 3000;   ///< 本帧目标指令到达后，等待夹爪到位的超时时间
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
        const CAlgoTrajPlayback::SJointPrarm *jointParamsOverride = nullptr;  // 自定义关节速度/加速度
        float_t minTimeS        = 0.0f;   // 最小时长
        bool    waitForArrival  = true;   // 是否等关节到位
        bool    gripKeepCurrent = false;  // 保持当前的夹爪的姿态
    };

    // FastJointParams 定义在 algo_traj_playback.hpp（紧贴 SJointPrarm 类型）

    //轨迹外部声明
    extern const float_t Traj_Grab[][FC_COUNT];
    extern const int     Traj_GrabLen;

    // 存矿任务 Runner
    class CStoreOreTaskRunner final {
    public:
        explicit CStoreOreTaskRunner(CSystemCore &core)
            : core_(core),
              arm_(*core.parm_),
              keyboard_(SysRemote.remoteInfo.keyboard),
              edge_(SysRemote.remoteInfo.keyboard_edge) {}

        bool RunManualStoreTask(ETrajID trajId);
        bool RunAutoOreTask();
        bool PlayGetClip(const TrajClip &clip, float_t rollOff = 0.0f);
        bool AlignEndRollToClipStart(const TrajClip &clip, float_t rollOff);

        CSystemCore &core_;
        CModArm &arm_;
        CSystemRemote::SKeyboardInfo &keyboard_;
        CSystemRemote::SKeyboardEdge &edge_;
        float_t endRollOffset_ = STORE_ROLL_DOWN_OFFSET;
    };

    //从 armInfo 读取 J::COUNT 个关节角度到 float_t[J::COUNT]
    void ReadArmjoint(const CModArm &arm, float_t output[J::COUNT]);

    // 检查所有关节是否到达目标角度
    bool CheckAllJointsArrived(const CModArm &arm, const float_t target[J::COUNT],
                               const SArrivalCheckConfig &cfg);

    //将 float_t[J::COUNT] 写入 armCmd 的 J::COUNT 个关节
    void WriteArmjoint(CModArm &arm, const float_t output[J::COUNT]);
    //拷贝数组
    void CopyArmJoint(float_t inputjoint[J::COUNT],const float_t targetjoint[J::COUNT]);
    //从轨迹二维数组提取第 row 行的 J::COUNT 个关节数据
    void Extrarow(const float_t traj[][FC_COUNT], int row, float_t output[J::COUNT]);

    //提取第 row 行的夹爪状态：返回true=夹紧，false=松开
    bool ExtractGripClose(const float_t traj[][FC_COUNT], int row);

    // 五次 P2P 播放器
    bool PlayJointTarget(CModArm &arm,
                         const float_t target[J::COUNT],
                         const SPlayJointTargetOptions &opt);

    // 单关节平滑规划到绝对角度（其余关节保持当前位姿，避免直接赋值的冲击）
    // 速度/加速度/夹爪策略由调用方通过 opt 传入，函数本身不硬编码任何参数
    bool MoveSingleJointToAbs(CModArm &arm,
                              int jointIdx,
                              float_t targetAbs,
                              const SPlayJointTargetOptions &opt);

    // 按轨迹第 seg 行播放一段
    bool PlayTrajRow(CModArm &arm,
                     const float_t traj[][FC_COUNT], int seg,
                     const float_t *prevTarget = nullptr,
                     bool waitForArrival = true,
                     float_t endRollOffset = 0.0f,
                     bool earlyGrip = false);

    // 按 traj 数组的第 0..segEnd-1 行逐段播放
    bool PlayTrajRows(CModArm &arm,
                      const float_t traj[][FC_COUNT],
                      int segEnd,
                      float_t *lastTargetOut = nullptr,
                      float_t endRollOffset = 0.0f,
                      bool earlyGrip = false);

    // 五次插值连续播放
    bool PlaySplineRange(CModArm &arm,
                         const TrajClip &clip,
                         int segFrom,
                         int segTo,
                         CAlgoQuinticSpline &spline,
                         float_t rollOff = 0.0f,
                         float_t speedScale = 1.0f);
    // 自定义参数设置
    void SetParam(CAlgoQuinticSpline &spline, CAlgoTrajPlayback::SConfig &param);
}

#endif // PROC_TRAJ_COMMON_HPP
