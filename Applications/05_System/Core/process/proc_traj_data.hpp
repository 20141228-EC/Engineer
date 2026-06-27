/******************************************************************************
 * @brief
 *
 * @file         proc_traj_data.hpp
 * @author       ciallo
 * @version      V2.0
 * @date         2026-06-07
 *
 * @copyright    Copyright (c) 2026
 *
 ******************************************************************************/
#ifndef TRAJ_DATA_HPP
#define TRAJ_DATA_HPP

#include "proc_common.hpp"

namespace my_engineer {

extern const SOreStep OreStepConfig[];
extern const int OreStepCount;

// 单个存矿轨迹（左/右）
extern const TrajClip StoresingleClip_L;
extern const TrajClip StoresingleClip_R;

// 单个兑矿轨迹（左/右）
extern const TrajClip ExchangesingleClip_L;
extern const TrajClip ExchangesingleClip_R;

} // namespace my_engineer

#endif
