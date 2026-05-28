/******************************************************************************
 * @brief        
 * 
 * @file         proc_traj_data.hpp
 * @author       ciallo
 * @version      V1.0
 * @date         2026-5-28
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/
#ifndef TRAJ_DATA_HPP
#define TRAJ_DATA_HPP

#include "proc_common.hpp"

namespace my_engineer {

// 存矿石轨迹
extern const SOreStep OreStepConfig[];
extern const int OreStepCount;

// 兑换轨迹
extern const TrajClip ExchangeClipL;
extern const TrajClip ExchangeClipR;

} // namespace my_engineer

#endif