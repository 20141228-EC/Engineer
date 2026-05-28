/******************************************************************************
 * @brief        
 * 
 * @file         proc_traj_data.cpp
 * @author       ciallo
 * @version      V1.0
 * @date         2026-5-28
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/
#include "proc_traj_data.hpp"

namespace my_engineer {

/*---------------------------------取六矿----------------------------------------------*/
// 编号与存矿槽位一一对应，角度待测量后填入

const float_t Traj_Get_Ore1[][FC_COUNT] = {
    // time  yaw  p1  p2  roll  endP  endR  grip  speed
    {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 4.f},  
};
const int Traj_GetLen_Ore1 = sizeof(Traj_Get_Ore1) / sizeof(Traj_Get_Ore1[0]);

const float_t Traj_Get_Ore2[][FC_COUNT] = {
    {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 4.f},  
};
const int Traj_GetLen_Ore2 = sizeof(Traj_Get_Ore2) / sizeof(Traj_Get_Ore2[0]);

const float_t Traj_Get_Ore3[][FC_COUNT] = {
    {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 4.f},  
};
const int Traj_GetLen_Ore3 = sizeof(Traj_Get_Ore3) / sizeof(Traj_Get_Ore3[0]);

const float_t Traj_Get_Ore4[][FC_COUNT] = {
    {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 4.f},  
};
const int Traj_GetLen_Ore4 = sizeof(Traj_Get_Ore4) / sizeof(Traj_Get_Ore4[0]);

const float_t Traj_Get_Ore5[][FC_COUNT] = {
    {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 4.f},  
};
const int Traj_GetLen_Ore5 = sizeof(Traj_Get_Ore5) / sizeof(Traj_Get_Ore5[0]);

const float_t Traj_Get_Ore6[][FC_COUNT] = {
    {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 4.f},  
};
const int Traj_GetLen_Ore6 = sizeof(Traj_Get_Ore6) / sizeof(Traj_Get_Ore6[0]);

/*--------------------------------存矿矿轨迹-------------------------------------------*/
// 存矿电机会旋转把对应槽位转到固定位置，所以存矿动作只有左/右两种

const float_t Traj_Store_L[][FC_COUNT] = {
    // time  yaw  p1  p2  roll  endP  endR  grip  speed
    {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 4.f},  
};
const int Traj_StoreLen_L = sizeof(Traj_Store_L) / sizeof(Traj_Store_L[0]);

const float_t Traj_Store_R[][FC_COUNT] = {
    // time  yaw  p1  p2  roll  endP  endR  grip  speed
    {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 4.f},  
};
const int Traj_StoreLen_R = sizeof(Traj_Store_R) / sizeof(Traj_Store_R[0]);

/*--------------------------------兑矿轨迹-------------------------------------------*/
static const float_t Traj_Exchange_L[][FC_COUNT] = {
    // time  yaw  p1  p2  roll  endP  endR  grip  speed
    {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 4.f}, 
};
const TrajClip ExchangeClipL = {Traj_Exchange_L,sizeof(Traj_Exchange_L) / sizeof(Traj_Exchange_L[0])};

static const float_t Traj_Exchange_R[][FC_COUNT] = {
    // time  yaw  p1  p2  roll  endP  endR  grip  speed
    {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 4.f},  
};
const TrajClip ExchangeClipR = {Traj_Exchange_R, sizeof(Traj_Exchange_R) / sizeof(Traj_Exchange_R[0])};


// 1~3号矿存在左侧（用 Traj_Store_L），4~6号矿存在右侧（用 Traj_Store_R）
// 存矿电机负责旋转到具体槽位

const SOreStep OreStepConfig[] = {
    //     槽位                  取矿帧                              存矿帧                      启用
    {  CModGimbal::EStorageSlot::UP,      {Traj_Get_Ore1,   Traj_GetLen_Ore1},  {Traj_Store_L, Traj_StoreLen_L},  true  },
    {  CModGimbal::EStorageSlot::L_UP,    {Traj_Get_Ore2,   Traj_GetLen_Ore2},  {Traj_Store_L, Traj_StoreLen_L},  true  },
    {  CModGimbal::EStorageSlot::L_DOWN,  {Traj_Get_Ore3,   Traj_GetLen_Ore3},  {Traj_Store_L, Traj_StoreLen_L},  true  },
    {  CModGimbal::EStorageSlot::DOWN,    {Traj_Get_Ore4,   Traj_GetLen_Ore4},  {Traj_Store_R, Traj_StoreLen_R},  true  },
    {  CModGimbal::EStorageSlot::R_DOWN,  {Traj_Get_Ore5,   Traj_GetLen_Ore5},  {Traj_Store_R, Traj_StoreLen_R},  true  },
    {  CModGimbal::EStorageSlot::R_UP,    {Traj_Get_Ore6,   Traj_GetLen_Ore6},  {Traj_Store_R, Traj_StoreLen_R},  true  },
};
const int OreStepCount = sizeof(OreStepConfig) / sizeof(OreStepConfig[0]);

} // namespace my_engineer