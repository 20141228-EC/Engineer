/******************************************************************************
 * @brief
 *
 * @file         traj_data.cpp
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
/*
 *
         1
        / \
       /   \
      2     6
      |     |
      3     5
       \   /
        \ /
         4
*/
// 编号与存矿槽位一一对应

const float_t Traj_Get_Ore1[][FC_COUNT] = {
        //    time    yaw       p1        p2       roll      endP       endR    grip  speed
        {     0,   -44.154f,   35.010f,  68.057f,  -40.160f,   -28.747f,  180.025f, 0, 2.5f},  // 起始位
};
const int Traj_GetLen_Ore1 = sizeof(Traj_Get_Ore1) / sizeof(Traj_Get_Ore1[0]);

const float_t Traj_Get_Ore2[][FC_COUNT] = {
        //    time    yaw       p1        p2       roll      endP       endR    grip  speed
        {     0,   -34.154f,   35.010f,  68.057f,  -40.160f,   -28.747f,  180.025f, 1, 1.5f},  // 起始位
        {  1000,   -34.822f,   58.004f,  84.225f,  -48.154f,   -21.960f,  180.140f, 1, 1.5f},  // 
        {  2000,   -27.926f,   67.905f,  90.085f,  -45.186f,   -19.133f,  180.146f, 1, 3.0f},  // 
        {  3000,   -27.926f,   67.905f,  90.085f,  -45.186f,   -19.133f,  180.146f, 0, 3.0f},  //
        {  4000,   -31.464f,   73.984f,  97.030f,  -44.575f,   -15.411f,  178.451f, 0, 3.0f},  // 
        {  5000,   -46.936f,   77.373f, 113.975f,  -44.125f,   -27.595f,  179.325f, 0, 3.0f},  // 
        {  6000,   -54.120f,   84.439f, 124.256f,  -44.109f,   -32.666f,  174.364f, 0, 3.0f},  // 
        {  1000,   -62.822f,   59.004f,  52.225f,  12.154f,    -0.960f,   180.140f, 0, 4.5f},  // 
};
const int Traj_GetLen_Ore2 = sizeof(Traj_Get_Ore2) / sizeof(Traj_Get_Ore2[0]);

const float_t Traj_Get_Ore3[][FC_COUNT] = {
        //    time    yaw       p1        p2       roll      endP       endR    grip  speed
        {     0,   -44.154f,   35.010f,  68.057f,  -40.160f,   -28.747f,  180.025f, 0, 2.5f},  // 起始位
};
const int Traj_GetLen_Ore3 = sizeof(Traj_Get_Ore3) / sizeof(Traj_Get_Ore3[0]);

const float_t Traj_Get_Ore4[][FC_COUNT] = {
        //    time    yaw       p1        p2       roll      endP       endR    grip  speed
        {     0,    -4.170f,    2.917f,  29.206f,   8.138f,   -15.735f,  1.235f, 1, 1.5f},
        {  1000,    -1.228f,   58.998f,  44.038f,  14.571f,   27.744f,   1.571f, 1, 1.5f},
        {  2000,    -1.228f,   58.998f,  44.038f,  14.571f,   27.744f,   1.571f, 1, 1.5f},
        {  3000,    -1.337f,   61.959f,  39.845f,  10.759f,   34.945f,   1.156f, 0, 3.0f},
        {  4000,    -2.957f,   62.948f,  33.245f,  12.516f,   42.345f,   0.631f, 0, 3.0f}, 
        {  5000,    -0.296f,   65.134f,  32.627f,  10.726f,   44.679f,   1.112f, 0, 3.0f},
        {  6000,     1.720f,   71.006f,  25.027f,   6.858f,   52.090f,   1.156f, 0, 3.0f},
        {  7000,     3.903f,   38.958f,  25.064f,  14.115f,  -10.528f,   1.177f, 0, 3.0f},
        {  8000,     3.993f,    0.231f,  10.899f,  13.093f,   12.873f,   1.156f, 0, 3.0f}, 
};
const int Traj_GetLen_Ore4 = sizeof(Traj_Get_Ore4) / sizeof(Traj_Get_Ore4[0]);

const float_t Traj_Get_Ore5[][FC_COUNT] = {
        //    time    yaw       p1        p2       roll      endP       endR    grip  speed
        {     0,   -44.154f,   35.010f,  68.057f,  -40.160f,   -28.747f,  180.025f, 0, 2.5f},  // 起始位

};
const int Traj_GetLen_Ore5 = sizeof(Traj_Get_Ore5) / sizeof(Traj_Get_Ore5[0]);

const float_t Traj_Get_Ore6[][FC_COUNT] = {
        //    time    yaw       p1        p2       roll      endP       endR    grip  speed
        {     0,    61.228f,   56.850f,  90.123f,  82.720f,  -58.945f,  179.915f, 1, 1.5f},
        {  1000,    33.026f,   74.017f,  99.081f,  71.629f,  -40.823f,  179.784f, 1, 1.5f},
        {  2000,    28.937f,   71.968f,  98.092f,  67.301f,  -35.868f,  181.052f, 0, 3.0f},
        {  3000,    42.413f,   74.418f, 106.515f,  68.136f,  -40.422f,  181.467f, 0, 3.0f},
        {  4000,    44.194f,   79.949f, 112.012f,  74.107f,  -33.880f,  180.943f, 0, 3.0f},
        {  5000,    54.823f,   91.079f, 125.910f,  86.362f,  -44.487f,  180.899f, 0, 3.0f},
        {  6000,    54.822f,   59.004f,  52.225f,  12.154f,  -0.960f,   180.140f, 0, 4.5f},  // 
};
const int Traj_GetLen_Ore6 = sizeof(Traj_Get_Ore6) / sizeof(Traj_Get_Ore6[0]);



/*-----------------------------单存一个矿石---------------------------------------------*/
  const float_t Traj_Store_L[][FC_COUNT] = {  
        //    time    yaw       p1        p2       roll      endP       endR    grip  speed
        //{     0,    -4.154f,   28.010f,  30.057f,  11.160f,     7.847f,   180.025f, 0, 4.5f},  // 起始位
        {  1000,   -62.822f,   59.004f,  52.225f,  12.154f,    -0.960f,   180.140f, 0, 4.5f},  // 
        {  2000,   -62.026f,   54.005f,  29.885f,  14.186f,     5.133f,   180.146f, 0, 3.0f},  // 
        {  4000,   -62.209f,   49.077f,  16.984f,  13.868f,    10.065f,   180.014f, 0, 3.0f},  //
        {  3000,   -62.847f,   50.032f,  12.026f,  15.840f,    10.319f,   180.832f, 0, 3.0f},  // 
        {  4000,   -62.209f,   49.077f,  16.984f,  13.868f,    10.065f,   180.014f, 1, 3.0f},  // 
        {  5000,   -62.695f,   10.998f,  16.992f,  11.132f,   -27.936f,   180.030f, 1, 3.5f},  // 收臂
        {  6000,    -1.852f,   13.980f,  29.893f,  11.390f,    62.939f,   180.019f, 1, 4.5f},  // 终止位
    };
    const int Traj_StoreLen_L = sizeof(Traj_Store_L) / sizeof(Traj_Store_L[0]);
    const TrajClip StoresingleClip_L = {Traj_Store_L, sizeof(Traj_Store_L) / sizeof(Traj_Store_L[0])};

    /*-----------------------------------------------------------------------------------------------------------------------*/


    // 存右矿石：右手 YAW 使用原实测值，其余关节复用左手存矿轨迹特征。
    const float_t Traj_Store_R[][FC_COUNT] = {
        //    time    yaw       p1        p2       roll      endP       endR    grip  speed
        //{     0,    4.154f,   28.010f,  30.057f,  11.160f,     7.847f,   180.025f, 0, 4.5f},  // 起始位
        {  1000,   54.822f,   59.004f,  52.225f,  12.154f,    -0.960f,   180.140f, 0, 4.5f},  // 
        {  2000,   54.026f,   54.005f,  29.885f,  14.186f,     5.133f,   180.146f, 0, 3.0f},  // 
        {  4000,   54.209f,   49.077f,  16.984f,  13.868f,    10.065f,   180.014f, 0, 3.0f},  //
        {  3000,   54.847f,   50.032f,  12.026f,  15.840f,    10.319f,   180.832f, 0, 3.0f},  // 
        {  4000,   54.209f,   49.077f,  13.984f,  13.868f,    10.065f,   180.014f, 1, 3.0f},  // 
        {  5000,   54.695f,   10.998f,  16.992f,  11.132f,   -27.936f,   180.030f, 1, 3.5f},  // 收臂
        {  6000,    -1.852f,   13.980f,  29.893f,  11.390f,    62.939f,  180.019f, 1, 4.5f},  // 终止位
    };
    const int Traj_StoreLen_R = sizeof(Traj_Store_R) / sizeof(Traj_Store_R[0]);
    const TrajClip StoresingleClip_R = {Traj_Store_R, sizeof(Traj_Store_R) / sizeof(Traj_Store_R[0])};
     /*-----------------------------------取矿石--------------------------------------*/
    // 取左矿轨迹帧 (从CSV数据提取), 夹爪 0夹紧 1松开
    
    const float_t Traj_Exchange_L[][FC_COUNT] = {  
        //    time    yaw       p1        p2       roll      endP       endR    grip  speed
        {     0,    -4.154f,   28.010f,  30.057f,  11.160f,     7.847f,   180.025f, 1, 3.5f},  // 起始位
        {  1000,   -26.194f,   9.984f,  18.028f,  7.132f,   -27.838f,     180.025f, 1, 3.5f},  // 中途过渡
        {  2000,   -62.194f,   10.984f,  15.028f,  11.132f,   -29.838f,   180.025f, 1, 3.5f},  // 中途过渡
        {  3000,   -62.978f,   50.989f,  18.032f,  10.116f,    13.806f,   180.019f, 0, 2.5f},  // 夹爪闭合起
        {  3000,   -62.978f,   50.989f,  19.032f,  10.116f,    13.806f,   180.019f, 0, 2.5f},  // 夹爪闭合起
        {  4000,   -62.041f,   51.043f,  22.942f,  19.693f,      5.715f,  180.025f, 0, 2.5f},
        {  5000,   -62.041f,   52.043f,  29.942f,  11.693f,      14.715f, 180.025f, 0, 2.5f},
        {  6000,   -62.041f,   52.043f,  32.942f,  15.693f,      8.715f,  180.025f, 0, 2.5f},
        {  7000,   -62.041f,   56.043f,  43.942f,  15.693f,      -7.715f,   180.025f, 0, 3.5f},
        {  8000,    -1.852f,   13.980f,  29.893f,  11.390f,    62.939f,   180.019f, 0, 4.f},  // 终止位
    };
    const TrajClip ExchangesingleClip_L = {Traj_Exchange_L, sizeof(Traj_Exchange_L) / sizeof(Traj_Exchange_L[0])};

    // 取右矿石：右手 YAW 使用原实测值，其余关节复用左手取矿轨迹。
    const float_t Traj_Exchange_R[][FC_COUNT] = {
        //    time    yaw       p1        p2       roll      endP       endR    grip  speed
        {     0,    4.154f,   28.010f,  30.057f,  11.160f,     7.847f,   180.025f, 1, 3.5f},  // 起始位
        {  1000,   26.194f,   9.984f,  18.028f,  7.132f,   -27.838f,   180.025f, 1, 3.5f},  // 中途过渡
        {  2000,   54.194f,   10.984f,  15.028f,  11.132f,   -29.838f,   180.025f, 1, 3.5f},  // 中途过渡
        {  3000,   54.978f,   51.989f,  18.032f,  10.116f,    13.806f,   180.019f, 0, 2.5f},  // 夹爪闭合起
        {  4000,   54.041f,   50.043f,  22.942f,  19.693f,      5.715f,   180.025f, 0, 2.5f},
        {  5000,   54.041f,   50.043f,  29.942f,  11.693f,      14.715f,   180.025f, 0, 2.5f},
        {  6000,   54.041f,   50.043f,  32.942f,  15.693f,      8.715f,   180.025f, 0, 2.5f},
        {  7000,   54.041f,   56.043f,  43.942f,  15.693f,      -7.715f,   180.025f, 0, 3.5f},
        {  8000,    -1.852f,   13.980f,  29.893f,  11.390f,    62.939f,   180.019f, 0, 4.f},  // 终止位
    };
    const TrajClip ExchangesingleClip_R = {Traj_Exchange_R, sizeof(Traj_Exchange_R) / sizeof(Traj_Exchange_R[0])};

    // 一键连续存贮矿
const SOreStep OreStepConfig[] = {
    //               取矿                                存矿                       rollOff                       
    {{Traj_Get_Ore2, Traj_GetLen_Ore2}, {Traj_Store_L, Traj_StoreLen_L}, STORE_ROLL_DOWN_OFFSET},  // step 0: Ore2  左
    {{Traj_Get_Ore6, Traj_GetLen_Ore6}, {Traj_Store_R, Traj_StoreLen_R}, STORE_ROLL_DOWN_OFFSET},  // step 1: Ore6  右
    {{Traj_Get_Ore4, Traj_GetLen_Ore4}, {Traj_Store_R, Traj_StoreLen_R}, 180.0f},  // step 2: Ore4  数据为未翻转基准，需 +180 翻转一次
    {{Traj_Get_Ore3, Traj_GetLen_Ore3}, {Traj_Store_L, Traj_StoreLen_L}, STORE_ROLL_DOWN_OFFSET},  // step 3: Ore3  左
    {{Traj_Get_Ore5, Traj_GetLen_Ore5}, {Traj_Store_R, Traj_StoreLen_R}, STORE_ROLL_DOWN_OFFSET},  // step 4: Ore5  右
    {{Traj_Get_Ore1, Traj_GetLen_Ore1}, {Traj_Store_L, Traj_StoreLen_L}, STORE_ROLL_DOWN_OFFSET},  // step 5: Ore1
};
const int OreStepCount = sizeof(OreStepConfig) / sizeof(OreStepConfig[0]);
    
  
} // namespace my_engineer
