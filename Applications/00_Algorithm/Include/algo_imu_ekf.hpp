/********************************************************************************
 * @brief        IMU EKF姿态解算算法
 * @file         algo_imu_ekf.hpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-31
 * @copyright    Copyright (c) 2025
 ********************************************************************************/

#ifndef ALGO_IMU_EKF_HPP
#define ALGO_IMU_EKF_HPP

#include "algo_filter_common.hpp"

namespace my_engineer
{
    // 全局可访问的EKF姿态角 (单位: 度)
    extern float g_ekf_roll;
    extern float g_ekf_pitch;
    extern float g_ekf_yaw;
    extern float g_ekf_yaw_total; // 累计Yaw角度

    /**
     * @brief 初始化IMU EKF姿态解算器
     *        应在系统主循环开始前调用一次
     * @return EAppStatus 
     */
    EAppStatus InitImuEkf();

    /**
     * @brief 更新IMU EKF姿态解算器
     *        应在系统主循环中周期性调用 (例如，每1ms)
     * @return EAppStatus 
     */
    EAppStatus UpdateImuEkf();

} // namespace my_engineer

#endif // ALGO_IMU_EKF_HPP