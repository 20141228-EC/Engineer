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
#include "Algorithm.hpp"

namespace my_engineer
{

/**
 * 
 * @brief 陀螺仪扩展卡尔曼滤波算法类
 * 
 */
class CAlgo_IMU_EKF : public CFilterBase
{
public:
    struct SAlgoImuEkfInitParam : public SFilterInitParam_Base
    {
        EDeviceID memsDevID;
        float DT;

        float process_noise_q;
        float process_noise_b;
        float measure_noise;    // 测量噪声协方差矩阵
        float lambda;

        bool use_transform;     // 是否转换坐标系
    };

    // 卡尔曼滤波信息结构体+实例
    struct SImuEkfInfo
    {
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        float yaw_total = 0.0f;

        float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

        float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

        float accel_x = 0.0f;   // 世界系下x轴加速度
        float accel_y = 0.0f;   // 世界系下y轴加速度
        float accel_z = 0.0f;   // 世界系下z轴加速度

        float gyro_x = 0.0f;    // 陀螺仪测得的原始x轴角速度
        float gyro_y = 0.0f;    // 陀螺仪测得的原始y轴角速度
        float gyro_z = 0.0f;    // 陀螺仪测得的原始z轴角速度

        bool is_initialized = false;
    } Imu_Ekf_Info;

    explicit CAlgo_IMU_EKF(SAlgoImuEkfInitParam &param)
    {
        InitAlgo_(param);
    }

protected:
    EAppStatus InitAlgo_(SFilterInitParam_Base &param) override;
    EAppStatus UpdateHandler_() override;

private:
    CMemsBase *mems = nullptr;

    float DT = 0.001f;
    bool use_transform = false;
};

} // namespace my_engineer

#endif // ALGO_IMU_EKF_HPP