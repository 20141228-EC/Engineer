/********************************************************************************
 * @brief        IMU EKF姿态解算算法实现
 * @file         algo_imu_ekf.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-31
 * @copyright    Copyright (c) 2025
 ********************************************************************************/
#include "algo_imu_ekf.hpp"
#include "mems/mems_bmi088.hpp"
#include "conf_device.hpp"
#include "bmi_EKF.h"
#include <math.h>

namespace my_engineer
{
    // 全局姿态角定义
    float g_ekf_roll = 0.0f;
    float g_ekf_pitch = 0.0f;
    float g_ekf_yaw = 0.0f;
    float g_ekf_yaw_total = 0.0f;

    // --- 内部变量和参数 ---
    static CMemsBmi088 *s_bmi088 = nullptr; // IMU设备指针

    // EKF初始化参数
    static constexpr float EKF_Q1 = 10.0f;
    static constexpr float EKF_Q2 = 0.001f;
    static constexpr float EKF_R = 1000000.0f;
    static constexpr float EKF_LAMBDA = 0.9996f;
    static constexpr float DT = 0.001f; // 采样周期 (秒)
    static float init_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    /**
     * @brief 初始化IMU EKF
     */
    EAppStatus InitImuEkf()
    {
        // 1. 获取传感器指针
        auto it = MemsIDMap.find(EDeviceID::DEV_MEMS_BMI088);
        if (it != MemsIDMap.end())
        {
            s_bmi088 = static_cast<CMemsBmi088 *>(it->second);
        }

        if (s_bmi088 == nullptr)
        {
            return APP_ERROR; // 如果找不到设备，返回错误
        }

        // 2. 初始化坐标变换
        // 注意：EKFgim_trans 结构体应在 bmi_EKF.h 中定义
        transform_init(&EKFgim_trans);

        // 3. 初始化EKF核心
        // 实际的初始化推迟到Update函数中，等待传感器数据就绪
        // IMU_QuaternionEKF_Init(init_quaternion, EKF_Q1, EKF_Q2, EKF_R, EKF_LAMBDA);
        
        return APP_OK;
    }

    /**
     * @brief 更新IMU EKF
     */
    EAppStatus UpdateImuEkf()
    {
        // 检查设备指针和状态
        if (s_bmi088 == nullptr)
        {
            return APP_ERROR;
        }

        // 如果EKF未初始化，尝试进行初始化
        if (!QEKF_INS.Initialized)
        {
            IMU_QuaternionEKF_Init(init_quaternion, EKF_Q1, EKF_Q2, EKF_R, EKF_LAMBDA);
            // 如果初始化失败，直接返回，下一轮再试
            if (!QEKF_INS.Initialized)
            {
                return APP_OK;
            }
        }

        // --- 核心EKF更新逻辑 ---

        // 1. 读取BMI088原始数据
        float gx = s_bmi088->memsData[CMemsBase::DATA_GYRO_X];
        float gy = s_bmi088->memsData[CMemsBase::DATA_GYRO_Y];
        float gz = s_bmi088->memsData[CMemsBase::DATA_GYRO_Z];
        float ax = s_bmi088->memsData[CMemsBase::DATA_ACC_X];
        float ay = s_bmi088->memsData[CMemsBase::DATA_ACC_Y];
        float az = s_bmi088->memsData[CMemsBase::DATA_ACC_Z];

        // 2. (可选) 坐标变换
        float ggx, ggy, ggz, aax, aay, aaz;
        Vector_Transform(gx, gy, gz, ax, ay, az, &ggx, &ggy, &ggz, &aax, &aay, &aaz);

        // 3. 调用EKF更新
        IMU_QuaternionEKF_Update(ggx, ggy, ggz, aax, aay, aaz, DT);

        // 4. 读取EKF输出结果到全局变量
        g_ekf_roll = QEKF_INS.Roll;
        g_ekf_pitch = QEKF_INS.Pitch;
        g_ekf_yaw = QEKF_INS.Yaw;
        g_ekf_yaw_total = QEKF_INS.YawTotalAngle;

        return APP_OK;
    }

} // namespace my_engineer