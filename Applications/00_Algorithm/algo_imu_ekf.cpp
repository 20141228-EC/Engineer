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

EAppStatus CAlgo_IMU_EKF::InitAlgo_(SFilterInitParam_Base &param)
{
    if (param.AlgoID == EAlgoID::ALGO_NULL)
    {
        return APP_ERROR;
    }

    auto &imu_ekf_param =
        static_cast<SAlgoImuEkfInitParam &>(param);

    AlgoID = imu_ekf_param.AlgoID;
    DT = imu_ekf_param.DT;
    use_transform = imu_ekf_param.use_transform;

    mems = MemsIDMap.at(imu_ekf_param.memsDevID);

    if (mems == nullptr)
    {
        return APP_ERROR;
    }

    mems->StartDevice();

    float init_q[4] =
    {
        1.0f,
        0.0f,
        0.0f,
        0.0f
    };

    IMU_QuaternionEKF_Init(
        init_q,
        imu_ekf_param.process_noise_q,
        imu_ekf_param.process_noise_b,
        imu_ekf_param.measure_noise,
        imu_ekf_param.lambda
    );

    if (use_transform)
    {
        transform_init(&EKFgim_trans);
    }

    Imu_Ekf_Info.is_initialized = true;

    RegisterAlgorithm_();

    return APP_OK;
}

EAppStatus CAlgo_IMU_EKF::UpdateHandler_()
{
    if (mems == nullptr)
    {
        return APP_ERROR;
    }

    float ax_raw = mems->memsData[CMemsBase::DATA_ACC_X];
    float ay_raw = mems->memsData[CMemsBase::DATA_ACC_Y];
    float az_raw = mems->memsData[CMemsBase::DATA_ACC_Z];

    float gx_raw = mems->memsData[CMemsBase::DATA_GYRO_X];
    float gy_raw = mems->memsData[CMemsBase::DATA_GYRO_Y];
    float gz_raw = mems->memsData[CMemsBase::DATA_GYRO_Z];

    float gx = gx_raw;
    float gy = gy_raw;
    float gz = gz_raw;

    float ax = ax_raw;
    float ay = ay_raw;
    float az = az_raw;

    Imu_Ekf_Info.gyro_x = gx;
    Imu_Ekf_Info.gyro_y = gy;
    Imu_Ekf_Info.gyro_z = gz;

    // 如果使用坐标系转换
    if (use_transform)
    {
        float tgx, tgy, tgz;
        float tax, tay, taz;

        Vector_Transform(
            gx, gy, gz,
            ax, ay, az,
            &tgx, &tgy, &tgz,
            &tax, &tay, &taz
        );

        gx = tgx;
        gy = tgy;
        gz = tgz;

        ax = tax;
        ay = tay;
        az = taz;
    }

    IMU_QuaternionEKF_Update(
        gx,
        gy,
        gz,
        ax,
        ay,
        az,
        DT
    );

    Imu_Ekf_Info.q[0] = QEKF_INS.q[0];
    Imu_Ekf_Info.q[1] = QEKF_INS.q[1];
    Imu_Ekf_Info.q[2] = QEKF_INS.q[2];
    Imu_Ekf_Info.q[3] = QEKF_INS.q[3];

    Imu_Ekf_Info.gyro_bias[0] = QEKF_INS.GyroBias[0];
    Imu_Ekf_Info.gyro_bias[1] = QEKF_INS.GyroBias[1];
    Imu_Ekf_Info.gyro_bias[2] = QEKF_INS.GyroBias[2];

    // 获取欧拉角
    Imu_Ekf_Info.roll = QEKF_INS.Roll;
    Imu_Ekf_Info.pitch = QEKF_INS.Pitch;
    Imu_Ekf_Info.yaw = QEKF_INS.Yaw;
    Imu_Ekf_Info.yaw_total = QEKF_INS.YawTotalAngle;

    // 获取世界系下的平动加速度
    BMI_Get_Acceleration(
        Imu_Ekf_Info.pitch,
        Imu_Ekf_Info.roll,
        Imu_Ekf_Info.yaw,
        ax,
        ay,
        az,
        &Imu_Ekf_Info.accel_x,
        &Imu_Ekf_Info.accel_y,
        &Imu_Ekf_Info.accel_z
    );

    return APP_OK;
}

} // namespace my_engineer