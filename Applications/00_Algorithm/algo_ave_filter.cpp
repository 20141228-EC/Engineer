/**
 * @file algo_ave_filter.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 互补滤波
 * @version 1.0
 * @date 2026-01-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "Algorithm.hpp"
#include "mems/mems_bmi088.hpp"
#include <math.h>

#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)

namespace my_engineer
{

CMemsBase *pmems_ave_test = nullptr;

/**
 * @brief 初始化互补滤波算法
 * @retval EAppStatus
 */
EAppStatus CAlgo_IMU_Ave::InitAlgo_(SFilterInitParam_Base &param){

    // 检查param是否正确
    if (param.AlgoID == EAlgoID::ALGO_NULL) return APP_ERROR;

    // 类型转换
    auto &imu_ave_param = static_cast<SAlgoImuAveInitParam &>(param);

    // 传感器指针与参数配置
    mems = MemsIDMap.at(imu_ave_param.memsDevID);
    ALPHA = imu_ave_param.ALPHA;
    DT = imu_ave_param.DT;

    // 调试用
    pmems_ave_test = mems;

    // 启动设备
    mems->StartDevice();

    // 注册算法
    RegisterAlgorithm_();

    return APP_OK;
}

/**
 * @brief 更新互补滤波数据
 * @retval EAppStatus
 */
EAppStatus CAlgo_IMU_Ave::UpdateHandler_()
    {
        // 检查设备指针和状态
        if (mems == nullptr)
        {
            return APP_ERROR;
        }

        // 读取原始数据
        float_t ax_raw = mems->memsData[CMemsBase::DATA_ACC_X];
        float_t ay_raw = mems->memsData[CMemsBase::DATA_ACC_Y];
        float_t az_raw = mems->memsData[CMemsBase::DATA_ACC_Z];
        float_t gx_raw = mems->memsData[CMemsBase::DATA_GYRO_X];
        float_t gy_raw = mems->memsData[CMemsBase::DATA_GYRO_Y];
        float_t gz_raw = mems->memsData[CMemsBase::DATA_GYRO_Z];

        // 如果未初始化，先用加速度计计算初始姿态
        if (!Imu_Ave_Info.is_initialized)
        {
            // 确保az不为0以避免atan2分母为0的问题，且加速度测量值有效
            if (az_raw != 0.0f || ax_raw != 0.0f || ay_raw != 0.0f)
            {
                Imu_Ave_Info.imu_ave_pitch = rad2deg(atan2f(ax_raw, sqrtf(ay_raw * ay_raw + az_raw * az_raw)));
                Imu_Ave_Info.imu_ave_roll = rad2deg(atan2f(-ay_raw, az_raw) );
                Imu_Ave_Info.imu_ave_yaw = 0.0f;    ///< yaw初始化为0
                Imu_Ave_Info.is_initialized = true;
            }
            return APP_OK; // 第一次仅计算初始值，下一次再开始滤波
        }
        // 初始化三轴姿态

        // 陀螺仪积分更新姿态（短期预测）
        Imu_Ave_Info.imu_ave_roll += rad2deg(gx_raw * DT);
        Imu_Ave_Info.imu_ave_pitch += rad2deg(gy_raw * DT);
        Imu_Ave_Info.imu_ave_yaw += rad2deg(gz_raw * DT);

        // 将yaw轴角度限制在-180到180之间
        if (Imu_Ave_Info.imu_ave_yaw > 180.0f)
        {
            Imu_Ave_Info.imu_ave_yaw -= 360.0f;
        }
        else if (Imu_Ave_Info.imu_ave_yaw < -180.0f)
        {
            Imu_Ave_Info.imu_ave_yaw += 360.0f;
        }

        // 加速度计计算姿态（长期参考，消除漂移）
        float pitch_acc = rad2deg(atan2f(ax_raw, sqrtf(ay_raw * ay_raw + az_raw * az_raw)));
        float roll_acc = rad2deg(atan2f(-ay_raw, az_raw));

        // 互补融合
        Imu_Ave_Info.imu_ave_roll = ALPHA * Imu_Ave_Info.imu_ave_roll + (1.0f - ALPHA) * roll_acc;
        Imu_Ave_Info.imu_ave_pitch = ALPHA * Imu_Ave_Info.imu_ave_pitch + (1.0f - ALPHA) * pitch_acc;
        Imu_Ave_Info.imu_ave_yaw -= 0.01f * 0.001f;   // 用补偿的方式解决零漂 大概44s偏一度(-) 

        return APP_OK;
    }

} // namespace my_engineer