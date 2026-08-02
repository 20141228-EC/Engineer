/**
 * @file conf_algo.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 算法配置
 * @version 1.0
 * @date 2026-01-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "conf_algo.hpp"
#include "Algorithm.hpp"

namespace my_engineer {

/**
 * @brief 配置并初始化所有算法
 * @return APP_OK - 初始化成功
 * @return APP_ERROR - 初始化失败
 */
EAppStatus InitAllAlgo(){

    /***************初始化互补滤波****************/
    CAlgo_IMU_Ave::SAlgoImuAveInitParam imu_ave_initparam;
    imu_ave_initparam.AlgoID = EAlgoID::ALGO_IMU_AVE;
    imu_ave_initparam.ALPHA = 0.98f;
    imu_ave_initparam.DT = 0.001f;
    imu_ave_initparam.memsDevID = EDeviceID::DEV_MEMS_BMI088;

    // 使用初始化后的参数创建ImuAveFilter实例
    static auto ImuAveFilter = CAlgo_IMU_Ave(imu_ave_initparam);

    /**************** 初始化 IMU EKF 姿态滤波 ****************/
    CAlgo_IMU_EKF::SAlgoImuEkfInitParam imu_ekf_initparam;

    imu_ekf_initparam.AlgoID = EAlgoID::ALGO_IMU_EKF;
    imu_ekf_initparam.memsDevID = EDeviceID::DEV_MEMS_BMI088;
    imu_ekf_initparam.DT = 0.001f;

    imu_ekf_initparam.process_noise_q = 10.0f;
    imu_ekf_initparam.process_noise_b = 0.001f;
    imu_ekf_initparam.measure_noise = 1000000.0f;
    imu_ekf_initparam.lambda = 0.9996f;

    imu_ekf_initparam.use_transform = false;

    // 创建ImuEkfFilter实例
    static auto ImuEkfFilter = CAlgo_IMU_EKF(imu_ekf_initparam);

    return APP_OK;
}

} // namespace my_engineer
