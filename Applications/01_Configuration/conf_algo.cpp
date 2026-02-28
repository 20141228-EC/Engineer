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

    /****************初始化卡尔曼滤波****************/
    CAlgo_Kf::SAlgoKfInitParam kf_initparam;
    kf_initparam.AlgoID = EAlgoID::ALGO_KF;
    kf_initparam.DT = 0.001f;
    kf_initparam.measurement_degree = {1};
    kf_initparam.measurement_map = {1};
    kf_initparam.memsDevID = EDeviceID::DEV_MEMS_BMI088;
    kf_initparam.r_diagonal_elements = {0.01f};
    kf_initparam.state_min_variance = {0.0001f};
    kf_initparam.u_size = 0;
    kf_initparam.use_auto_adjustment = false;
    kf_initparam.x_size = 1;
    kf_initparam.z_size = 1;
    // test param...

    // 使用初始化后的参数创建KfFilter实例
    static auto KfFilter = CAlgo_Kf(kf_initparam);

    return APP_OK;
}

} // namespace my_engineer
