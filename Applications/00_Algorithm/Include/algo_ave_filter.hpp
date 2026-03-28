/**
 * @file algo_ave_filter.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 互补滤波
 * @version 1.0
 * @date 2026-01-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef ALGO_AVE_FILTER_HPP
#define ALGO_AVE_FILTER_HPP

#include "algo_filter_common.hpp"

namespace my_engineer
{

/**
 * @brief 互补滤波算法类
 * 
 */
class CAlgo_IMU_Ave final: public CFilterBase{
public:
    // 继承基类初始化结构体
    struct SAlgoImuAveInitParam : public SFilterInitParam_Base
    {
        EDeviceID memsDevID = EDeviceID::DEV_NULL;  ///< 传感器设备ID
        float_t ALPHA = 0.0f;                         ///< 陀螺仪信任系数
        float_t DT = 0.0f;                            ///< 调度周期
    };

    // 定义互补滤波信息结构体+实例
    struct SAlgoImuAveInfo
    {
        bool is_initialized = false;                ///< 是否完成初始化
        float_t imu_ave_roll = 0.0f;                  ///< roll轴
        float_t imu_ave_pitch = 0.0f;                 ///< pitch轴
        float_t imu_ave_yaw = 0.0f;                   ///< yaw轴
        float_t acc_x_filter = 0.f;                 ///< x轴加速度滤波值
        float_t acc_y_filter = 0.f;                 ///< y轴加速度滤波值
        float_t acc_z_filter = 0.f;                 ///< z轴加速度滤波值
        float_t accel_y = 0.f;                      ///< y轴平动加速度
        float_t accel_x = 0.f;                      ///< x轴平动加速度
        float_t accel_z = 0.f;                      ///< z轴平动加速度
    } Imu_Ave_Info;

    // 传感器实例指针
    CMemsBase *mems = nullptr;

    float_t ALPHA = 0.0f;                           ///< 陀螺仪信任系数
    float_t DT = 0.0f;                              ///< 调度周期
    
    CAlgo_IMU_Ave() = default;  ///< 默认构造函数

    explicit CAlgo_IMU_Ave(SFilterInitParam_Base &param) {InitAlgo_(param);};   ///< 带参的构造函数，用初始化结构体构造

    // 模块析构函数
	~CAlgo_IMU_Ave() final { UnregisterAlgorithm_(); };

    EAppStatus InitAlgo_(SFilterInitParam_Base &param) final;   ///< 初始化

    EAppStatus UpdateHandler_() final;    ///< 更新

};

} // namespace my_engineer

#endif // ALGO_AVE_FILTER_HPP