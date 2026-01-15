/**
 * @file Configuration.hpp
 * @author Zoe, Ciallo
 * @brief My Engineer Robot 应用程序的总配置头文件。
 * @version 1.1
 * @date 2024-10-30
 * @LastEditors Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-15
 *
 * @details
 * 在此文件中定义应用于各个配置的枚举类型。
 */


#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include "conf_common.hpp"
#include "conf_process.hpp"
#include "conf_interface.hpp"
#include "conf_module.hpp"
#include "conf_device.hpp"

namespace my_engineer {

/**
 * @brief 接口ID枚举类型
 * 
 */
enum class EInterfaceID{
    INF_NULL = -1,  ///< 空接口
    INF_TEST = 0,   ///< 测试接口
    INF_DBUS,       ///< DBUS接口(遥控器，使用USART5)
    INF_UART1,      ///< 串口1(用于裁判系统)
    INF_UART7,      ///< 串口7(用于视觉)
    INF_UART10,     ///< 串口10(用于调试)
    INF_CAN1,       ///< CAN1(用于底盘电机)
    INF_CAN2,       ///< CAN2(用于龙门架电机)
    INF_CAN3,       ///< CAN3(用于机械臂电机)
    INF_SPI2,       ///< SPI2(用于BMI-088 MEMS传感器)
    INF_SPI6,       ///< SPI6(用于WS2312 LED)
    INF_ADC1,       ///< ADC1(用于摇杆)
    INF_USB_CDC, ///< USB CDC(用于调试)
};


/**
 * @brief 设备ID枚举类型
 * 
 */
enum class EDeviceID{
    DEV_NULL = -1,          ///< 空设备
    DEV_TEST = 0,           ///< 测试设备
    DEV_RC_DR16,            ///< 遥控接收器（DR16）
    DEV_RM_REFEREE,         ///< RoboMaster裁判系统设备
    DEV_RP_VISION,          ///< RoboPilots视觉系统设备
    DEV_BOARD_LINK,         ///< 板间通信设备
    DEV_CONTROLLER_LINK,    ///< 控制器通信设备
    DEV_ROCKER,             ///< 摇杆设备
    DEV_BUZZER,             ///< 蜂鸣器设备
    DEV_LASER_L,            ///< 激光雷达左
    DEV_LASER_R,            ///< 激光雷达右
    DEV_MEMS_BMI088,        ///< 6轴MEMS（BMI-088）
    DEV_MULTI_BUTTON,       ///< 多按键设备
    /*----------- 左臂电机 -----------*/
    DEV_MTR_YAW_L,          ///< 左臂Yaw电机（M6020, CAN1 ID5）
    DEV_MTR_PITCH1_L,       ///< 左臂Pitch1电机（DM4310, CAN2 0x30）
    DEV_MTR_PITCH2_L,       ///< 左臂Pitch2电机（DM4310, CAN2 0x32）
    DEV_MTR_ROLL_L,         ///< 左臂Roll电机（DM3510, CAN2 0x34）
    DEV_MTR_PITCH_END_L,    ///< 左臂末端Pitch电机（DM3510, CAN2 0x36）
    /*----------- 右臂电机 -----------*/
    DEV_MTR_YAW_R,          ///< 右臂Yaw电机（M6020, CAN1 ID6）
    DEV_MTR_PITCH1_R,       ///< 右臂Pitch1电机（DM4310, CAN3 0x30）
    DEV_MTR_PITCH2_R,       ///< 右臂Pitch2电机（DM4310, CAN3 0x32）
    DEV_MTR_ROLL_R,         ///< 右臂Roll电机（DM3510, CAN3 0x34）
    DEV_MTR_PITCH_END_R,    ///< 右臂末端Pitch电机（DM3510, CAN3 0x36）
};

/**
 * @brief 模块ID枚举类型
 * 
 */
enum class EModuleID{
    MOD_NULL = -1,          ///< 空模块
    MOD_TEST = 0,           ///< 测试模块
    MOD_CHASSIS,            ///< 底盘模块
    MOD_MANTIS,             ///< 螳螂臂模块
    MOD_GIMBAL,             ///< 云台模块
    MOD_GANTRY,             ///< 龙门模块
    MOD_SUBGANTRY,          ///< 子龙门模块
    MOD_CONTROLLER,         ///< 自定义控制器模块（单臂兼容）
    MOD_CONTROLLER_LEFT,    ///< 左臂控制器模块
    MOD_CONTROLLER_RIGHT,   ///< 右臂控制器模块
    MOD_BUTTON,             ///< 按键模块
};

/**
 * @brief 系统ID枚举类型
 * 
 */
enum class ESystemID{
    SYS_NULL = -1,          ///< 空系统
    SYS_TEST = 0,           ///< 测试系统
    SYS_REMOTE,             ///< 遥控系统
    SYS_REFEREE,            ///< RoboMaster裁判系统
    SYS_VISION,             ///< RoboPilots视觉系统
    SYS_BOARDLINK,          ///< 板间通信系统
    SYS_CONTROLLERLINK,     ///< 控制器通信系统
};

/**
 * @brief Application的入口函数
 * @return None
 */
void ApplicationEntryPoint();

} // namespace my_engineer

#endif // CONFIGURATION_HPP
