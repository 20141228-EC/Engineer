/**
 * @file sys_board_link.hpp
 * @author Ciallo～(∠·ω< )⌒☆(1002046597@qq.com)
 * @brief 板间通信系统层
 * @version 1.0
 * @date 2025-12-11
 *
 * @details 封装板间通信设备，提供统一的遥控器数据接口
 *          当板间通信在线时，其他系统可通过本系统获取主板传来的遥控器数据
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef SYS_BOARD_LINK_HPP
#define SYS_BOARD_LINK_HPP

#include "sys_common.hpp"
#include "sys_remote.hpp"
#include "dev_board_link.hpp"

namespace my_engineer {

/**
 * @brief 板间通信系统类
 *
 */
class CSystemBoardLink final: public CSystemBase {
public:

    /**
     * @brief 板间通信系统初始化参数结构体
     */
    struct SSystemInitParam_BoardLink: public SSystemInitParam_Base {
        EDeviceID boardLinkDevID = EDeviceID::DEV_NULL;  ///< 板间通信设备ID
    };

    /**
     * @brief 控制标志信息结构体
     * @note 包含关控等控制底盘的所有信息
     */
    struct SCtrlFlags {
        uint8_t remote_is_online;   // 遥控器是否在线

        int16_t speed_x;    // 底盘x轴速度
        int16_t speed_y;    // 底盘y轴速度
        int16_t speed_w;    // 底盘旋转速度
    } __packed ctrlInfos = {};

    /**
     * @brief 云台+臂数据包
     * @note 包含云台朝向 臂关节角度信息
     * 
     */
    struct SAngleInfo {
        uint8_t grip_close;   // 夹爪是否收紧

        int16_t pitch1;     ///< pitch1角度值
        int16_t pitch2;     ///< pitch2角度值
        int16_t pitch3;     ///< pitch3角度值
    }__packed angleInfos = {};

    /**
     * @brief 其他数据，包括是否开小陀螺等
     * 
     */
     struct SOtherInfo {
        int16_t yaw_gyro;   ///< 陀螺仪yaw值
        uint8_t is_spin_on;     ///< 是否开小陀螺
        uint8_t autoTask;   /// 自动任务编号
        uint8_t use_controller; ///< 是否使用控制器
     }__packed otherInfos = {};

    /**
     * @brief 检查板间通信是否在线
     * @return true - 在线
     * @return false - 离线或复位状态
     */
    bool IsOnline() const;

    /**
     * @brief 获取通信状态
     * @return CDevBoardLink::EBoardLinkStatus 通信状态枚举
     */
    CDevBoardLink::EBoardLinkStatus GetLinkStatus() const;

    /**
     * @brief 初始化系统
     * @param pStruct 初始化参数指针
     * @return EAppStatus 初始化状态
     */
    EAppStatus InitSystem(SSystemInitParam_Base *pStruct) final;

protected:

    /**
     * @brief 更新处理
     * @note  从设备层读取数据，转换为系统层格式
     */
    void UpdateHandler_() final;

    /**
     * @brief 心跳处理
     * @note  检查通信状态
     */
    void HeartbeatHandler_() final;

private:

    CDevBoardLink *pBoardLinkDev_ = nullptr;  ///< 板间通信设备指针

    EAppStatus UpdateCtrlInfos_();

};

extern CSystemBoardLink SysBoardLink;

}   // namespace my_engineer

#endif // SYS_BOARD_LINK_HPP
