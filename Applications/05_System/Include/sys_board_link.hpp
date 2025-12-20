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
 * @copyright Copyright (c) 2025
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
     * @note  由主板根据拨杆状态计算后传递
     */
    struct SCtrlFlags {
        // 控制模式
        bool chassis_ctrl = false;      ///< 底盘控制使能
        bool gimbal_ctrl = false;       ///< 云台控制使能
        bool arm_front_ctrl = false;    ///< 机械臂前四轴控制
        bool arm_rear_ctrl = false;     ///< 机械臂后四轴控制

        // 使能标志
        bool arm_enable = false;        ///< 机械臂使能
        bool gimbal_enable = false;     ///< 云台使能
        bool chassis_enable = false;    ///< 底盘使能

        // 状态标志
        bool rc_online = false;         ///< 遥控器在线
        bool is_rc_ctrl = false;        ///< 遥控器控制模式
        bool is_key_ctrl = false;       ///< 键盘控制模式
        bool climb_stair = false;       ///< 上台阶标志
    };

    /**
     * @brief 板间通信的遥控器信息
     * @note  格式与 SysRemote.remoteInfo.remote 一致，方便直接替换使用
     */
    CSystemRemote::SRemoteInfo remoteInfo;

    /**
     * @brief 控制标志信息
     */
    SCtrlFlags ctrlFlags;

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

    /**
     * @brief 更新遥控器数据
     * @note  将设备层的原始摇杆值转换为系统层格式
     */
    EAppStatus UpdateRemoteData_();

    /**
     * @brief 更新控制标志
     * @note  解析设备层的控制标志包
     */
    EAppStatus UpdateCtrlFlags_();

};

extern CSystemBoardLink SysBoardLink;

}   // namespace my_engineer

#endif // SYS_BOARD_LINK_HPP
