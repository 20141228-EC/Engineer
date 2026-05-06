/******************************************************************************
 * @brief   板间通信设备类
 *
 * @file    dev_board_link.hpp
 * @author  sllllr (2997708711@qq.com)
 * @version V1.0
 * @date    2026-04-11
 *
 *
 * @copyright Copyright (c) 2026
 *
 ******************************************************************************/

#ifndef DEV_BOARD_LINK_HPP
#define DEV_BOARD_LINK_HPP

#include "dev_common.hpp"
#include "inf_can.hpp"

namespace my_engineer {

/**
 * @brief 板间通信设备类
 */
class CDevBoardLink final : public CDevBase {
public:

    /**
     * @brief 初始化参数结构体
     */
    struct SDevInitParam_BoardLink : public SDevInitParam_Base {
        EInterfaceID interfaceID = EInterfaceID::INF_NULL;  ///< CAN接口ID
        uint32_t offlineTimeout = 100;                      ///< 离线超时时间(ms)
    };

    /**
     * @brief 数据包ID枚举
     * @note  pack_id位于每个数据包的第一个字节+
     * 
     */
    enum EPacketID : uint8_t {
        PKT_CTRL_INFOS = 0,  ///< 控制信息包
        PKT_JOINT_INFOS = 1,    ///< 云台朝向、臂关节角度
        PKT_OTHER_INFOS = 2,    ///< 其他数据
        PKT_COUNT,           ///< 包类型数量
        PKT_FEEDBACK   = 0xFE,  ///< 反馈包（副板发送给主板）
    };

    /**
     * @brief 控制数据包
     * @note 包含关控 yaw角度等控制底盘的所有信息
     * 
     */
    struct SCtrlInfo {

        uint8_t pack_id;    // ID：1

        uint8_t remote_is_online;   // 遥控器是否在线
        int8_t speed_x;    // 底盘x轴速度
        int8_t speed_y;    // 底盘y轴速度
        float_t speed_w;    // 底盘旋转速度


    }__packed ctrlInfo_ = {};

    /**
     * @brief 臂数据包
     * @note 包含臂关节角度 夹爪信息
     * 
     */
    struct SAngleInfo {

        uint8_t pack_id;    // ID：2

        uint8_t grip_close;   // 夹爪是否收紧

        int16_t pitch1;     ///< pitch1角度值
        int16_t pitch2;     ///< pitch2角度值
        int16_t pitch3;     ///< pitch3角度值
    }__packed angleInfo_ = {};

    /**
     * @brief 其他数据，包括是否开小陀螺等
     * 
     * 
     */
     struct SOtherInfo {

        uint8_t pack_id;    // ID：3
        int16_t yaw_gyro;   ///< 陀螺仪yaw值
        uint8_t is_spin_on;     ///< 是否开小陀螺

        uint8_t autoTask;   /// 自动任务编号

        uint8_t reserved[4];    /// 保留
     }__packed otherInfo_ = {};

    /**
     * @brief 反馈包 - 副板发送给主板
     * @note  8字节，包含接收状态和通信状态
     */
    struct SFeedbackPack {
        uint8_t  pack_id;           ///< 包ID = 0xFE（反馈包标识）
        uint8_t  rx_status;         ///< 各包接收状态（bit0~2对应包0~2，1=已接收）
        uint8_t  link_status;       ///< 通信状态（0=RESET 1=OFFLINE 2=ONLINE）
        uint8_t  reserved[5];       ///< 预留
    } __packed;

    /**
     * @brief 板间通信状态枚举
     */
    enum class EBoardLinkStatus {
        RESET,      ///< 复位状态
        OFFLINE,    ///< 离线状态
        ONLINE,     ///< 在线状态
    } linkStatus = EBoardLinkStatus::RESET;

    /**
     * @brief 构造函数
     */
    CDevBoardLink() { deviceType = EDevType::DEV_BOARD_LINK; }

    /**
     * @brief 初始化设备
     * @param pStructInitParam 初始化参数指针
     * @return EAppStatus 初始化状态
     */
    EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override;

    /**
	 * @brief 修改CAN发送数据
	 */
	EAppStatus Modify_CanTxData(uint8_t* data) 
	{
		if (data == nullptr) return APP_ERROR;
		std::copy(data, data + txNode_.dataBuffer.size(), txNode_.dataBuffer.begin());
		return APP_OK;
	}

    EAppStatus SendPackage(EPacketID pack_id);

private:

    CInfCAN::CCanRxNode rxNode_;             ///< CAN接收节点
    CInfCAN::CCanTxNode txNode_;  ///< CAN发送节点指针

    /**
     * @brief 超时检测相关参数
     */
    struct STimeoutParam {
        uint32_t offlineTimeout = 100;   ///< 离线超时时间(ms)
        uint32_t rxTimestamp = 0;        ///< 最后接收时间戳
        uint32_t lastParseTime = 0;      ///< 最后解析时间（用于判断是否有新数据）
    } timeoutParam_;

    /**
     * @brief 更新处理（在UpdateTask中调用）
     * @note  检查是否有新数据，有则解析
     */
    void UpdateHandler_() override;

    /**
     * @brief 心跳处理（在HeartbeatTask中调用）
     * @note  检查离线状态
     */
    void HeartbeatHandler_() override;

    /**
     * @brief 解析接收到的数据包
     * @return EAppStatus 解析状态
     */
    EAppStatus ParseRxPacket_();
};

} // namespace my_engineer

#endif // DEV_BOARD_LINK_HPP