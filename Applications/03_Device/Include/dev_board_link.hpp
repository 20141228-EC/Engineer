/******************************************************************************
 * @brief   板间通信设备类
 *
 * @file    dev_board_link.hpp
 * @author  Ciallo～(∠·ω< )⌒☆(1002046597@qq.com)
 * @version V1.0
 * @date    2025-12-06
 *
 * @details 用于主副板之间的CAN通信
 *          使用单CAN ID (0x300) + pack_id方式区分数据包
 *
 * @copyright Copyright (c) 2025
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
        CInfCAN::CCanTxNode *txNode = nullptr;              ///< CAN发送节点指针（用于发送反馈）
    };

    /**
     * @brief 数据包ID枚举
     * @note  pack_id位于每个数据包的第一个字节
     */
    enum EPacketID : uint8_t {
        PKT_ARM_JOINT1 = 0,  ///< 机械臂关节1（Yaw, Pitch1, Pitch2）
        PKT_ARM_JOINT2 = 1,  ///< 机械臂关节2（Roll, Grip_Roll, Grip_Pitch）
        PKT_GIMBAL     = 2,  ///< 云台目标
        PKT_CTRL_FLAGS = 3,  ///< 控制标志
        PKT_COUNT,           ///< 包类型数量
        PKT_FEEDBACK   = 0xFE,  ///< 反馈包（副板发送给主板）
    };

    /**
     * @brief 包0 - 机械臂关节1（yaw+大臂+小臂）
     * @note  8字节，包含前3个关节的目标角度
     */
    struct SArmJoint1Target {
        uint8_t  pack_id;           ///< 包ID = 0
        int16_t  arm_yaw_target;    ///< 基座Yaw目标角度（×100）
        int16_t  arm_pitch1_target; ///< 大臂Pitch1目标角度（×100）
        int16_t  arm_pitch2_target; ///< 小臂Pitch2目标角度（×100）
        uint8_t  reserved;          ///< 预留
    } __packed armJoint1Target = {};

    /**
     * @brief 包1 - 机械臂关节2（末端+夹爪）
     * @note  8字节，包含末端Roll和夹爪两轴目标角度
     */
    struct SArmJoint2Target {
        uint8_t  pack_id;           ///< 包ID = 1
        int16_t  arm_roll_target;   ///< 末端Roll目标角度（×100）
        int16_t  grip_roll_target;  ///< 夹爪Roll目标角度（×100）
        int16_t  grip_pitch_target; ///< 夹爪Pitch目标角度（×100）
        uint8_t  grip_target;       ///< 夹爪的目标开合度（0-255）
    } __packed armJoint2Target = {};

    /**
     * @brief 包2 - 云台
     * @note  8字节，包含云台3轴目标角度
     */
    struct SGimbalTarget {
        uint8_t  pack_id;            ///< 包ID = 2
        int16_t  gimbal_lift_target; ///< 云台抬升目标角度（×100）
        int16_t  gimbal_yaw_target;  ///< 云台Yaw目标角度（×100）
        int16_t  gimbal_pitch_target;///< 云台Pitch目标角度（×100）
        uint8_t  reserved;           ///< 预留
    } __packed gimbalTarget = {};

    /**
     * @brief 包3 - 控制标志
     * @note  8字节，包含遥控器状态、工作模式、命令标志等
     */
    struct SControlFlags {
        uint8_t  pack_id;           ///< 包ID = 3
        uint8_t  rc_switch_R : 2;   ///< 右拨杆状态（1=上 2=中 3=下）或可改成标志位
        uint8_t  reserved_1 : 6;     ///< 预留
        uint8_t  is_rc_ctrl : 1;    ///< 遥控器控制模式标志
        uint8_t  is_key_ctrl : 1;   ///< 键盘控制模式标志
        uint8_t  reserved_2 : 6;     ///< 预留
        uint8_t  work_mode;         ///< 工作模式（0=停止 1=双臂协同 2=主臂工作，副臂休息）
        uint8_t  cmd_grip : 1;      ///< 抓取命令
        uint8_t  cmd_release : 1;   ///< 释放命令
        uint8_t  emergency_stop : 1;///< 急停信号，专用上台阶
        uint8_t  arm_enable : 1;    ///< 机械臂使能
        uint8_t  gimbal_enable : 1; ///< 云台使能
        uint8_t  rc_status : 1; 	///< 遥控器是否关控，是0非1
        uint8_t  reserved_3 : 2;     ///< 预留
        uint8_t  reserved4[3] = {0};      ///< 预留给未来扩展
    } __packed ctrlFlags = {};

    /**
     * @brief 反馈包 - 副板发送给主板
     * @note  8字节，包含接收状态和通信状态
     */
    struct SFeedbackPack {
        uint8_t  pack_id;           ///< 包ID = 0xFE（反馈包标识）
        uint8_t  rx_status;         ///< 各包接收状态（bit0~3对应包0~3，1=已接收）
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
     * @brief 填充反馈数据到发送缓冲区
     * @note  只填充数据，不发送。发送由 sys_task.cpp 统一管理
     */
    void SendFeedback();

private:

    CInfCAN::CCanRxNode rxNode_;             ///< CAN接收节点
    CInfCAN::CCanTxNode *txNode_ = nullptr;  ///< CAN发送节点指针

    /**
     * @brief 接收状态管理（使用union和位域）
     * @note  用于跟踪各数据包的接收情况
     *        可通过 rxStatus_.all 访问整个字节
     *        可通过 rxStatus_.bit.xxx 访问单个位
     */
    union URxStatus {
        uint8_t all = 0;  ///< 整体状态字节
        struct {
            uint8_t pkt_arm_joint1 : 1;  ///< bit0 - 包0机械臂关节1已接收
            uint8_t pkt_arm_joint2 : 1;  ///< bit1 - 包1机械臂关节2已接收
            uint8_t pkt_gimbal : 1;      ///< bit2 - 包2云台目标已接收
            uint8_t pkt_ctrl_flags : 1;  ///< bit3 - 包3控制标志已接收
            uint8_t reserved : 4;        ///< bit4~7 预留
        } bit;

        // 边界检查标记某个包已接收
        void SetReceived(uint8_t packId) {
            if (packId < PKT_COUNT) all |= (1 << packId);
        }

        // 检查某个包是否已接收
        bool IsReceived(uint8_t packId) const { return (all & (1 << packId)) != 0; }

        // 检查是否所有包都已接收
        bool IsAllReceived() const { return (all & 0x0F) == 0x0F; }

        // 清除所有状态
        void Clear() { all = 0; }

        // 获取状态值
        uint8_t GetBits() const { return all; }
    } rxStatus_;

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
