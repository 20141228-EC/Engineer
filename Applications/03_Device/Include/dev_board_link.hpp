/******************************************************************************
 * @brief   板间通信设备类
 *
 * @file    dev_board_link.hpp
 * @author  Ciallo～(∠·ω< )⌒☆(1002046597@qq.com)
 * @version V2.0
 * @date    2025-12-09
 *
 * @details V1.0：用于主副板之间的CAN通信
 *          V2.0: 重构协议，传递遥控器原始摇杆值和控制标志位
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
        PKT_REMOTE_1   = 0,  ///< 遥控器摇杆包1（RX, RY, LX）
        PKT_REMOTE_2   = 1,  ///< 遥控器摇杆包2（LY, 拨轮）
        PKT_CTRL_FLAGS = 2,  ///< 控制标志包
        PKT_COUNT,           ///< 包类型数量
        PKT_FEEDBACK   = 0xFE,  ///< 反馈包（副板发送给主板）
    };

    /**
     * @brief 包0 - 遥控器摇杆包1
     * @note  8字节，包含右摇杆XY和左摇杆X
     *        数据格式: 主板发送值 × 3 ÷ 100 = 原始摇杆值 (-660~660)
     *        系统层转换: 接收值 ÷ 220 = 百分比值 (-100~100)
     */
    struct SRemoteJoystick1 {
        uint8_t  pack_id;           ///< 包ID = 0
        int16_t  joystick_RX;       ///< 右摇杆X（需 ÷220 转百分比）
        int16_t  joystick_RY;       ///< 右摇杆Y（需 ÷220 转百分比）
        int16_t  joystick_LX;       ///< 左摇杆X（需 ÷220 转百分比）
        uint8_t  reserved;          ///< 预留
    } __packed remoteJoystick1 = {};

    /**
     * @brief 包1 - 遥控器摇杆包2
     * @note  8字节，包含左摇杆Y、拨轮和拨杆状态
     *        摇杆/拨轮数据格式同包0
     */
    struct SRemoteJoystick2 {
        uint8_t  pack_id;           ///< 包ID = 1
        int16_t  joystick_LY;       ///< 左摇杆Y（需 ÷220 转百分比）
        int16_t  thumbWheel;        ///< 拨轮（需 ÷220 转百分比）
        uint8_t  switch_L;          ///< 左拨杆状态（1/2/3）
        uint8_t  switch_R;          ///< 右拨杆状态（1/2/3）
        uint8_t  reserved;          ///< 预留
    } __packed remoteJoystick2 = {};

    /**
     * @brief 包2 - 控制标志包
     * @note  8字节，包含控制模式、使能标志、状态标志（由主板根据拨杆状态计算）
     */
    struct SControlFlags {
        uint8_t  pack_id;           ///< 包ID = 2

        // 控制模式 (1字节)
        uint8_t  chassis_ctrl : 1;  ///< 底盘控制使能
        uint8_t  gimbal_ctrl : 1;   ///< 云台控制使能
        uint8_t  arm_front_ctrl : 1;///< 机械臂前四轴控制
        uint8_t  arm_rear_ctrl : 1; ///< 机械臂后四轴控制
        uint8_t  reserved_mode : 4; ///< 预留

        // 使能标志 (1字节)
        uint8_t  arm_enable : 1;    ///< 机械臂使能
        uint8_t  gimbal_enable : 1; ///< 云台使能
        uint8_t  chassis_enable : 1;///< 底盘使能
        uint8_t  reserved_en : 5;   ///< 预留

        // 状态标志 (1字节)
        uint8_t  rc_online : 1;     ///< 遥控器在线
        uint8_t  is_rc_ctrl : 1;    ///< 遥控器控制模式
        uint8_t  is_key_ctrl : 1;   ///< 键盘控制模式
        //uint8_t  climb_stair : 1;   ///< 上台阶标志
        uint8_t  reserved_st : 5;   ///< 预留

        uint8_t  reserved[4];       ///< 预留给未来扩展
    } __packed ctrlFlags = {};

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
     * @brief 填充反馈数据到发送缓冲区
     * @note  只填充数据到发送缓冲区，不发送。发送由CAN接口层统一管理
     */
    void FillFeedbackBuffer();

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
            uint8_t pkt_remote_1 : 1;    ///< bit0 - 包0遥控器摇杆1已接收
            uint8_t pkt_remote_2 : 1;    ///< bit1 - 包1遥控器摇杆2已接收
            uint8_t pkt_ctrl_flags : 1;  ///< bit2 - 包2控制标志已接收
            uint8_t reserved : 5;        ///< bit3~7 预留
        } bit;

        // 边界检查标记某个包已接收
        void SetReceived(uint8_t packId) {
            if (packId < PKT_COUNT) all |= (1 << packId);
        }

        // 检查某个包是否已接收
        bool IsReceived(uint8_t packId) const { return (all & (1 << packId)) != 0; }

        // 检查是否所有包都已接收
        bool IsAllReceived() const { return (all & 0x07) == 0x07; }

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