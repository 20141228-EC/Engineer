/******************************************************************************
 * @brief        
 * 
 * @file         dev_board_link.hpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-06
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#ifndef DEV_BOARD_LINK_HPP
#define DEV_BOARD_LINK_HPP

#include "dev_common.hpp"
#include "inf_can.hpp"

namespace my_engineer{

/**
 * @brief 板间通信设备类
 * 
 */
class CDevBoardLink final: public CDevBase{
public:

	// 定义板间通信设备初始化参数结构体
	struct SDevInitParam_BoardLink: public SDevInitParam_Base{
		EInterfaceID interfaceID = EInterfaceID::INF_NULL; ///< CAN总线
	};

	//板间通信包信息

	/**
     * @brief 数据包ID枚举
     * @note  pack_id位于每个数据包的第一个字节
     */
    enum EPacketID : uint8_t {
        PKT_ARM_BACKWARD = 0,  ///< 机械臂后三轴（Yaw, Pitch1, Pitch2）
        PKT_ARM_FORWARD = 1,  ///< 机械臂前四轴（Roll, Grip_Roll, Grip_Pitch, 夹爪）
        PKT_GIMBAL     = 2,  ///< 云台目标
        PKT_CTRL_FLAGS = 3,  ///< 控制标志
        PKT_COUNT,           ///< 包类型数量
        PKT_FEEDBACK   = 0xFE,  ///< 反馈包（副板发送给主板）
    };

    /**
     * @brief 包0 - 机械臂后三轴（Yaw, Pitch1, Pitch2）
     * @note  8字节，包含前3个关节的目标角度
     */
    struct SArmBackwardTargetPKT {
        uint8_t  pack_id;           ///< 包ID = 0
        int16_t  arm_yaw_target;    ///< 基座Yaw目标角度（×100）
        int16_t  arm_pitch1_target; ///< 大臂Pitch1目标角度（×100）
        int16_t  arm_pitch2_target; ///< 小臂Pitch2目标角度（×100）
        uint8_t  reserved;          ///< 预留
    } __packed armBackwardTarget_pkt = {};

    /**
     * @brief 包1 - 机械臂前四轴（Roll, Grip_Roll, Grip_Pitch, 夹爪）
     * @note  8字节，包含末端Roll和夹爪两轴目标角度
     */
    struct SArmForwardTargetPKT {
        uint8_t  pack_id;           ///< 包ID = 1
        int16_t  arm_roll_target;   ///< 末端Roll目标角度（×100）
        int16_t  grip_roll_target;  ///< 夹爪Roll目标角度（×100）
        int16_t  grip_pitch_target; ///< 夹爪Pitch目标角度（×100）
        uint8_t  grip_target;          ///< 夹爪收放目标角度
    } __packed armForwardTarget_pkt = {};

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
    } __packed gimbalTarget_pkt = {};

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
        uint8_t  arm_reset : 1;		///< 臂复位信号
        uint8_t  arm_enable : 1;    ///< 机械臂使能
        uint8_t  gimbal_enable : 1; ///< 云台使能
		uint8_t  rc_status : 1; 	///< 遥控器是否关控，是0非1
        uint8_t  reserved_3 : 2;     ///< 预留
        uint8_t  reserved4[3] = {0};      ///< 预留给未来扩展
    } __packed ctrlFlags_pkt = {};

    /**
     * @brief 反馈包 - 副板发送给主板
     * @note  8字节，包含接收状态和通信状态
     */
    struct SFeedbackPack {
        uint8_t  pack_id;           ///< 包ID = 0xFE（反馈包标识）
        uint8_t  pack0_status : 1;
        uint8_t  pack1_status : 1;         
        uint8_t  pack2_status : 1;
        uint8_t  pack3_status : 1;  ///< 各包接收状态（bit0~3对应包0~3，1=已接收）
        uint8_t  pack_status : 4;   /// 包通信状态 保留
        uint8_t  link_status;       ///< 通信状态（0=RESET 1=OFFLINE 2=ONLINE）
        uint8_t  reserved[5];       ///< 预留
    } __packed fdbInfo_pkt = {};

	enum class EBoardLinkStatus {
		RESET,
		OFFLINE,
		ONLINE,
	} boardLinkStatus = EBoardLinkStatus::RESET; ///< 板间通信状态

	CDevBoardLink() {deviceType = EDevType::DEV_BOARD_LINK; }

	EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override;

	EAppStatus SendPackage(EPacketID pack_id);

    /**
	 * @brief 修改CAN发送数据
	 */
	EAppStatus Modify_CanTxData(uint8_t* data) 
	{
		if (data == nullptr) return APP_ERROR;
		std::copy(data, data + canTxNode_.dataBuffer.size(), canTxNode_.dataBuffer.begin());
		return APP_OK;
	}

private:

	CInfCAN *canInterface_ = nullptr; ///< CAN接口指针

    CInfCAN::CCanRxNode canRxNode_; ///< 定义接收节点

    CInfCAN::CCanTxNode canTxNode_; ///< 定义发送节点

	std::array<uint8_t, 8> rxBuffer_ = {0}; ///< 接收缓冲区

	uint32_t rxTimestamp_ = 0; ///< 接收时间戳

	void UpdateHandler_() override;

	void HeartbeatHandler_() override;

	EAppStatus ResolveRxPackage_();
};

} // namespace my_engineer

#endif // DEV_BOARD_LINK_HPP