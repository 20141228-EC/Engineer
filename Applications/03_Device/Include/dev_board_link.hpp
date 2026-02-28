/******************************************************************************
 * @brief        板间通信设备类
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
        PKT_CTRL_FLAGS = 0,  ///< 控制标志
        PKT_CTRLER_L_B = 1,     ///< 控制器左臂后三轴
        PKT_CTRLER_L_F = 2,     ///< 控制器左臂前三轴
        PKT_COUNT,           ///< 发送包类型数量
        PKT_FEEDBACK   = 0xFE,  ///< 反馈包（副板发送给主板）
    };

    /**
     * @brief 包0 - 控制标志
     * @note  8字节，包含遥控器状态、工作模式、命令标志等
     */
    struct SControlFlags {
        uint8_t  pack_id;           ///< 包ID = 0

        // 控制模式 (1字节)
        uint8_t  chassis_ctrl : 1;          ///< 底盘控制使能
        uint8_t  gimbal_ctrl : 1;           ///< 云台控制使能
        uint8_t  arm_front_ctrl : 1;        ///< 机械臂前三轴控制
        uint8_t  arm_rear_ctrl : 1;         ///< 机械臂后三轴控制
        uint8_t  reserved_mode : 4;         ///< 预留

        // 使能标志 (1字节)
        uint8_t  arm_enable : 1;            ///< 机械臂使能
        uint8_t  gimbal_enable : 1;         ///< 云台使能
        uint8_t  chassis_enable : 1;        ///< 底盘使能
        uint8_t  reserved_en : 5;           ///< 预留

        // 状态标志
        uint8_t  rc_status : 1;             ///< 遥控器在线，是1非0
        uint8_t  ctrl_mode : 3;             ///< 控制模式
        uint8_t  move_mode : 3;             ///< 运动模式
        uint8_t  emergency_stop : 1;        ///< 急停信号
        
        // 自动控制标志
        uint8_t  chassis_auto_ctrl : 1;     ///< 底盘自动控制标志位
        uint8_t  gimbal_auto_ctrl : 1;      ///< 云台自动控制标志位
        uint8_t  arm_auto_ctrl : 1;         ///< 臂自动控制标志位
        uint8_t  auto_ctrl_mode : 5;        ///< 自动控制任务类型

        uint8_t  reserved[3];               ///< 预留给未来扩展
    } __packed ctrlFlags_pkt = {};

    /**
     * @brief 自定义控制器左臂后三轴命令包
     * @note  自定义控制模式下的臂目标位置等
     * 
     */
    struct SControllerBackCmd_L{
        uint8_t pack_id;        ///< 包ID = 1

        // 角度指令
        int16_t yaw;            ///< Yaw角度 (×100)
	    int16_t pitch1;         ///< Pitch1角度 (×100)
	    int16_t pitch2;         ///< Pitch2角度 (×100)
        uint8_t  reserved;          ///< 预留
        
    } __packed controllerbackcmd_l_b_pkt = {};

    /**
     * @brief 自定义控制器左臂前三轴命令包
     * @note  自定义控制模式下的臂目标位置等
     * 
     */
    struct SControllerFrontCmd_L{
        uint8_t pack_id;        ///< 包ID = 2

        // 角度指令
        int16_t roll;       ///< Roll角度 (×100)
	    int16_t pitch_end;  ///< PitchEnd角度 (×100)
        int8_t roll_end;    ///< 左臂roll_end增量 (-100~100)，控制第6轴
        uint8_t grip_close; ///< 夹爪闭合

        uint8_t chassis_speed;  ///< 底盘速度
        
    } __packed controllerfrontcmd_l_f_pkt = {};
    
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
