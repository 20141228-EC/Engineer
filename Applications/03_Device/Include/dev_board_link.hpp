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
        PKT_REMOTE_1 = 0,      ///< 遥控器值（右摇杆xy、左摇杆x）
        PKT_REMOTE_2 = 1,      ///< 遥控器值（左摇杆y、拨轮和拨杆）
        PKT_CTRL_FLAGS = 2,  ///< 控制标志
        PKT_CTRLER_L_B = 3,     ///< 控制器左臂后三轴
        PKT_CTRLER_R_B = 4,     ///< 控制器右臂后三轴
        PKT_CTRLER_L_F = 5,     ///< 控制器左臂前三轴
        PKT_CTRLER_R_F = 6,     ///< 控制器右臂前三轴
        PKT_KEYB = 7,          ///< 键鼠
        PKT_COUNT,           ///< 发送包类型数量
        PKT_FEEDBACK   = 0xFE,  ///< 反馈包（副板发送给主板）
    };

    /**
     * @brief 包0 - 遥控器摇杆包1
     * @note  8字节，包含右摇杆XY和左摇杆X的原始值
     */
    struct SRemoteJoystick1 {
        uint8_t  pack_id;           ///< 包ID = 0
        int16_t  joystick_RX;       ///< 右摇杆X(原始值归一到±100.f内再放大220倍)
        int16_t  joystick_RY;       ///< 右摇杆Y(原始值归一到±100.f内再放大220倍)
        int16_t  joystick_LX;       ///< 左摇杆X(原始值归一到±100.f内再放大220倍)
        uint8_t  reserved;          ///< 预留
    } __packed remoteInfo1_pkt = {};

    /**
     * @brief 包1 - 遥控器包2
     * @note  8字节，包含左摇杆Y和拨轮的原始值和拨杆值
     */
    struct SRemoteJoystick2 {
        uint8_t  pack_id;           ///< 包ID = 1
        int16_t  joystick_LY;       ///< 左摇杆Y(原始值归一到±100.f内再放大220倍)
        int16_t  thumbWheel;        ///< 拨轮(原始值归一到±100.f内再放大220倍)
        uint8_t  switch_l;          ///< 左拨杆
        uint8_t  switch_r;          ///< 右拨杆
        uint8_t  reserved[1];       ///< 预留
    } __packed remoteInfo2_pkt = {};

    /**
     * @brief 包2 - 控制标志
     * @note  8字节，包含遥控器状态、工作模式、命令标志等
     */
    struct SControlFlags {
        uint8_t  pack_id;           ///< 包ID = 2

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
        uint8_t pack_id;        ///< 包ID = 3

        // 角度指令
        int16_t yaw;            ///< Yaw角度 (×100)
	    int16_t pitch1;         ///< Pitch1角度 (×100)
	    int16_t pitch2;         ///< Pitch2角度 (×100)
        uint8_t  reserved;          ///< 预留
        
    } __packed controllerbackcmd_l_b_pkt = {};

    /**
     * @brief 自定义控制器右臂后三轴命令包
     * @note  自定义控制模式下的臂目标位置等
     * 
     */
    struct SControllerBackCmd_R{
        uint8_t pack_id;        ///< 包ID = 4

        // 角度指令
        int16_t yaw;            ///< Yaw角度 (×100)
	    int16_t pitch1;         ///< Pitch1角度 (×100)
	    int16_t pitch2;         ///< Pitch2角度 (×100)
        uint8_t  reserved;          ///< 预留
        
    } __packed controllerbackcmd_r_b_pkt = {};

    /**
     * @brief 自定义控制器左臂前三轴命令包
     * @note  自定义控制模式下的臂目标位置等
     * 
     */
    struct SControllerFrontCmd_L{
        uint8_t pack_id;        ///< 包ID = 5

        // 角度指令
        int16_t roll;       ///< Roll角度 (×100)
	    int16_t pitch_end;  ///< PitchEnd角度 (×100)
        int8_t roll_end;    ///< 左臂roll_end增量 (-100~100)，控制第6轴
        uint8_t grip_close; ///< 夹爪闭合

        uint8_t reserved;    ///< 保留
        
    } __packed controllerfrontcmd_l_f_pkt = {};

    /**
     * @brief 自定义控制器右臂前三轴命令包
     * @note  自定义控制模式下的臂目标位置等
     * 
     */
    struct SControllerFrontCmd_R{
        uint8_t pack_id;        ///< 包ID = 6

        // 角度指令
        int16_t roll;       ///< Roll角度 (×100)
	    int16_t pitch_end;  ///< PitchEnd角度 (×100)
        int8_t roll_end;    ///< 右臂roll_end增量 (-100~100)，控制第6轴
        uint8_t grip_close; ///< 夹爪闭合
        uint8_t chassis_speed;  ///< 底盘速度
        
    } __packed controllerfrontcmd_r_f_pkt = {};

    /**
     * @brief 键鼠信息包
     * @note  包括键盘和鼠标信息
     * 
     */
    struct SKeyBoardInfo{
        uint8_t pack_id;        ///< 包ID = 7
        
        uint8_t KEY_W     : 1;      ///< W键
        uint8_t KEY_S     : 1;      ///< S键
        uint8_t KEY_A     : 1;      ///< A键
        uint8_t KEY_D     : 1;      ///< D键
        uint8_t KEY_SHIFT : 1;      ///< SHIFT键
        uint8_t KEY_CTRL  : 1;      ///< CTRL键
        uint8_t KEY_Q     : 1;      ///< Q键
        uint8_t KEY_E     : 1;      ///< E键

        uint8_t KEY_R     : 1;      ///< R键
        uint8_t KEY_F     : 1;      ///< F键
        uint8_t KEY_G     : 1;      ///< G键
        uint8_t KEY_Z     : 1;      ///< Z键
        uint8_t KEY_X     : 1;      ///< X键
        uint8_t KEY_C     : 1;      ///< C键
        uint8_t KEY_V     : 1;      ///< V键
        uint8_t KEY_B     : 1;      ///< B键

        uint8_t MOUSE_L   : 1;      ///< 左键
        uint8_t MOUSE_R   : 1;      ///< 右键
        uint8_t reserved  : 6;      ///< 保留

        int16_t mouse_vx;           ///< 鼠标x轴速度
        int16_t mouse_vy;           ///< 鼠标y轴速度
        
    } __packed keyboard_pkt = {};
    
    

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
