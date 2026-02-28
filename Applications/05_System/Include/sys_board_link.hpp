/******************************************************************************
 * @brief        板间通信系统类
 * 
 * @file         sys_board_link.hpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-07
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#ifndef SYS_BOARD_LINK_HPP
#define SYS_BOARD_LINK_HPP

#include "sys_common.hpp"
#include "Device.hpp"

namespace my_engineer {

/**
 * @brief 板间通信系统类
 * 
 */
class CSystemBoardLink final: public CSystemBase{
public:

	// 定义板间通信系统初始化参数结构体
	struct SSystemInitParam_BoardLink: public SSystemInitParam_Base{
		EDeviceID boardLinkDevID = EDeviceID::DEV_NULL; ///< 板间通信设备ID
	};

    //发送信息结构体

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

        // 状态标志 (1字节)
        uint8_t  rc_status : 1;             ///< 遥控器在线，是1非0
        uint8_t  ctrl_mode : 3;             ///< 控制模式
        uint8_t  move_mode : 3;             ///< 运动模式
        uint8_t  emergency_stop : 1;        ///< 急停信号

        // 自动控制标志
        uint8_t  chassis_auto_ctrl : 1;     ///< 底盘自动控制标志位
        uint8_t  gimbal_auto_ctrl : 1;      ///< 云台自动控制标志位
        uint8_t  arm_auto_ctrl : 1;         ///< 臂自动控制标志位
        uint8_t  auto_ctrl_mode : 5;        ///< 自动控制任务类型

        uint8_t  reserved[4];               ///< 预留给未来扩展
    } __packed ctrlFlags = {};

    struct SControllerBackCmd_L{
        uint8_t pack_id;        ///< 包ID = 1

        // 角度指令
        int16_t yaw;            ///< Yaw角度 (×100)
	    int16_t pitch1;         ///< Pitch1角度 (×100)
	    int16_t pitch2;         ///< Pitch2角度 (×100)
        uint8_t  reserved;          ///< 预留
        
    } __packed controllerbackcmd_l = {};

    struct SControllerFrontCmd_L{
        uint8_t pack_id;        ///< 包ID = 2

        // 角度指令
        int16_t roll;       ///< Roll角度 (×100)
	    int16_t pitch_end;  ///< PitchEnd角度 (×100)
        int8_t roll_end;    ///< 左臂roll_end增量 (-100~100)，控制第6轴
        uint8_t grip_close; ///< 夹爪闭合
        uint8_t chassis_speed;  ///< 底盘速度
        
    } __packed controllerfrontcmd_l = {};

    	// 接收信息结构体
    struct SFeedbackPack {
        uint8_t  pack_id;       ///< 包ID = 0xFE
        uint8_t  pack0_status : 1;
        uint8_t  pack1_status : 1;         
        uint8_t  pack2_status : 1;
        uint8_t  pack3_status : 1;  ///< 各包接收状态（bit0~3对应包0~3，1=已接收）
        uint8_t  pack_status : 4;   /// 包通信状态 保留
        uint8_t  link_status;       ///< 通信状态（0=RESET 1=OFFLINE 2=ONLINE）
        uint8_t  reserved[5];       ///< 预留
    } __packed fdbInfo = {};

	// 初始化系统
	EAppStatus InitSystem(SSystemInitParam_Base *pStruct) final;

private:

	// 板间通信设备指针
	CDevBoardLink *pboardLink_ = nullptr;

	void UpdateHandler_() final;

	void HeartbeatHandler_() final;

	// 更新接收包信息
	void UpdateBoardRxData_();

	// 更新发送包信息
	void UpdateBoardTxPkg_();

};

extern CSystemBoardLink SysBoardLink;

}   // namespace my_engineer

#endif // SYS_BOARD_LINK_HPP
