/******************************************************************************
 * @brief        
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

    struct SArmBackwardTargetPKT {
        uint8_t  pack_id;           ///< 包ID = 0
        int16_t  arm_yaw_target;    ///< 基座Yaw目标角度（×100）
        int16_t  arm_pitch1_target; ///< 大臂Pitch1目标角度（×100）
        int16_t  arm_pitch2_target; ///< 小臂Pitch2目标角度（×100）
        uint8_t  reserved;          ///< 预留
    } __packed armBackwardTarget = {};

    struct SArmForwardTargetPKT {
        uint8_t  pack_id;           ///< 包ID = 1
        int16_t  arm_roll_target;   ///< 末端Roll目标角度（×100）
        int16_t  grip_roll_target;  ///< 夹爪Roll目标角度（×100）
        int16_t  grip_pitch_target; ///< 夹爪Pitch目标角度（×100）
        uint8_t  grip_target;          ///< 夹爪收放目标角度
    } __packed armForwardTarget = {};

    struct SGimbalTarget {
        uint8_t  pack_id;            ///< 包ID = 2
        int16_t  gimbal_lift_target; ///< 云台抬升目标角度（×100）
        int16_t  gimbal_yaw_target;  ///< 云台Yaw目标角度（×100）
        int16_t  gimbal_pitch_target;///< 云台Pitch目标角度（×100）
        uint8_t  reserved;           ///< 预留
    } __packed gimbalTarget = {};

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
    } __packed ctrlFlags = {};


	// 接收信息结构体
    struct SFeedbackPack {
        uint8_t  pack0_status : 1;
        uint8_t  pack1_status : 1;         
        uint8_t  pack2_status : 1;
        uint8_t  pack3_status : 1;  ///< 各包接收状态（bit0~3对应包0~3，1=已接收）
        uint8_t  pack_status : 4;   /// 包通信状态 保留
        uint8_t  link_status;       ///< 通信状态（0=RESET 1=OFFLINE 2=ONLINE）
        uint8_t  reserved[6];       ///< 预留
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
