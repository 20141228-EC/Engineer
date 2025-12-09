/******************************************************************************
 * @brief        
 * 
 * @file         sys_board_link.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-07
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "sys_board_link.hpp"

namespace my_engineer {

// 实例化一个板间通信系统
CSystemBoardLink SysBoardLink;

/**
 * @brief 初始化板间通信系统
 * 
 * @param pStruct 
 * @return EAppStatus 
 */
EAppStatus CSystemBoardLink::InitSystem(SSystemInitParam_Base *pStruct) {
	
	// 检查参数及ID是否为空
	if (pStruct == nullptr) return APP_ERROR;
	if (pStruct->systemID == ESystemID::SYS_NULL) return APP_ERROR;

	// 类型转换
	auto &param = *reinterpret_cast<SSystemInitParam_BoardLink *>(pStruct);

	// 初始化控制器通信设备
	systemID = param.systemID;
	auto it = DeviceIDMap.find(param.boardLinkDevID);
	if (it != DeviceIDMap.end() && it->second != nullptr) {
		pboardLink_ = static_cast<CDevBoardLink *>(it->second);
	}

	// 注册系统
	RegisterSystem_();

	systemStatus = APP_ERROR; ///< 初始化为error
	return APP_OK;
}

/**
 * @brief 更新处理
 * 
 */
void CSystemBoardLink::UpdateHandler_() {
	// 检查系统状态
	if (systemStatus != APP_OK) return;

	// static uint8_t delay = 40;
	// delay -= 4;
	// if (delay > 0) return;
	// delay = 40; // 分频

    //此处先不分频，如果后面can负载爆了再说

	if (!pboardLink_) return;

	// 更新副板信息
	UpdateBoardRxData_();
	// 更新发送数据包
	UpdateBoardTxPkg_();
	// 发送机器人信息

	for(uint8_t i = 0; i < CDevBoardLink::EPacketID::PKT_COUNT; i++){
            pboardLink_->SendPackage(static_cast<CDevBoardLink::EPacketID>(i));
    }

	// // 获取当前系统时间
    // uint32_t now = HAL_GetTick(); 
	// static uint32_t last_control_send_time = 0;
	// static uint32_t last_backarm_send_time = 0;
	// static uint32_t last_forwardarm_send_time = 0;

    // // 发送高频数据包 (100Hz)
    // if (now - last_control_send_time >= 5)
    // {
    //     last_control_send_time = now;
    //     pboardLink_->SendPackage(CDevBoardLink::EPacketID::PKT_CTRL_FLAGS); 
    // }

    // // 发送中频数据包 (50Hz)
    // if (now - last_forwardarm_send_time >= 10)
    // {
    //     last_forwardarm_send_time = now;
    //     pboardLink_->SendPackage(CDevBoardLink::EPacketID::PKT_ARM_BACKWARD);
    // }

    // // 发送低频数据包 (25Hz)
    // if (now - last_backarm_send_time >= 20)
    // {
    //     last_backarm_send_time = now;
    //     pboardLink_->SendPackage(CDevBoardLink::EPacketID::PKT_ARM_FORWARD);
    // }
    
    //如果can负载爆了的话也可以试试不用for 换上面这种方式发
}

/**
 * @brief 更新副板信息
 * 
 */
void CSystemBoardLink::UpdateBoardRxData_() {
	if (systemStatus != APP_OK) return;
	if (!pboardLink_) return;

	// 从设备层更新副板信息
    fdbInfo.pack0_status = pboardLink_->fdbInfo_pkt.pack0_status;
    fdbInfo.pack0_status = pboardLink_->fdbInfo_pkt.pack0_status;
    fdbInfo.pack0_status = pboardLink_->fdbInfo_pkt.pack0_status;
    fdbInfo.pack0_status = pboardLink_->fdbInfo_pkt.pack0_status; ///< 这个说不存在成员不用管 vscode乱报错 ninja编译是可以过的
	
}

/**
 * @brief 更新发送数据包
 */
void CSystemBoardLink::UpdateBoardTxPkg_() {
	if (systemStatus != APP_OK) return;
	if (!pboardLink_) return;

	// 更新发送包的信息 将系统层的数据传递给设备层

    // 更新包0 - 机械臂后三轴
    pboardLink_->armBackwardTarget_pkt.arm_yaw_target = armBackwardTarget.arm_yaw_target;
    pboardLink_->armBackwardTarget_pkt.arm_pitch1_target = armBackwardTarget.arm_pitch1_target;
    pboardLink_->armBackwardTarget_pkt.arm_pitch2_target = armBackwardTarget.arm_pitch2_target;
    
    // 更新包1 - 机械臂前四轴
    pboardLink_->armForwardTarget_pkt.arm_roll_target = armForwardTarget.arm_roll_target;
    pboardLink_->armForwardTarget_pkt.grip_roll_target = armForwardTarget.grip_roll_target;
    pboardLink_->armForwardTarget_pkt.grip_pitch_target = armForwardTarget.grip_pitch_target;
    pboardLink_->armForwardTarget_pkt.grip_target = armForwardTarget.grip_target;

    // 更新包2 - 云台
    pboardLink_->gimbalTarget_pkt.gimbal_lift_target = gimbalTarget.gimbal_lift_target;
    pboardLink_->gimbalTarget_pkt.gimbal_yaw_target = gimbalTarget.gimbal_yaw_target;
    pboardLink_->gimbalTarget_pkt.gimbal_pitch_target = gimbalTarget.gimbal_pitch_target;

    // 更新包3 - 控制标志
    pboardLink_->ctrlFlags_pkt.rc_switch_R = ctrlFlags.rc_switch_R;
    pboardLink_->ctrlFlags_pkt.is_rc_ctrl = ctrlFlags.is_rc_ctrl;
    pboardLink_->ctrlFlags_pkt.is_key_ctrl = ctrlFlags.is_key_ctrl;
    pboardLink_->ctrlFlags_pkt.work_mode = ctrlFlags.work_mode;
    pboardLink_->ctrlFlags_pkt.cmd_grip = ctrlFlags.cmd_grip;
    pboardLink_->ctrlFlags_pkt.cmd_release = ctrlFlags.cmd_release;
    pboardLink_->ctrlFlags_pkt.arm_reset = ctrlFlags.arm_reset;
    pboardLink_->ctrlFlags_pkt.arm_enable = ctrlFlags.arm_enable;
    pboardLink_->ctrlFlags_pkt.gimbal_enable = ctrlFlags.gimbal_enable;
    pboardLink_->ctrlFlags_pkt.rc_status = ctrlFlags.rc_status;
}

/**
 * @brief 心跳处理
 * 
 */
void CSystemBoardLink::HeartbeatHandler_() {
	systemStatus = APP_OK;
}

} // namespace my_engineer

