/******************************************************************************
 * @brief        板间通信系统
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

    // 更新包0 - 遥控器值（右摇杆xy、左摇杆x）
    pboardLink_->remoteInfo1_pkt.joystick_RX = remoteInfo1.joystick_RX;
    pboardLink_->remoteInfo1_pkt.joystick_RY = remoteInfo1.joystick_RY;
    pboardLink_->remoteInfo1_pkt.joystick_LX = remoteInfo1.joystick_LX;

    // 更新包1 - 遥控器值（左摇杆y、拨轮和拨杆）
    pboardLink_->remoteInfo2_pkt.joystick_LY = remoteInfo2.joystick_LY;
    pboardLink_->remoteInfo2_pkt.thumbWheel = remoteInfo2.thumbWheel;

    // 更新包2 - 控制标志
    pboardLink_->ctrlFlags_pkt.chassis_ctrl = ctrlFlags.chassis_ctrl;
    pboardLink_->ctrlFlags_pkt.gimbal_ctrl = ctrlFlags.gimbal_ctrl;
    pboardLink_->ctrlFlags_pkt.arm_front_ctrl = ctrlFlags.arm_front_ctrl;
    pboardLink_->ctrlFlags_pkt.arm_rear_ctrl = ctrlFlags.arm_rear_ctrl;

    pboardLink_->ctrlFlags_pkt.arm_enable = ctrlFlags.arm_enable;
    pboardLink_->ctrlFlags_pkt.gimbal_enable = ctrlFlags.gimbal_enable;
    pboardLink_->ctrlFlags_pkt.chassis_enable = ctrlFlags.chassis_enable;

    pboardLink_->ctrlFlags_pkt.rc_status = ctrlFlags.rc_status;
    pboardLink_->ctrlFlags_pkt.ctrl_mode = ctrlFlags.ctrl_mode;
    pboardLink_->ctrlFlags_pkt.move_mode = ctrlFlags.move_mode;
    pboardLink_->ctrlFlags_pkt.emergency_stop = ctrlFlags.emergency_stop;
}

/**
 * @brief 心跳处理
 * 
 */
void CSystemBoardLink::HeartbeatHandler_() {
	systemStatus = APP_OK;
}

} // namespace my_engineer

