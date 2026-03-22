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

	// for(uint8_t i = 0; i < CDevBoardLink::EPacketID::PKT_COUNT; i++){
    //         pboardLink_->SendPackage(static_cast<CDevBoardLink::EPacketID>(i));
    // }

	// 获取当前系统时间
    uint32_t now = HAL_GetTick(); 
	static uint32_t last_control_send_time = 0;
	static uint32_t last_backarm_send_time = 0;
	static uint32_t last_forwardarm_send_time = 0;

    // 发送高频数据包 (500Hz)
    if (now - last_control_send_time >= 1)
    {
        last_control_send_time = now;
        pboardLink_->SendPackage(CDevBoardLink::EPacketID::PKT_CTRL_FLAGS); 
    }

    // 发送控制器数据包 (250Hz)
   if (now - last_backarm_send_time >= 4)
   {
       last_backarm_send_time = now;
       for(int i = CDevBoardLink::EPacketID::PKT_CTRLER_L_B; i < CDevBoardLink::EPacketID::PKT_COUNT; ++i){
           pboardLink_->SendPackage(static_cast<CDevBoardLink::EPacketID>(i));
   }   // 暂且写成for循环发 后面不行再看看改一下
    }
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
    fdbInfo.pack_id = pboardLink_->fdbInfo_pkt.pack_id;
    fdbInfo.pack0_status = pboardLink_->fdbInfo_pkt.pack0_status;
    fdbInfo.pack1_status = pboardLink_->fdbInfo_pkt.pack1_status;
    fdbInfo.pack2_status = pboardLink_->fdbInfo_pkt.pack2_status;
    fdbInfo.pack3_status = pboardLink_->fdbInfo_pkt.pack3_status; ///< 这个说不存在成员不用管 vscode乱报错 ninja编译是可以过的

    gimbalInfo.pack_id = pboardLink_->gimbalInfo_pkt.pack_id;
    gimbalInfo.yaw = pboardLink_->gimbalInfo_pkt.yaw;
    gimbalInfo.yaw_rate = pboardLink_->gimbalInfo_pkt.yaw_rate;
    gimbalInfo.data_valid = pboardLink_->gimbalInfo_pkt.data_valid;
	
}

/**
 * @brief 更新发送数据包
 */
void CSystemBoardLink::UpdateBoardTxPkg_() {
    if (systemStatus != APP_OK) return;
    if (!pboardLink_) return;

    // 更新发送包的信息 将系统层的数据传递给设备层

    // 更新包0 - 控制标志
    pboardLink_->ctrlFlags_pkt.pack_id = ctrlFlags.pack_id;
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

    pboardLink_->ctrlFlags_pkt.chassis_auto_ctrl = ctrlFlags.chassis_auto_ctrl;
    pboardLink_->ctrlFlags_pkt.gimbal_auto_ctrl = ctrlFlags.gimbal_auto_ctrl;
    pboardLink_->ctrlFlags_pkt.arm_auto_ctrl = ctrlFlags.arm_auto_ctrl;
    pboardLink_->ctrlFlags_pkt.auto_ctrl_mode = ctrlFlags.auto_ctrl_mode;

    // 更新包1 - 控制器左臂后三轴命令包
    pboardLink_->controllerbackcmd_l_b_pkt.pack_id = controllerbackcmd_l.pack_id;
    pboardLink_->controllerbackcmd_l_b_pkt.yaw = controllerbackcmd_l.yaw;
    pboardLink_->controllerbackcmd_l_b_pkt.pitch1 = controllerbackcmd_l.pitch1;
    pboardLink_->controllerbackcmd_l_b_pkt.pitch2 = controllerbackcmd_l.pitch2;

    // 更新包3 - 控制器左臂前三轴命令包
    pboardLink_->controllerfrontcmd_l_f_pkt.pack_id = controllerfrontcmd_l.pack_id;
    pboardLink_->controllerfrontcmd_l_f_pkt.roll = controllerfrontcmd_l.roll;
    pboardLink_->controllerfrontcmd_l_f_pkt.pitch_end = controllerfrontcmd_l.pitch_end;
    pboardLink_->controllerfrontcmd_l_f_pkt.roll_end = controllerfrontcmd_l.roll_end;
    pboardLink_->controllerfrontcmd_l_f_pkt.grip_close = controllerfrontcmd_l.grip_close;
    pboardLink_->controllerfrontcmd_l_f_pkt.chassis_speed = controllerfrontcmd_l.chassis_speed;
    
}

/**
 * @brief 心跳处理
 * 
 */
void CSystemBoardLink::HeartbeatHandler_() {
	systemStatus = APP_OK;
}

} // namespace my_engineer

