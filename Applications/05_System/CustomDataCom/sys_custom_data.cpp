#include "sys_custom_data.hpp"
#include "Configuration.hpp"
#include "Core.hpp"
#include "mod_chassis.hpp"
#include "mod_arm.hpp"
#include "mod_common.hpp"
#include "mod_gimbal.hpp"
#include "sys_controller_link.hpp"

namespace my_engineer {

CSystemCustomDataCom SysCustomDataCom;

/**
 * @brief 映射自动控制流程枚举
 */
static AutoCtrlProcess MapAutoProcess(CSystemCore::EAutoCtrlProcess process) {
	using E = CSystemCore::EAutoCtrlProcess;
    switch (process) {
        case E::NONE:          return AutoCtrlProcess_PROCESS_NONE;
        case E::RETURN_ORIGIN: return AutoCtrlProcess_PROCESS_RETURN_ORIGIN;
        case E::CLIMBING:      return AutoCtrlProcess_PROCESS_CLIMBING;
        case E::DOWN_STAIR:    return AutoCtrlProcess_PROCESS_DOWN_STAIR;
        case E::ENERGY_UNIT:   return AutoCtrlProcess_PROCESS_ENERGY_UNIT;
        case E::EXCHANGE_ORE:  return AutoCtrlProcess_PROCESS_EXCHANGE_ORE;
        case E::STORE_ORE:     return AutoCtrlProcess_PROCESS_STORE_ORE;
        case E::GROUND_ORE:    return AutoCtrlProcess_PROCESS_GROUND_ORE;
        default: return AutoCtrlProcess_PROCESS_NONE;
    }
}

EAppStatus CSystemCustomDataCom::InitSystem(SSystemInitParam_Base *pStruct) {
    if (pStruct == nullptr) return APP_ERROR;
    auto &param = *static_cast<SSystemInitParam_CustomDataCom *>(pStruct);
    
    systemID = param.systemID;
    
    auto it = DeviceIDMap.find(param.customDataDevID);
    if (it != DeviceIDMap.end()) {
        pDev_ = reinterpret_cast<CDevCustomData *>(it->second);
    }

    RegisterSystem_();
    systemStatus = APP_OK;
    return APP_OK;
}

void CSystemCustomDataCom::UpdateHandler_() {
    if (pDev_ == nullptr || systemStatus == APP_RESET) return;

    static uint32_t last_send_tick = 0;
    if (HAL_GetTick() - last_send_tick < 20) return;
    last_send_tick = HAL_GetTick();

    auto it_chassis = ModuleIDMap.find(EModuleID::MOD_CHASSIS);
    auto it_arm = ModuleIDMap.find(EModuleID::MOD_ARM);
	auto it_gimbal = ModuleIDMap.find(EModuleID::MOD_GIMBAL);
    
    if (it_chassis == ModuleIDMap.end() || it_arm == ModuleIDMap.end() || it_gimbal == ModuleIDMap.end()) return;
    
    auto &chassis_info = reinterpret_cast<CModChassis *>(it_chassis->second)->chassisInfo;
    auto &arm_info = reinterpret_cast<CModArm *>(it_arm->second)->armInfo;
    auto &arm_cmd = reinterpret_cast<CModArm *>(it_arm->second)->armCmd;
	auto &gimbal_info = reinterpret_cast<CModGimbal*>(it_gimbal->second)->gimbalInfo;

    EngineerDataPacketToClient packet = EngineerDataPacketToClient_init_default;

    packet.has_current_process = true;
    packet.current_process = MapAutoProcess(SystemCore.currentAutoCtrlProcess_);
    
    packet.has_ctrl_mode = true;
    packet.ctrl_mode = SystemCore.use_Controller_ ? CtrlMode_CTRL_CONTROLLER : CtrlMode_CTRL_RC;

    packet.has_chassis = true;
    packet.chassis.speed_x = chassis_info.speed_X;
    packet.chassis.speed_y = chassis_info.speed_Y;
    packet.chassis.speed_w = chassis_info.speed_W;
    packet.chassis.hip_length = chassis_info.L_Length;
    packet.chassis.crawler_on = chassis_info.crawler_on;
    packet.chassis.roll_angle = chassis_info.roll_Measure[0];

    packet.has_arm = true;
    packet.arm.angle_yaw = arm_info.angle_Yaw;
    packet.arm.angle_pitch1 = arm_info.angle_Pitch1;
    packet.arm.angle_pitch2 = arm_info.angle_Pitch2;
    packet.arm.angle_pitch3 = arm_info.angle_Pitch3;
    packet.arm.angle_roll = arm_info.angle_Roll;
    packet.arm.angle_end_pitch = arm_info.angle_end_pitch;
    packet.arm.angle_end_roll = arm_info.angle_end_roll;
    packet.arm.grip_close = arm_cmd.gripClose;

	packet.has_gimbal = true;
	packet.gimbal.yaw = gimbal_info.angle_visualyaw;

    packet.has_status = true;
    packet.status.p3_lock = SysControllerLink.robotInfo.p3_lock;
    packet.status.controlled_by_controller = SystemCore.use_Controller_;
    packet.status.robot_init_ok = (systemStatus == APP_OK);

    pDev_->SendProtobuf(packet);
}

void CSystemCustomDataCom::HeartbeatHandler_() {}

} // namespace my_engineer
