/******************************************************************************
 * @brief        自定义客户端通信系统 (Service)
 * 
 * @file         sys_custom_data.hpp
 * @author       vix_hentx
 * @version      V1.1
 * @date         2026-04-30
 * 
 ******************************************************************************/

#pragma once

#include "Configuration.hpp"
#include "sys_common.hpp"
#include "dev_custom_data.hpp"
#include "enginner.pb.h"

namespace my_engineer {

class CSystemCustomDataCom final : public CSystemBase {
public:
    struct SSystemInitParam_CustomDataCom final : public SSystemInitParam_Base {
        EDeviceID customDataDevID; // 设备 ID
    };

    EAppStatus InitSystem(SSystemInitParam_Base *pStruct) override;

private:
    CDevCustomData *pDev_ = nullptr; // 设备指针
    EngineerDataPacketToClient msg;

    void UpdateHandler_() override;
    void HeartbeatHandler_() override;
};

extern CSystemCustomDataCom SysCustomDataCom;

} // namespace my_engineer
