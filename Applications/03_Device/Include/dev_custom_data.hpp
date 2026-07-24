/******************************************************************************
 * @brief        自定义数据通信设备层, 负责将protobuf结构体送到串口
 * 
 * @file         dev_custom_data.hpp
 * @author       vix_hentx
 * @version      V1.0
 * @date         2026-04-30
 * 
 ******************************************************************************/

#pragma once

#include "dev_common.hpp"
#include "enginner.pb.h"
#include "inf_uart.hpp"
#include <cstdint>
#include <sys/cdefs.h>

namespace my_engineer {

class CDevCustomData final : public CDevBase {
public:
    static constexpr uint16_t tx_cmd_id = 0x0310;
    struct SDevInitParam_CustomData : public SDevInitParam_Base {
        EInterfaceID interfaceID = EInterfaceID::INF_NULL;
    };

    /**
     * @brief 图传链路通用帧头
     */
    struct SPkgHeader {
        uint8_t SOF = 0xA5;
        uint16_t pkgLen = sizeof(STxDataFrame);
        uint8_t seq = 0;
        uint8_t CRC8 = 0x00;
        uint16_t cmd_Id = tx_cmd_id;
    } __packed;

    struct STxDataFrame {
        uint16_t len;
        uint8_t data[298]; //这个需要填充到 { tx_data_len 字节}
    } __packed;

    static_assert(sizeof(STxDataFrame) == 300); //通信手册要求

    CDevCustomData() { deviceType = EDevType::DEV_CUSTOM_DATA_COM; }

    EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override;

    /**
     * @brief 发送 Protobuf 结构体, 走 send_cmd_id
     * @param message Nanopb 消息体
     */
    EAppStatus SendProtobuf(const EngineerDataPacketToClient &message);

private:
    CInfUART *uartInterface_ = nullptr;
    uint8_t txBuffer_[512] = {0};
};

} // namespace my_engineer
