#include "dev_custom_data.hpp"
#include "pb_encode.h"
#include "algo_crc.hpp"
#include <cstring>

namespace my_engineer {

EAppStatus CDevCustomData::InitDevice(const SDevInitParam_Base *pStructInitParam) {
    if (pStructInitParam == nullptr) return APP_ERROR;
    auto &param = *static_cast<const SDevInitParam_CustomData *>(pStructInitParam);

    deviceID = param.deviceID;
    uartInterface_ = reinterpret_cast<CInfUART *>(InterfaceIDMap.at(param.interfaceID));

    RegisterDevice_();
    deviceStatus = APP_OK;
    return APP_OK;
}

EAppStatus CDevCustomData::SendProtobuf(const EngineerDataPacketToClient &message) {
    /*
        发送顺序:
        SPkgHeader (7bytes) - STxDataFrame(300bytes) - CRC16(2bytes)
        Total: 309 bytes
    */
    constexpr uint16_t totalPkgLen = sizeof(SPkgHeader) + sizeof(STxDataFrame) + 2;

    if (uartInterface_ == nullptr || deviceStatus == APP_RESET) return APP_ERROR;

    auto *header = reinterpret_cast<SPkgHeader*>(txBuffer_);
    auto *frame = reinterpret_cast<STxDataFrame*>(txBuffer_ + sizeof(SPkgHeader));

    // 1. 序列化数据体
    // 注意：直接从 frame->data 开始，不偏移 sizeof(SPkgHeader)
    // 剩余可用空间为 sizeof(STxDataFrame::data) = 298
    pb_ostream_t stream = pb_ostream_from_buffer(frame->data, sizeof(frame->data));
    
    if (!pb_encode(&stream, EngineerDataPacketToClient_fields, &message)) {
        return APP_ERROR;
    }
    frame->len = static_cast<uint16_t>(stream.bytes_written);

    // 2. 填充帧头 (A5 帧头)
    header->SOF = 0xA5;
    header->pkgLen = sizeof(STxDataFrame); // 固定为 300
    header->cmd_Id = tx_cmd_id;
    header->seq++;
    // CRC8 校验前 4 字节
    header->CRC8 = CCrcValidator::Crc8Calculate(txBuffer_, 4);

    // 3. 填充帧尾 CRC16 (校验除去最后 2 字节的所有内容)
    uint16_t crc16 = CCrcValidator::Crc16Calculate(txBuffer_, totalPkgLen - 2);
    memcpy(txBuffer_ + totalPkgLen - 2, &crc16, 2);

    // 4. 发送 (全长 309 字节)
    return uartInterface_->Transmit(txBuffer_, totalPkgLen);
}

} // namespace my_engineer
