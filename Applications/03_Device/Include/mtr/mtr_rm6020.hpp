/**
 * @file mtr_rm6020.hpp
 * @author GitHub Copilot
 * @brief 定义基于DJI电机设备类的RM6020电机设备类
 * @version 1.0
 * @date 2026-03-01
 */

#ifndef MTR_RM6020_HPP
#define MTR_RM6020_HPP

#include "mtr_dji.hpp"

namespace my_engineer {

class CDevMtrRM6020 : public CDevMtrDJI {
private:
    void UpdateHandler_() override {
        if (deviceStatus == APP_RESET) return;

        if (canRxNode_.timestamp >= lastHeartbeatTime_) {
            motorData[DATA_ANGLE]   = (int16_t)(canRxNode_.dataBuffer[0] << 8 | canRxNode_.dataBuffer[1]);
            motorData[DATA_SPEED]   = (int16_t)(canRxNode_.dataBuffer[2] << 8 | canRxNode_.dataBuffer[3]);
            motorData[DATA_CURRENT] = (int16_t)(canRxNode_.dataBuffer[4] << 8 | canRxNode_.dataBuffer[5]);
            motorData[DATA_POSIT]   = (useAngleToPosit_) ? getPosition_() : 0;
            motorData[DATA_TEMP]    = (int8_t)(canRxNode_.dataBuffer[6]);
            motorData[DATA_ERR]     = (int8_t)(canRxNode_.dataBuffer[7]);
            lastHeartbeatTime_ = canRxNode_.timestamp;
        }
    }

public:
    using SMtrInitParam_RM6020 = SMtrInitParam_DJI;

    CDevMtrRM6020() { motorType = EMotorType::MTR_RM6020; }
};

} // namespace my_engineer

#endif // MTR_RM6020_HPP
