/**
 * @file com_wheelset.cpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 底盘轮组
 * @version 1.0
 * @date 2024-11-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "mod_chassis.hpp"
#include "RTT_DEBUG.h"
#include <cmath>

namespace my_engineer {

// 舵轮零位补偿和方向（如需反向可将1改为-1）
constexpr int STEER_MECH_MID[4] = {600, 7302, 4700, 7450}; // LF, RF, LB, RB
constexpr int STEER_DIR[4]      = {1, 1, 1, 1};

CMemsBase *pmems_wheel_test = nullptr;


/**
 * @brief 初始化底盘轮组组件
 * 
 * @param param
 * @return EAppStatus 
 */
EAppStatus CModChassis::CComWheelset::InitComponent(SModInitParam_Base &param){
    // 检查param是否正确
    if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    // 类型转换
    auto chassisParam = static_cast<SModInitParam_Chassis &>(param);

    // 保存电机和传感器指针
    mems = MemsIDMap.at(chassisParam.memsDevID);
    motor[LF] = MotorIDMap.at(chassisParam.wheelsetMotorID_LF);
    motor[RF] = MotorIDMap.at(chassisParam.wheelsetMotorID_RF);
    motor[LB] = MotorIDMap.at(chassisParam.wheelsetMotorID_LB);
    motor[RB] = MotorIDMap.at(chassisParam.wheelsetMotorID_RB);

    if (chassisParam.steerMotorID_LF != EDeviceID::DEV_NULL) {
        steerMotor[LF] = MotorIDMap.at(chassisParam.steerMotorID_LF);
    }
    if (chassisParam.steerMotorID_RF != EDeviceID::DEV_NULL) {
        steerMotor[RF] = MotorIDMap.at(chassisParam.steerMotorID_RF);
    }
    if (chassisParam.steerMotorID_LB != EDeviceID::DEV_NULL) {
        steerMotor[LB] = MotorIDMap.at(chassisParam.steerMotorID_LB);
    }
    if (chassisParam.steerMotorID_RB != EDeviceID::DEV_NULL) {
        steerMotor[RB] = MotorIDMap.at(chassisParam.steerMotorID_RB);
    }

    // 设置发送节点
    mtrCanTxNode[LF] = chassisParam.wheelsetMotorTxNode_LF;
    mtrCanTxNode[RF] = chassisParam.wheelsetMotorTxNode_RF;
    mtrCanTxNode[LB] = chassisParam.wheelsetMotorTxNode_LB;
    mtrCanTxNode[RB] = chassisParam.wheelsetMotorTxNode_RB;

    mtrSteerCanTxNode[LF] = chassisParam.steerMotorTxNode_LF;
    mtrSteerCanTxNode[RF] = chassisParam.steerMotorTxNode_RF;
    mtrSteerCanTxNode[LB] = chassisParam.steerMotorTxNode_LB;
    mtrSteerCanTxNode[RB] = chassisParam.steerMotorTxNode_RB;

    // 初始化PID控制器
    for (int i = 0; i < 4; i++) {
        chassisParam.wheelsetSpdPidParam[i].threadNum = 1;
        pidSpdCtrl[i].InitPID(&chassisParam.wheelsetSpdPidParam[i]);

        chassisParam.steerPosPidParam[i].threadNum = 1;
        pidSteerPosCtrl[i].InitPID(&chassisParam.steerPosPidParam[i]);

        chassisParam.steerSpdPidParam[i].threadNum = 1;
        pidSteerSpdCtrl[i].InitPID(&chassisParam.steerSpdPidParam[i]);
    }

    chassisParam.lineCorrectionPidParam.threadNum = 3;
    pidLineCorrectionCtrl.InitPID(&chassisParam.lineCorrectionPidParam);

    chassisParam.yawCorrectionPidParam.threadNum = 1;
    pidYawCtrl.InitPID(&chassisParam.yawCorrectionPidParam);

    // test
    pmems_wheel_test = mems;

    mems->StartDevice();

    // 初始化电机数据输出缓冲区
    mtrOutputBuffer.fill(0);
    mtrSteerOutputBuffer.fill(0);

    enableSwerve = (steerMotor[LF] != nullptr && steerMotor[RF] != nullptr &&
                    steerMotor[LB] != nullptr && steerMotor[RB] != nullptr &&
                    mtrSteerCanTxNode[LF] != nullptr && mtrSteerCanTxNode[RF] != nullptr &&
                    mtrSteerCanTxNode[LB] != nullptr && mtrSteerCanTxNode[RB] != nullptr);

    if (!enableSwerve) {
        componentStatus = APP_ERROR;
        return APP_ERROR;
    }

    Component_FSMFlag_ = FSM_RESET;
    componentStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 更新组件
 * 
 */
EAppStatus CModChassis::CComWheelset::UpdateComponent(){
    // 检查组件状态
    if (componentStatus == APP_RESET) return APP_ERROR;

    switch (Component_FSMFlag_)
    {
        case FSM_RESET: {
            // 重置状态下，电机数据输出缓冲区始终为0
            mtrOutputBuffer.fill(0);
            mtrSteerOutputBuffer.fill(0);
            steerErrRad.fill(0.0f);
            return APP_OK;
        }
        case FSM_PREINIT: {
            // 预初始化状态下，清空电机的position和输出缓冲区，并重置PID控制器
            motor[LF]->motorData[CDevMtr::DATA_POSIT] = 0;
            motor[RF]->motorData[CDevMtr::DATA_POSIT] = 0;
            motor[LB]->motorData[CDevMtr::DATA_POSIT] = 0;
            motor[RB]->motorData[CDevMtr::DATA_POSIT] = 0;

            steerMotor[LF]->motorData[CDevMtr::DATA_POSIT] = 0;
            steerMotor[RF]->motorData[CDevMtr::DATA_POSIT] = 0;
            steerMotor[LB]->motorData[CDevMtr::DATA_POSIT] = 0;
            steerMotor[RB]->motorData[CDevMtr::DATA_POSIT] = 0;

            for (auto &pid : pidSpdCtrl) {
                pid.ResetPidController();
            }

            for (auto &pid : pidSteerPosCtrl) {
                pid.ResetPidController();
            }

            for (auto &pid : pidSteerSpdCtrl) {
                pid.ResetPidController();
            }

            pidYawCtrl.ResetPidController();
            Component_FSMFlag_ = FSM_INIT;
        }
        case FSM_INIT: {
            if(mems->memsStatus == CMemsBase::EMemsStatus::NORMAL) {
                wheelsetCmd = SWheelsetCommand();
                Component_FSMFlag_ = FSM_CTRL;
                componentStatus = APP_OK;
            }
            return _UpdateOutput(wheelsetCmd.speed_X, wheelsetCmd.speed_Y, wheelsetCmd.speed_W);
        }
        case FSM_CTRL: {
            DataBuffer<float_t> yawSpd = {wheelsetCmd.speed_W / 10.0f};
            DataBuffer<float_t> yawSpdMeasure = {mems->memsData[CMemsBase::DATA_GYRO_Z]};

            // 底盘角速度是一个双环控制，外环输入为目标真实角速度，输出一个映射到电机速度的目标速度
            auto output_yaw = pidYawCtrl.UpdatePidController(yawSpd, yawSpdMeasure);

            return _UpdateOutput(wheelsetCmd.speed_X, wheelsetCmd.speed_Y, output_yaw[0]);
            
        }
    
    
        default: {
            Component_FSMFlag_ = FSM_RESET;
            return APP_ERROR;
        }
    }
}

/**
 * @brief 更新输出（舵轮解算）
 * @param speed_X 
 * @param speed_Y 
 * @param speed_W 
 * @return EAppStatus 
 */
EAppStatus CModChassis::CComWheelset::_UpdateOutput(float speed_X, float speed_Y, float speed_W){

    // 从底盘电机中读取当前速度存入缓冲区
    DataBuffer<float_t> wheelSpdMeasure = {
        static_cast<float_t>(motor[LF]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float_t>(motor[RF]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float_t>(motor[LB]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float_t>(motor[RB]->motorData[CDevMtr::DATA_SPEED]),
    };
    DataBuffer<float_t> steerSpdMeasure = {
        static_cast<float_t>(steerMotor[LF]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float_t>(steerMotor[RF]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float_t>(steerMotor[LB]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float_t>(steerMotor[RB]->motorData[CDevMtr::DATA_SPEED]),
    };



    int32_t steerErrDbg[4] = {0, 0, 0, 0};
    float steerRawOutDbg[4] = {0, 0, 0, 0};
    static float steerSpdTargetFilt[4] = {0, 0, 0, 0};
    static int32_t stopHoldSteerEcd[4] = {0, 0, 0, 0};
    static bool lastIsStopCmd = false;

    auto normAngle = [](float angle) {
        while (angle > kPi) angle -= kTwoPi;
        while (angle < -kPi) angle += kTwoPi;
        return angle;
    };

    const float vx = speed_X;
    const float vy = speed_Y;
    const float vw = -speed_W;
    const bool isStopCmd = (std::fabs(vx) < 1e-4f && std::fabs(vy) < 1e-4f && std::fabs(vw) < 1e-4f);

    // 几何约定：X向右，Y向前，W逆时针为正；轮序：LF, RF, LB, RB
    float targetVx[4] = {
        vx - vw * SIN_45,
        vx - vw * SIN_45,
        vx + vw * SIN_45,
        vx + vw * SIN_45,
    };
    float targetVy[4] = {
        vy - vw * COS_45,
        vy + vw * COS_45,
        vy - vw * COS_45,
        vy + vw * COS_45,
    };

    if (!isStopCmd) {
        float maxWheelVecMag = 0.0f;
        for (int i = 0; i < 4; i++) {
            const float wheelVecMag = std::sqrt(targetVx[i] * targetVx[i] + targetVy[i] * targetVy[i]);
            if (wheelVecMag > maxWheelVecMag) {
                maxWheelVecMag = wheelVecMag;
            }
        }

        const float cmdVecMag = std::sqrt(vx * vx + vy * vy + vw * vw);
        if (maxWheelVecMag > cmdVecMag + 1e-4f && cmdVecMag > 1e-4f) {
            const float normScale = cmdVecMag / maxWheelVecMag;
            for (int i = 0; i < 4; i++) {
                targetVx[i] *= normScale;
                targetVy[i] *= normScale;
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        // 获取当前电机编码值
        int32_t rawSteerEcd = static_cast<int32_t>(steerMotor[i]->motorData[CDevMtr::DATA_ANGLE]) % ECD_CYCLE;
        if (rawSteerEcd < 0) rawSteerEcd += ECD_CYCLE;
        int32_t currentSteerEcd = rawSteerEcd;
        if (currentSteerEcd < 0) currentSteerEcd += ECD_CYCLE;
        // 计算目标速度和舵轮角度
        float targetDriveSpeed = std::sqrt(targetVx[i] * targetVx[i] + targetVy[i] * targetVy[i]);
        // 坐标约定：底盘前进方向(+Y)对应舵向0度
        float targetSteerAngleRad = std::atan2(targetVx[i], targetVy[i]);
        int32_t targetSteerEcd = static_cast<int32_t>(std::lround(targetSteerAngleRad * RAD_TO_DJI_ECD + STEER_MECH_MID[i]));
        if (targetSteerEcd < 0) targetSteerEcd += ECD_CYCLE;
        // 停下时锁存当前舵角，不自动回正
        if (isStopCmd) {
            targetDriveSpeed = 0.0f;
            if (!lastIsStopCmd) {
                stopHoldSteerEcd[i] = currentSteerEcd;
            }
            targetSteerEcd = stopHoldSteerEcd[i];
        }

        // 计算编码器误差，并进行轮子翻转优化
        int32_t ecdErr = targetSteerEcd - currentSteerEcd;
        if (ecdErr > ECD_HALF) {
            ecdErr -= ECD_CYCLE;
        } else if (ecdErr < -ECD_HALF) {
            ecdErr += ECD_CYCLE;
        }

        // 增加翻转迟滞，避免舵角误差在90度附近反复跨阈值导致抖动。
        if (!isStopCmd && std::abs(ecdErr) > (ECD_QUARTER + ECD_FLIP_HYST)) {
            targetSteerEcd -= ECD_HALF;
            if (targetSteerEcd < 0) {
                targetSteerEcd += ECD_CYCLE;
            }
            targetDriveSpeed = -targetDriveSpeed;

            ecdErr = targetSteerEcd - currentSteerEcd;
            if (ecdErr > ECD_HALF) {
                ecdErr -= ECD_CYCLE;
            } else if (ecdErr < -ECD_HALF) {
                ecdErr += ECD_CYCLE;
            }
        }
        steerErrDbg[i] = ecdErr;

        const float delta = static_cast<float>(ecdErr) * DJI_ECD_TO_RAD;
        steerErrRad[i] = std::fabs(delta);
        const float alignFactor = std::clamp(std::pow(std::cos(delta), 3.0f), ALIGN_FACTOR_MIN, 1.0f);
        targetDriveSpeed *= alignFactor;

        float currentSteerAngle = static_cast<float>(currentSteerEcd) * DJI_ECD_TO_RAD;
        float targetSteerAngle = static_cast<float>(targetSteerEcd) * DJI_ECD_TO_RAD;
        currentSteerAngle = normAngle(currentSteerAngle);
        targetSteerAngle = normAngle(targetSteerAngle);
        targetSteerAngle = currentSteerAngle + normAngle(targetSteerAngle - currentSteerAngle);

        const float currentSteerDeg = rad2deg(currentSteerAngle);
        switch (i) {
            case LF: steer_angle_lf_deg = currentSteerDeg; break;
            case RF: steer_angle_rf_deg = currentSteerDeg; break;
            case LB: steer_angle_lb_deg = currentSteerDeg; break;
            case RB: steer_angle_rb_deg = currentSteerDeg; break;
            default: break;
        }

        DataBuffer<float_t> driveTarget = {targetDriveSpeed};
        DataBuffer<float_t> driveMeasure = {wheelSpdMeasure[i]};
        float driveOutput = pidSpdCtrl[i].UpdatePidController(driveTarget, driveMeasure)[0];

        DataBuffer<float_t> steerTarget = {targetSteerAngle};
        DataBuffer<float_t> steerMeasure = {currentSteerAngle};
        float steerTargetSpdRad = pidSteerPosCtrl[i].UpdatePidController(steerTarget, steerMeasure)[0];
        float steerTargetSpd = steerTargetSpdRad * RADPS_TO_RPM; // 将目标速度从rad/s转换为RPM
        steerTargetSpd *= STEER_SPD_CMD_GAIN; // 增益调整，提升响应速度
        steerTargetSpd = std::clamp(steerTargetSpd, -STEER_SPD_TGT_LIMIT, STEER_SPD_TGT_LIMIT);

        if (isStopCmd) {
            steerSpdTargetFilt[i] = steerTargetSpd;
        } else {
            steerSpdTargetFilt[i] += STEER_SPD_TGT_FILTER_ALPHA * (steerTargetSpd - steerSpdTargetFilt[i]);
        }

        DataBuffer<float_t> steerSpdTarget = {steerSpdTargetFilt[i]};
        DataBuffer<float_t> steerSpdMea = {steerSpdMeasure[i]};
        float steerOutput = pidSteerSpdCtrl[i].UpdatePidController(steerSpdTarget, steerSpdMea)[0];
        steerRawOutDbg[i] = steerOutput;
        steerOutput *= STEER_DIR[i];
        if (std::fabs(steerOutput) < STEER_CMD_DEADBAND) {
            steerOutput = 0.0f;
        }
        steerOutput = std::clamp(steerOutput, -STEER_CMD_LIMIT, STEER_CMD_LIMIT);

        mtrOutputBuffer[i] = static_cast<int16_t>(driveOutput);
        mtrSteerOutputBuffer[i] = static_cast<int16_t>(steerOutput);
    }

    lastIsStopCmd = isStopCmd;


    return APP_OK;

}


} // namespace my_engineer
