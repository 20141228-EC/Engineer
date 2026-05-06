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
constexpr int STEER_MECH_MID[4] = {6726, 5421, 6710, 1357}; // LF, RF, LB, RB
    // -668 -53 -1946 2815
constexpr int STEER_DIR[4]      = {1, 1, 1, 1};
uint8_t steer_error_dir = 1;
float_t steerPosTarget_debug[4] = {0};
float_t debug_ = 0.f;

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
            return APP_OK;
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
    DataBuffer<float_t> steerPosMeasure = {
        static_cast<float_t>(steerMotor[LF]->motorData[CDevMtr::DATA_ANGLE]),
        static_cast<float_t>(steerMotor[RF]->motorData[CDevMtr::DATA_ANGLE]),
        static_cast<float_t>(steerMotor[LB]->motorData[CDevMtr::DATA_ANGLE]),
        static_cast<float_t>(steerMotor[RB]->motorData[CDevMtr::DATA_ANGLE]),
    };

    // 几何约定：X向右，Y向前，W逆时针为正；轮序：LF, RF, LB, RB
    // 计算每个轮子的 X/Y 速度分量
		float sqrt2_2 = 0.70710678;
		
    float vx1 = (float)speed_Y - (float)speed_W*sqrt2_2;
    float vy1 = (float)speed_X + (float)speed_W*sqrt2_2;
    float vx2 = (float)speed_Y + (float)speed_W*sqrt2_2;
    float vy2 = (float)speed_X + (float)speed_W*sqrt2_2;
    float vx3 = (float)speed_Y - (float)speed_W*sqrt2_2;
    float vy3 = (float)speed_X - (float)speed_W*sqrt2_2;
    float vx4 = (float)speed_Y + (float)speed_W*sqrt2_2;
    float vy4 = (float)speed_X - (float)speed_W*sqrt2_2;

    arm_atan2_f32(vy2, vx2, &debug_);
		debug_ = debug_ / 3.1415926f * 4096.f;

    // 计算目标舵角
    DataBuffer<float_t> steerPosTarget = {
        atan2(vy1, vx1) / 3.1415926f * 4096.f,
        atan2(vy2, vx2) / 3.1415926f * 4096.f,
        atan2(vy3, vx3) / 3.1415926f * 4096.f,
        atan2(vy4, vx4) / 3.1415926f * 4096.f,
    };

    steerPosTarget_debug[0] = steerPosTarget[0];
    steerPosTarget_debug[1] = steerPosTarget[1];
    steerPosTarget_debug[2] = steerPosTarget[2];
    steerPosTarget_debug[3] = steerPosTarget[3];

    // 计算舵向当前相对角度 [-4096, 4096]
    DataBuffer<float_t> steerPosRelative = {
        steerPosMeasure[LF] - STEER_MECH_MID[LF],
        steerPosMeasure[RF] - STEER_MECH_MID[RF],
        steerPosMeasure[LB] - STEER_MECH_MID[LB],
        steerPosMeasure[RB] - STEER_MECH_MID[RB],
    };
    for(int i = 0; i < 4; i++) {
        if (steerPosRelative[i] > 4096) steerPosRelative[i] -= 8192;
        if (steerPosRelative[i] < -4096) steerPosRelative[i] += 8192;
    }

    // 计算目标轮速
    DataBuffer<float_t> SpdTarget = {
        sqrtf(vx1*vx1 + vy1*vy1),
        sqrtf(vx2*vx2 + vy2*vy2),
        sqrtf(vx3*vx3 + vy3*vy3),
        sqrtf(vx4*vx4 + vy4*vy4),
    };

    // 半圈处理
    for(int i = 0; i < 4; i++){
    
        float err = steerPosTarget[i] - steerPosRelative[i];
        
        // 半圈处理，保证误差在[-4096, 4096]之间
        if (err > 4096) err -= 8192;
        else if (err < -4096) err += 8192;

        if (err > 2048) {
            steerPosTarget[i] -= 4096;
            SpdTarget[i] *= static_cast<float_t>(pow(cos(err * PI / 4096),11));  // 轮速反向
        }
        else if (err < -2048) {
            steerPosTarget[i] += 4096;
            SpdTarget[i] *= static_cast<float_t>(pow(cos(err * PI / 4096),11));  // 轮速反向
        }

        // 规范化target到[-4096, 4096]
        if (steerPosTarget[i] > 4096) steerPosTarget[i] -= 8192;
        else if (steerPosTarget[i] < -4096) steerPosTarget[i] += 8192;
    }

    // 计算轮毂输出
    DataBuffer<float_t> output_wheelset(4,0.0f);
    for (int i = 0; i < 4; i++) {
        DataBuffer<float_t> wheelSpd_i = {SpdTarget[i]};    // 目标值
        DataBuffer<float_t> wheelSpdMeasure_i = {wheelSpdMeasure[i]};   // 测量值
        output_wheelset[i] = pidSpdCtrl[i].UpdatePidController(wheelSpd_i, wheelSpdMeasure_i)[0];
    }

    // 计算舵向输出
    DataBuffer<float_t> spd_target(4,0.f);  // 速度目标值
    DataBuffer<float_t> output_steer(4,0.0f);   // 舵向最终输出
    for (int i = 0; i < 4; i++) {
        DataBuffer<float_t> steerPos_i = {steerPosTarget[i] + STEER_MECH_MID[i]};    // 位置目标值
        // DataBuffer<float_t> steerPos_i = {steerPosTarget[i]};    // 位置目标值
        DataBuffer<float_t> steerPosMeasure_i = {steerPosMeasure[i]};   // 位置测量值
        DataBuffer<float_t> steerSpdMeasure_i = {steerSpdMeasure[i]};   // 速度测量值

        spd_target[i] = pidSteerPosCtrl[i].UpdatePidController(steerPos_i, steerPosMeasure_i)[0];

        output_steer[i] = pidSteerSpdCtrl[i].UpdatePidController({spd_target[i]}, steerSpdMeasure_i)[0];
    }

    // 轮毂电机输出
    mtrOutputBuffer = {
        static_cast<int16_t>(output_wheelset[LF]),
        static_cast<int16_t>(output_wheelset[RF]),
        static_cast<int16_t>(output_wheelset[LB]),
        static_cast<int16_t>(output_wheelset[RB]), ///< 轮毂电机输出
    };

    // 舵向电机输出
    mtrSteerOutputBuffer = {
        static_cast<int16_t>(output_steer[LF]),
        static_cast<int16_t>(output_steer[RF]),
        static_cast<int16_t>(output_steer[LB]),
        static_cast<int16_t>(output_steer[RB]), ///< 舵向电机输出
    };

    return APP_OK;

}


} // namespace my_engineer
