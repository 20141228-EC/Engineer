/******************************************************************************
 * @brief        
 * 
 * @file         com_pitch_end.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-03-30
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "mod_controller.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    初始化末端Pitch电机模块
 ******************************************************************************/
EAppStatus CModController::CComPitchEnd::InitComponent(SModInitParam_Base &param) {
    // 检查param是否正确
    if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    // 类型转换
    auto controllerParam = static_cast<SModInitParam_Controller &>(param);

    // 保存电机指针
    motor[0] = MotorIDMap.at(controllerParam.pitch_end_id);

    // 设置发送节点
    mtrCanTxNode_[0] = controllerParam.pitchEndTxNode;

    // 初始化PID控制器
    controllerParam.pitchEndPosPidParam.threadNum = 1;
    pidPosCtrl.InitPID(&controllerParam.pitchEndPosPidParam);

    controllerParam.pitchEndSpdPidParam.threadNum = 1;
    pidSpdCtrl.InitPID(&controllerParam.pitchEndSpdPidParam);

    //初始化电机数据输出缓冲区
    mtrOutputBuffer.fill(0);

    Component_FSMFlag_ = FSM_RESET;
    componentStatus = APP_OK;

    return APP_OK;
}

/******************************************************************************
 * @brief    更新组件
 ******************************************************************************/
EAppStatus CModController::CComPitchEnd::UpdateComponent() {
    // 检查组件状态
    if (componentStatus == APP_RESET) return APP_ERROR;

    // 更新组件信息
    pitchEndInfo.posit = motor[0]->motorData[CDevMtr::DATA_POSIT] * -1;
    pitchEndInfo.isPositArrived = (abs(pitchEndCmd.setPosit - pitchEndInfo.posit) < 8192 * 0.02);

    switch (Component_FSMFlag_) {
        case FSM_RESET: {
            StopComponent();
            mtrOutputBuffer.fill(0);
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            componentStatus = APP_OK;
            return APP_OK;
        }

        case FSM_PREINIT: {
            pitchEndCmd.setPosit = static_cast<int32_t>(rangeLimit * 1.2);
            motor[0]->motorData[CDevMtr::DATA_POSIT] = pitchEndCmd.setPosit;
            mtrOutputBuffer.fill(0);
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT: {
            if (motor[0]->motorStatus == CDevMtr::EMotorStatus::STALL) {
                pitchEndCmd = SPitchEndCmd();
                motor[0]->motorData[CDevMtr::DATA_POSIT] =  CONTROLLER_PITCH_END_MOTOR_OFFSET;
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                Component_FSMFlag_ = FSM_CTRL;
                componentStatus = APP_OK;
                return APP_OK;
            }
            pitchEndCmd.setPosit -= 100;
            return _UpdateOutput(static_cast<float_t>(pitchEndCmd.setPosit));
        }

        case FSM_CTRL: {
            pitchEndCmd.setPosit = std::clamp(pitchEndCmd.setPosit, static_cast<int32_t>(0), rangeLimit);
            if (pitchEndCmd.isFree) {
                mtrOutputBuffer.fill(0);
                return APP_OK;
            }
            return _UpdateOutput(static_cast<float_t>(pitchEndCmd.setPosit));
        }

        default: {
            StopComponent();
            mtrOutputBuffer.fill(0);
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            componentStatus = APP_ERROR;
            return APP_ERROR;
        }
    }
}

/******************************************************************************
 * @brief    物理位置转换为电机位置
 * 
 * @param    phyPosit 
 * @return   int32_t 
 ******************************************************************************/
int32_t CModController::CComPitchEnd::PhyPositToMtrPosit(float_t phyPosit) {
    const int32_t zeroOffset = 0;
    const float_t scale = 22.76f;

    return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}

/******************************************************************************
 * @brief    电机位置转换为物理位置
 * 
 * @param    mtrPosit 
 * @return   float_t 
 ******************************************************************************/
float_t CModController::CComPitchEnd::MtrPositToPhyPosit(int32_t mtrPosit) {
    const int32_t zeroOffset = 0;
    const float_t scale = 22.76f;

    return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}

/******************************************************************************
 * @brief    输出更新函数
 ******************************************************************************/
EAppStatus CModController::CComPitchEnd::_UpdateOutput(float_t posit) {
    // 位置环
    DataBuffer<float_t> pitchEndPos = {
        static_cast<float_t>(posit * -1),
    };

    DataBuffer<float_t> pitchEndPosMeasure = {
        static_cast<float_t>(motor[0]->motorData[CDevMtr::DATA_POSIT]),
    };

    auto pitchEndSpd = 
        pidPosCtrl.UpdatePidController(pitchEndPos, pitchEndPosMeasure);

    // 速度环
    DataBuffer<float_t> pitchEndSpdMeasure = {
        static_cast<float_t>(motor[0]->motorData[CDevMtr::DATA_SPEED]),
    };

    auto output = 
        pidSpdCtrl.UpdatePidController(pitchEndSpd, pitchEndSpdMeasure);

    mtrOutputBuffer = {
        static_cast<int16_t>(output[0]),
    };

    return APP_OK;
}

} // namespace my_engineer

