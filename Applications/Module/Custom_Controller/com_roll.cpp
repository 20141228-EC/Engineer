/******************************************************************************
 * @brief        
 * 
 * @file         com_roll.cpp
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
 * @brief    初始化自定义控制器Roll电机模块
 ******************************************************************************/
EAppStatus CModController::CComRoll::InitComponent(SModInitParam_Base &param) {
    // 检查param是否正确
    if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    // 类型转换
    auto controllerParam = static_cast<SModInitParam_Controller &>(param);

    // 保存电机指针
    motor[0] = MotorIDMap.at(controllerParam.roll_id);

    // 设置发送节点
    mtrCanTxNode_[0] = controllerParam.rollTxNode;

    // 初始化PID控制器
    controllerParam.rollPosPidParam.threadNum = 1;
    pidPosCtrl.InitPID(&controllerParam.rollPosPidParam);

    controllerParam.rollSpdPidParam.threadNum = 1;
    pidSpdCtrl.InitPID(&controllerParam.rollSpdPidParam);

    //初始化电机数据输出缓冲区
    mtrOutputBuffer.fill(0);

    Component_FSMFlag_ = FSM_RESET;
    componentStatus = APP_OK;

    return APP_OK;
}

/******************************************************************************
 * @brief    更新组件
 ******************************************************************************/
EAppStatus CModController::CComRoll::UpdateComponent() {
    // 检查组件状态
    if (componentStatus == APP_RESET) return APP_ERROR;

    // 更新组件信息
    rollInfo.posit = motor[0]->motorData[CDevMtr::DATA_POSIT];
    rollInfo.isPositArrived = (abs(rollCmd.setPosit - rollInfo.posit) < 8192 * 0.02);

    switch (Component_FSMFlag_) {
        case FSM_RESET: {
            StopComponent();
            mtrOutputBuffer.fill(0);
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            return APP_OK;
        }

        case FSM_PREINIT: {
            rollCmd.setPosit = static_cast<int32_t>(rangeLimit * 1.2);
            motor[0]->motorData[CDevMtr::DATA_POSIT] = rollCmd.setPosit;
            mtrOutputBuffer.fill(0);
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT: {
            if (motor[0]->motorStatus == CDevMtr::EMotorStatus::STALL) {
                rollCmd = SRollCmd();
                motor[0]->motorData[CDevMtr::DATA_POSIT] = CONTROLLER_ROLL_END_MOTOR_OFFSET;
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                Component_FSMFlag_ = FSM_CTRL;
                componentStatus = APP_OK;
                return APP_OK;
            }
            rollCmd.setPosit -= 100;
            return _UpdateOutput(static_cast<float_t>(rollCmd.setPosit));
        }

        case FSM_CTRL: {
            rollCmd.setPosit = std::clamp(rollCmd.setPosit, static_cast<int32_t>(0), rangeLimit);
            if (rollCmd.isFree) {
                mtrOutputBuffer.fill(0);
                return APP_OK;
            }
            return _UpdateOutput(static_cast<float_t>(rollCmd.setPosit));
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
int32_t CModController::CComRoll::PhyPositToMtrPosit(float_t phyPosit) {
    const int32_t zeroOffset = 0;
    const float_t scale = 22.756f;

    return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}

/******************************************************************************
 * @brief    电机位置转换为物理位置
 * 
 * @param    mtrPosit 
 * @return   float_t 
 ******************************************************************************/
float_t CModController::CComRoll::MtrPositToPhyPosit(int32_t mtrPosit) {
    const int32_t zeroOffset = 0;
    const float_t scale = 22.756f;

    return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}

/******************************************************************************
 * @brief    输出更新函数
 ******************************************************************************/
EAppStatus CModController::CComRoll::_UpdateOutput(float_t posit) {
    // 位置环
    DataBuffer<float_t> rollPos = {
        static_cast<float_t>(posit),
    };

    DataBuffer<float_t> rollPosMeasure = {
        static_cast<float_t>(motor[0]->motorData[CDevMtr::DATA_POSIT]),
    };

    auto rollSpd = 
        pidPosCtrl.UpdatePidController(rollPos, rollPosMeasure);

    // 速度环
    DataBuffer<float_t> rollSpdMeasure = {
        static_cast<float_t>(motor[0]->motorData[CDevMtr::DATA_SPEED]),
    };

    auto output = 
        pidSpdCtrl.UpdatePidController(rollSpd, rollSpdMeasure);

    mtrOutputBuffer = {
        static_cast<int16_t>(output[0]),
    };

    return APP_OK;
}


} // namespace my_engineer
