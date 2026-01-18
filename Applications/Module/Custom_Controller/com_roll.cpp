/******************************************************************************
 * @file         com_roll.cpp
 * @author       Fish_Joe (2328339747@qq.com), Ciallo
 * @brief        Roll轴电机组件
 * @version      V1.2
 * @date         2025-03-30
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-17
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#include "mod_controller.hpp"

namespace my_engineer {

/******************************************************************************
 * @brief    初始化自定义控制器Roll电机模块（MIT模式）
 ******************************************************************************/
EAppStatus CModController::CComRoll::InitComponent(SModInitParam_Base &param) {
    // 检查param是否正确
    if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    // 类型转换
    auto controllerParam = static_cast<SModInitParam_Controller &>(param);

    // 保存电机指针（MIT模式使用DM电机）
    motor[0] = static_cast<CDevMtrDM*>(MotorIDMap.at(controllerParam.roll_id));

    // 设置发送节点
    mtrCanTxNode_[0] = controllerParam.rollTxNode;

    // 初始化MIT控制参数
    rollCmd.setParam[EMotorParam::KP] = motor[0]->Kp;   ///< set Kp
    rollCmd.setParam[EMotorParam::KD] = motor[0]->Kd;   ///< set Kd
    rollCmd.setParam[EMotorParam::SPEED] = 0.0f;        ///< Position control means the speed is zero.

    Component_FSMFlag_ = FSM_RESET;
    componentStatus = APP_OK;

    return APP_OK;
}

/******************************************************************************
 * @brief    更新组件（MIT模式）
 ******************************************************************************/
EAppStatus CModController::CComRoll::UpdateComponent() {
    // 检查组件状态
    if (componentStatus == APP_RESET) return APP_ERROR;

    // 更新电机信息
    rollInfo.posit = MotortruePositToOffsetPosit(    ///<注意是在这里更新的示教器控制信息传给机器人，下面的状态机是用来控制自定义控制器的重力补偿的
            CDevMtrDM::uint_to_float(motor[0]->motorData[CDevMtr::DATA_ANGLE], Pos_MIN, Pos_MAX, 16));
    rollInfo.isPositArrived = (fabs(rollCmd.setParam[EMotorParam::POSIT] - rollInfo.posit) < 5.0f);

    switch (Component_FSMFlag_) {
        case FSM_RESET: {
            std::fill(std::begin(rollCmd.setParam), std::end(rollCmd.setParam), 0.0f);
            return APP_OK;
        }

        case FSM_PREINIT: {
            float_t params[] = {0.0f, 0.0f, motor[0]->Kp, motor[0]->Kd, 0.0f};
            std::copy(std::begin(params), std::end(params), rollCmd.setParam);
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT: {
            if (rollInfo.isPositArrived) {
                rollCmd.setParam[static_cast<int>(EMotorParam::POSIT)] = 0.0f;
                Component_FSMFlag_ = FSM_CTRL;
                componentStatus = APP_OK;
                return APP_OK;
            }
            float_t params[] = {0.0f, 0.0f, motor[0]->Kp, motor[0]->Kd, 0.0f};
            std::copy(std::begin(params), std::end(params), rollCmd.setParam);
            return _UpdateOutput(rollCmd.setParam);
        }

        case FSM_CTRL: {
            if (rollCmd.isFree) {
                std::fill(std::begin(rollCmd.setParam), std::end(rollCmd.setParam), 0.0f);
                return _UpdateOutput(rollCmd.setParam);
            }
            return _UpdateOutput(rollCmd.setParam);
        }

        default: {
            StopComponent();
            std::fill(std::begin(rollCmd.setParam), std::end(rollCmd.setParam), 0.0f);
            componentStatus = APP_ERROR;
            return APP_ERROR;
        }
    }
}

/******************************************************************************
 * @brief    物理位置转换为电机位置（MIT模式角度转换）
 *
 * @param    offsetPosit
 * @return   float_t
 ******************************************************************************/
float_t CModController::CComRoll::OffsetPositToMotortruePosit(float_t offsetPosit) {
    const float_t zeroOffset = 0.0f;
    const float_t scale = 180.0f / PI;
    return (static_cast<float_t>(offsetPosit - zeroOffset) / scale);
}

/******************************************************************************
 * @brief    电机位置转换为物理位置（MIT模式角度转换）
 *
 * @param    motortruePosit
 * @return   float_t
 ******************************************************************************/
float_t CModController::CComRoll::MotortruePositToOffsetPosit(float_t motortruePosit) {
    const float_t zeroOffset = 0.0f;
    const float_t scale = 180.0f / PI;

    return (static_cast<float_t>(motortruePosit * scale) + zeroOffset);
}

/******************************************************************************
 * @brief    输出更新函数（MIT模式）
 ******************************************************************************/
EAppStatus CModController::CComRoll::_UpdateOutput(float_t* setParam){
    float_t posit = OffsetPositToMotortruePosit(setParam[static_cast<int>(EMotorParam::POSIT)]);
    float_t torq = setParam[static_cast<int>(EMotorParam::TF)];

    CDevMtrDM::FillCanTxBuffer_MIT(motor[0], mtrCanTxNode_[0]->dataBuffer,
                                posit, setParam[static_cast<int>(EMotorParam::SPEED)],
                                torq, setParam[static_cast<int>(EMotorParam::KP)],
                                setParam[static_cast<int>(EMotorParam::KD)]);

    /*----------- CAN发送 -----------*/
    mtrCanTxNode_[0]->Transmit();
    return APP_OK;
}

} // namespace my_engineer
