/******************************************************************************
 * @file         com_roll.cpp
 * @author       Fish_Joe (2328339747@qq.com), Ciallo
 * @brief        Roll轴电机组件
 * @version      V1.2
 * @date         2025-03-30
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-22
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#include "mod_controller.hpp"

extern "C" {
extern volatile int dbg_position_hold_mode;
extern volatile float dbg_hold_kp;
extern volatile float dbg_hold_kd;
extern volatile float dbg_target_roll;
}

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
            CDevMtrDM::uint_to_float(motor[0]->motorData[CDevMtr::DATA_ANGLE], -motor[0]->mitLimit_.Q_MAX, motor[0]->mitLimit_.Q_MAX, 16));
    rollInfo.isPositArrived = (fabs(rollCmd.setParam[EMotorParam::POSIT] - rollInfo.posit) < 8.0f);

    uint8_t setZero_flag = 0;
    if(setZero_flag == 1) {
        motor[0]->SetZero();  ///<测试用，将当前角度设为零点
    }

    switch (Component_FSMFlag_) {
        case FSM_RESET: {
            std::fill(std::begin(rollCmd.setParam), std::end(rollCmd.setParam), 0.0f);
            return APP_OK;
        }

        case FSM_PREINIT: {
            rollCmd.setParam[EMotorParam::POSIT] = rollInfo.posit;
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT: {
            // 初始化阶段：移动到初始位置
            rollCmd.setParam[EMotorParam::POSIT] = 0.0f;  // 设置目标
            rollCmd.setParam[EMotorParam::KP] = motor[0]->Kp;
            rollCmd.setParam[EMotorParam::KD] = motor[0]->Kd;
            if (rollInfo.isPositArrived) {
                Component_FSMFlag_ = FSM_CTRL;
                componentStatus = APP_OK;
            }
            motor[0]->Control_MIT(
                rollCmd.setParam[static_cast<int>(EMotorParam::KP)],
                rollCmd.setParam[static_cast<int>(EMotorParam::KD)],
                OffsetPositToMotortruePosit(rollCmd.setParam[static_cast<int>(EMotorParam::POSIT)]),
                rollCmd.setParam[static_cast<int>(EMotorParam::SPEED)],
                rollCmd.setParam[static_cast<int>(EMotorParam::TF)]);
            return APP_OK;
        }

        case FSM_CTRL: {
            if (rollCmd.isFree) {
                // 示教模式
                rollCmd.setParam[EMotorParam::KP] = 0.0f;     // 无位置刚度
                rollCmd.setParam[EMotorParam::KD] = 0.0f;     // 无速度刚度
                if (dbg_position_hold_mode) {
                    // 辨识模式: MIT位置环锁定目标角, 供采保持力矩
                    rollCmd.setParam[EMotorParam::KP] = motor[0]->Kp;
                    rollCmd.setParam[EMotorParam::KD] = motor[0]->Kd;
                    rollCmd.setParam[EMotorParam::POSIT] = dbg_target_roll;
                } else {
                    rollCmd.setParam[EMotorParam::POSIT] = rollInfo.posit;  // 同步目标
                }
                return _UpdateOutput(rollCmd.setParam);
            }

            // 位控模式, grav_ff 在位控模式下为0
            rollCmd.setParam[EMotorParam::KP] = motor[0]->Kp;
            rollCmd.setParam[EMotorParam::KD] = motor[0]->Kd;
            motor[0]->Control_MIT(
                rollCmd.setParam[static_cast<int>(EMotorParam::KP)],
                rollCmd.setParam[static_cast<int>(EMotorParam::KD)],
                OffsetPositToMotortruePosit(rollCmd.setParam[static_cast<int>(EMotorParam::POSIT)]),
                rollCmd.setParam[static_cast<int>(EMotorParam::SPEED)],
                0);
            return APP_OK;
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
    const float_t scale = 180.0f / PI;
    return CONTROLLER_ROLL_MOTOR_DIR * (static_cast<float_t>(offsetPosit) / scale);
}

/******************************************************************************
 * @brief    电机位置转换为物理位置（MIT模式角度转换）
 *
 * @param    motortruePosit
 * @return   float_t
 ******************************************************************************/
float_t CModController::CComRoll::MotortruePositToOffsetPosit(float_t motortruePosit) {
    const float_t scale = 180.0f / PI;
    return CONTROLLER_ROLL_MOTOR_DIR * (static_cast<float_t>(motortruePosit * scale));
}

/******************************************************************************
 * @brief    输出更新函数（MIT模式）
 ******************************************************************************/
EAppStatus CModController::CComRoll::_UpdateOutput(float_t* setParam){
    float_t posit = OffsetPositToMotortruePosit(setParam[static_cast<int>(EMotorParam::POSIT)]);
    // TF 由模块层 UpdateGravityComp_ 写入 grav_ff 成员
    float_t torq = this->grav_ff;

    /* 使用电机内部 TxNode 发送 */
    motor[0]->Control_MIT(
        setParam[static_cast<int>(EMotorParam::KP)],
        setParam[static_cast<int>(EMotorParam::KD)],
        posit,
        setParam[static_cast<int>(EMotorParam::SPEED)],
        torq);
    return APP_OK;
}

} // namespace my_engineer
