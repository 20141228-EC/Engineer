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
    rollInfo.isPositArrived = (fabs(rollCmd.setParam[EMotorParam::POSIT] - rollInfo.posit) < 5.0f);

    // 一阶低通滤波
    // filteredPosit 独立于 setParam[POSIT]，避免目标值被覆盖
    static float_t filteredPosit = 0.0f;
    constexpr float_t LPF_ALPHA = 0.005f;   // 滤波系数
    constexpr float_t LPF_MIN   = 0.03f;    // 最小分辨率

    filteredPosit += (rollCmd.setParam[EMotorParam::POSIT] - filteredPosit) * LPF_ALPHA;
    if (fabs(rollCmd.setParam[EMotorParam::POSIT] - filteredPosit) < LPF_MIN) {
        filteredPosit = rollCmd.setParam[EMotorParam::POSIT];
    }

    switch (Component_FSMFlag_) {
        case FSM_RESET: {
            std::fill(std::begin(rollCmd.setParam), std::end(rollCmd.setParam), 0.0f);
            filteredPosit = 0.0f;
            return APP_OK;
        }

        case FSM_PREINIT: {
            // 初始化滤波器到当前实际位置，避免初始化时跳变
            filteredPosit = rollInfo.posit;
            rollCmd.setParam[EMotorParam::POSIT] = rollInfo.posit;
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT: {
            // 初始化阶段：移动到初始位置
            rollCmd.setParam[EMotorParam::POSIT] = 0.0f;  // 设置目标（不会被滤波覆盖）
            rollCmd.setParam[EMotorParam::KP] = motor[0]->Kp;
            rollCmd.setParam[EMotorParam::KD] = motor[0]->Kd;
            if (rollInfo.isPositArrived) {
                Component_FSMFlag_ = FSM_CTRL;
                componentStatus = APP_OK;
            }
            // 使用滤波后的位置发送（参考主控板 next_angle）
            motor[0]->Control_MIT(
                rollCmd.setParam[static_cast<int>(EMotorParam::KP)],
                rollCmd.setParam[static_cast<int>(EMotorParam::KD)],
                OffsetPositToMotortruePosit(filteredPosit),
                rollCmd.setParam[static_cast<int>(EMotorParam::SPEED)],
                rollCmd.setParam[static_cast<int>(EMotorParam::TF)]);
            return APP_OK;
        }

        case FSM_CTRL: {
            if (rollCmd.isFree) {
                // 示教模式：无位置刚度，TF由UpdateGravityComp_设置（虚拟阻尼）
                rollCmd.setParam[EMotorParam::KP] = 0.0f;     // 无位置刚度
                rollCmd.setParam[EMotorParam::KD] = 0.0f;     // 无速度刚度（阻尼由软件TF实现）
                filteredPosit = rollInfo.posit;  // 同步滤波器，确保切换回位控时不跳变
                rollCmd.setParam[EMotorParam::POSIT] = rollInfo.posit;  // 同步目标
                return _UpdateOutput(rollCmd.setParam);
            }

            // 位控模式：使用滤波后的位置
            rollCmd.setParam[EMotorParam::KP] = motor[0]->Kp;
            rollCmd.setParam[EMotorParam::KD] = motor[0]->Kd;
            motor[0]->Control_MIT(
                rollCmd.setParam[static_cast<int>(EMotorParam::KP)],
                rollCmd.setParam[static_cast<int>(EMotorParam::KD)],
                OffsetPositToMotortruePosit(filteredPosit),
                rollCmd.setParam[static_cast<int>(EMotorParam::SPEED)],
                rollCmd.setParam[static_cast<int>(EMotorParam::TF)]);
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
    // 注意：这里不使用重力补偿偏置，只做单位转换（度->弧度）
    return (static_cast<float_t>(offsetPosit) / scale);
}

/******************************************************************************
 * @brief    电机位置转换为物理位置（MIT模式角度转换）
 *
 * @param    motortruePosit
 * @return   float_t
 ******************************************************************************/
float_t CModController::CComRoll::MotortruePositToOffsetPosit(float_t motortruePosit) {
    const float_t scale = 180.0f / PI;
    // 注意：这里不使用重力补偿偏置，只做单位转换（弧度->度）
    return (static_cast<float_t>(motortruePosit * scale));
}

/******************************************************************************
 * @brief    输出更新函数（MIT模式）
 ******************************************************************************/
EAppStatus CModController::CComRoll::_UpdateOutput(float_t* setParam){
    float_t posit = OffsetPositToMotortruePosit(setParam[static_cast<int>(EMotorParam::POSIT)]);
    float_t torq = setParam[static_cast<int>(EMotorParam::TF)];

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
