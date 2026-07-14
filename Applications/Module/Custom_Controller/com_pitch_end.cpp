/******************************************************************************
 * @file         com_pitch_end.cpp
 * @author       Fish_Joe (2328339747@qq.com), Ciallo
 * @brief        末端Pitch电机组件（MIT模式，DM3510）
 * @version      V1.1
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
extern volatile float dbg_target_pitchEnd;
}

namespace my_engineer {

/******************************************************************************
 * @brief    初始化末端Pitch电机模块（MIT模式）
 ******************************************************************************/
EAppStatus CModController::CComPitchEnd::InitComponent(SModInitParam_Base &param) {
    // 检查param是否正确
    if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    // 类型转换
    auto controllerParam = static_cast<SModInitParam_Controller &>(param);

    // 保存电机指针（MIT模式使用DM电机）
    motor[0] = static_cast<CDevMtrDM*>(MotorIDMap.at(controllerParam.pitch_end_id));

    // 设置发送节点
    mtrCanTxNode_[0] = controllerParam.pitchEndTxNode;

    // 初始化MIT控制参数
    pitchEndCmd.setParam[EMotorParam::KP] = motor[0]->Kp;   ///< set Kp
    pitchEndCmd.setParam[EMotorParam::KD] = motor[0]->Kd;   ///< set Kd
    pitchEndCmd.setParam[EMotorParam::SPEED] = 0.0f;        ///< Position control means the speed is zero.

    Component_FSMFlag_ = FSM_RESET;
    componentStatus = APP_OK;

    return APP_OK;
}

/******************************************************************************
 * @brief    更新组件（MIT模式）
 ******************************************************************************/
EAppStatus CModController::CComPitchEnd::UpdateComponent() {
    // 检查组件状态
    if (componentStatus == APP_RESET) return APP_ERROR;

    // 更新电机信息
    pitchEndInfo.posit = MotortruePositToOffsetPosit(                 ///<注意是在这里更新的示教器控制信息传给机器人，下面的状态机是用来控制自定义控制器的重力补偿的
            CDevMtrDM::uint_to_float(motor[0]->motorData[CDevMtr::DATA_ANGLE], -motor[0]->mitLimit_.Q_MAX, motor[0]->mitLimit_.Q_MAX, 16));
    pitchEndInfo.isPositArrived = (fabs(pitchEndCmd.setParam[EMotorParam::POSIT] - pitchEndInfo.posit) < 8.0f);

    uint8_t setZero_flag = 0;
    if(setZero_flag == 1) {
        motor[0]->SetZero();  ///<测试用，将当前角度设为零点
    }

    switch (Component_FSMFlag_) {
        case FSM_RESET: {
            std::fill(std::begin(pitchEndCmd.setParam), std::end(pitchEndCmd.setParam), 0.0f);
            componentStatus = APP_OK;
            return APP_OK;
        }

        case FSM_PREINIT: {
            pitchEndCmd.setParam[EMotorParam::POSIT] = pitchEndInfo.posit;
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT: {
            // 初始化阶段：移动到初始位置
            pitchEndCmd.setParam[EMotorParam::POSIT] = -11.0f;  // 设置目标
            pitchEndCmd.setParam[EMotorParam::KP] = motor[0]->Kp;
            pitchEndCmd.setParam[EMotorParam::KD] = motor[0]->Kd;
            if (pitchEndInfo.isPositArrived) {
                Component_FSMFlag_ = FSM_CTRL;
                componentStatus = APP_OK;
            }
            motor[0]->Control_MIT(
                pitchEndCmd.setParam[static_cast<int>(EMotorParam::KP)],
                pitchEndCmd.setParam[static_cast<int>(EMotorParam::KD)],
                OffsetPositToMotortruePosit(pitchEndCmd.setParam[static_cast<int>(EMotorParam::POSIT)]),
                pitchEndCmd.setParam[static_cast<int>(EMotorParam::SPEED)],
                pitchEndCmd.setParam[static_cast<int>(EMotorParam::TF)]);
            return APP_OK;
        }

        case FSM_CTRL: {
            if (pitchEndCmd.isFree) {
                // 示教模式：TF(重力补偿+力反馈)由模块层写入grav_ff
                pitchEndCmd.setParam[EMotorParam::KP] = 0.0f;    // 无位置刚度
                pitchEndCmd.setParam[EMotorParam::KD] = 0.0f;    // 无速度刚度
                if (dbg_position_hold_mode) {
                    // 辨识模式: MIT位置环锁定目标角, 供采保持力矩
                    pitchEndCmd.setParam[EMotorParam::KP] = motor[0]->Kp;
                    pitchEndCmd.setParam[EMotorParam::KD] = motor[0]->Kd;
                    pitchEndCmd.setParam[EMotorParam::POSIT] = dbg_target_pitchEnd;
                } else {
                    pitchEndCmd.setParam[EMotorParam::POSIT] = 0;  // 同步目标
                }
                return _UpdateOutput(pitchEndCmd.setParam);
            }

            // 位控模式
            pitchEndCmd.setParam[EMotorParam::KP] = motor[0]->Kp;
            pitchEndCmd.setParam[EMotorParam::KD] = motor[0]->Kd;
            motor[0]->Control_MIT(
                pitchEndCmd.setParam[static_cast<int>(EMotorParam::KP)],
                pitchEndCmd.setParam[static_cast<int>(EMotorParam::KD)],
                OffsetPositToMotortruePosit(pitchEndCmd.setParam[static_cast<int>(EMotorParam::POSIT)]),
                pitchEndCmd.setParam[static_cast<int>(EMotorParam::SPEED)],
                pitchEndCmd.setParam[static_cast<int>(EMotorParam::TF)]);
            return APP_OK;
        }

        default: {
            StopComponent();
            std::fill(std::begin(pitchEndCmd.setParam), std::end(pitchEndCmd.setParam), 0.0f);
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
float_t CModController::CComPitchEnd::OffsetPositToMotortruePosit(float_t offsetPosit) {
    const float_t scale = 180.0f / PI;
    return CONTROLLER_PITCH_END_MOTOR_DIR * (static_cast<float_t>(offsetPosit) / scale);
}

/******************************************************************************
 * @brief    电机位置转换为物理位置（MIT模式角度转换）
 *
 * @param    motortruePosit
 * @return   float_t
 ******************************************************************************/
float_t CModController::CComPitchEnd::MotortruePositToOffsetPosit(float_t motortruePosit) {
    const float_t scale = 180.0f / PI;
    return CONTROLLER_PITCH_END_MOTOR_DIR * (static_cast<float_t>(motortruePosit * scale));
}

/******************************************************************************
 * @brief    输出更新函数（MIT模式）
 ******************************************************************************/
EAppStatus CModController::CComPitchEnd::_UpdateOutput(float_t* setParam){
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
