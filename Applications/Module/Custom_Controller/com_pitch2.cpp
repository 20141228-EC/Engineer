/******************************************************************************
 * @brief
 *
 * @file         com_pitch2.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
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
 * @brief    初始化自定义控制器Pitch2电机模块
 ******************************************************************************/
EAppStatus CModController::CComPitch2::InitComponent(SModInitParam_Base &param){
    // 检查param是否正确
    if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    // 类型转换
    auto controllerParam = static_cast<SModInitParam_Controller &>(param);

    // 保存电机指针
    motor[0] = static_cast<CDevMtrDM*>(MotorIDMap.at(controllerParam.pitch2_id));

    // 设置发送节点
    mtrCanTxNode_[0] = controllerParam.pitch2TxNode;

    // 初始化控制参数
    pitch2Cmd.setParam[EMotorParam::KP] = motor[0]->Kp;   ///< set Kp
    pitch2Cmd.setParam[EMotorParam::KD] = motor[0]->Kd;  ///< set Kd
    pitch2Cmd.setParam[EMotorParam::SPEED] = 0.0f;      ///< Position control means the speed is zero.

    Component_FSMFlag_ = FSM_RESET;
    componentStatus = APP_OK;

    return APP_OK;
}

/******************************************************************************
 * @brief    更新组件
 ******************************************************************************/
EAppStatus CModController::CComPitch2::UpdateComponent() {
    // 检查组件状态
    if (componentStatus == APP_RESET) return APP_ERROR;

    // 更新电机信息
    pitch2Info.posit = MotortruePositToOffsetPosit_test(    ///<注意是在这里更新的示教器控制信息传给机器人，下面的状态机是用来控制自定义控制器的重力补偿的
            CDevMtrDM::uint_to_float(motor[0]->motorData[CDevMtr::DATA_ANGLE], -motor[0]->mitLimit_.Q_MAX, motor[0]->mitLimit_.Q_MAX, 16));
    pitch2Info.isPositArrived = (fabs(pitch2Cmd.setParam[EMotorParam::POSIT] - pitch2Info.posit) < 5.0f);

    switch (Component_FSMFlag_){
        case FSM_RESET: {
            std::fill(std::begin(pitch2Cmd.setParam), std::end(pitch2Cmd.setParam), 0.0f);
            return APP_OK;
        }

        case FSM_PREINIT: {
            float_t params[] = {11.0f, 0.0f, motor[0]->Kp, motor[0]->Kd, 0.0f};
            std::copy(std::begin(params), std::end(params), pitch2Cmd.setParam);
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT: {
            if (pitch2Info.isPositArrived) {
                pitch2Cmd.setParam[static_cast<int>(EMotorParam::POSIT)] = 11.0f;
                Component_FSMFlag_ = FSM_CTRL;
                componentStatus = APP_OK;
                return APP_OK;
            }
            float_t params[] = {11.0f, 0.0f, motor[0]->Kp, motor[0]->Kd, 0.0f};
            std::copy(std::begin(params), std::end(params), pitch2Cmd.setParam);
            return _UpdateOutput(pitch2Cmd.setParam);
        }

        case FSM_CTRL: {
            if (pitch2Cmd.isFree) {
                // 示教模式：使用低阻尼参数\重力补偿前馈
                float_t savedTF = pitch2Cmd.setParam[EMotorParam::TF];
                std::fill(std::begin(pitch2Cmd.setParam), std::end(pitch2Cmd.setParam), 0.0f);
                pitch2Cmd.setParam[EMotorParam::KP] = 0.0f;    // 示教模式位置刚度
                pitch2Cmd.setParam[EMotorParam::KD] = 0.03f;    // 示教模式低阻尼
                pitch2Cmd.setParam[EMotorParam::TF] = savedTF; // 保留重力补偿
                return _UpdateOutput(pitch2Cmd.setParam);
            }
            // 联动模式：恢复位控参数，跟随机器人回传位置
            pitch2Cmd.setParam[EMotorParam::KP] = motor[0]->Kp;
            pitch2Cmd.setParam[EMotorParam::KD] = motor[0]->Kd;
            return _UpdateOutput(pitch2Cmd.setParam);
        }

        default: {
            StopComponent();
            std::fill(std::begin(pitch2Cmd.setParam), std::end(pitch2Cmd.setParam), 0.0f);
            componentStatus = APP_ERROR;
            return APP_ERROR;
        }
    }
}

/******************************************************************************
 * @brief    物理位置转换为电机位置
 *
 * @param    offsetPosit
 * @return   float_t
 ******************************************************************************/
float_t CModController::CComPitch2::OffsetPositToMotortruePosit_test(float_t offsetPosit) {
    const float_t scale = 180.0f / PI;
    return CONTROLLER_PITCH2_MOTOR_DIR * (static_cast<float_t>(offsetPosit) / scale);
}

/******************************************************************************
 * @brief    电机位置转换为物理位置
 *
 * @param    motortruePosit
 * @return   float_t
 ******************************************************************************/
float_t CModController::CComPitch2::MotortruePositToOffsetPosit_test(float_t motortruePosit) {
    const float_t scale = 180.0f / PI;
    return (static_cast<float_t>(CONTROLLER_PITCH2_MOTOR_DIR * motortruePosit * scale));
}

/******************************************************************************
 * @brief    输出更新函数
 ******************************************************************************/
EAppStatus CModController::CComPitch2::_UpdateOutput(float_t* setParam){
  float_t posit = OffsetPositToMotortruePosit_test(setParam[static_cast<int>(EMotorParam::POSIT)]);
  float_t torq = setParam[static_cast<int>(EMotorParam::TF)];

  /* 使用电机内部 TxNode 发送 (CAN重分配后Pitch2独占CAN3，无需降频) */
  motor[0]->Control_MIT(
      setParam[static_cast<int>(EMotorParam::KP)],
      setParam[static_cast<int>(EMotorParam::KD)],
      posit,
      setParam[static_cast<int>(EMotorParam::SPEED)],
      torq);

  return APP_OK;
}

} // namespace my_engineer
