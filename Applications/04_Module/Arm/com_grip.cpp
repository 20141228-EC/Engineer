/**
 * @file com_grip.cpp
 * @author Ciallo～(∠·ω< )⌒☆(1002046597@qq.com)
 * @brief 机械臂夹爪组件
 * @version 1.4
 * @date 2026-4-25
 *
 * @details V1.4: 控制逻辑解耦,软件堵转检测
 *          V1.3: 双状态机重构（RELEASE/HOLD）
 *          V1.2: 二次夹紧改为闭合方向堵转重新标定
 *          V1.1: 添加Roll轴耦合补偿
 *
 * @copyright Copyright (c) 2025
 *
 */

 #include "mod_arm.hpp"

 namespace my_engineer {

/**
 * @brief 初始化夹爪组件
 */
EAppStatus CModArm::CComGrip::InitComponent(SModInitParam_Base &param) {
    if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    auto armParam = static_cast<SModInitParam_Arm &>(param);

    motor = MotorIDMap.at(armParam.MotorID_Grip);
    mtrCanTxNode = armParam.MotorTxNode_Grip;

    // 初始化位置PID参数
    armParam.GripPosPidParam.threadNum = 1;
    pidPosCtrl.InitPID(&armParam.GripPosPidParam);

    armParam.GripSpdPidParam.threadNum = 1;
    pidSpdCtrl.InitPID(&armParam.GripSpdPidParam);

    mtrOutputBuffer = 0;

    Component_FSMFlag_ = FSM_RESET;
    componentStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 更新组件
 */
EAppStatus CModArm::CComGrip::UpdateComponent() {
    if (componentStatus == APP_RESET) return APP_ERROR;

    // 更新夹爪当前位置（含Roll耦合补偿）
    gripInfo.posit_grip = static_cast<int32_t>(
        motor->motorData[CDevMtr::DATA_POSIT] * ARM_GRIP_MOTOR_DIR
        + gripInfo.rollCompAccum);

    // 获取End Roll位置用于耦合补偿
    int32_t endRollPosit = 0;
    if (parentModule != nullptr) {
        endRollPosit = parentModule->comEnd_.endInfo.posit_Roll;
    }

    switch (Component_FSMFlag_){

            case FSM_RESET:{
                mtrOutputBuffer = 0;
                motor->motorData[CDevMtr::DATA_POSIT] = 0;
                gripInfo = SGripInfo();  // 完整复位所有状态
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                return APP_OK;
            }
            case FSM_PREINIT:{
                mtrOutputBuffer = 0;
                motor->motorData[CDevMtr::DATA_POSIT] = 0;
                gripInfo = SGripInfo();  // 完整复位所有状态
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                Component_FSMFlag_ = FSM_INIT;
                return APP_OK;
            }
            case FSM_INIT:{
                if(motor->motorStatus == CDevMtr::EMotorStatus::STALL){

                    gripCmd = SGripCmd();
                    motor->motorData[CDevMtr::DATA_POSIT] = static_cast<int32_t> (0.1*8192 + rangeLimit_Grip) * ARM_GRIP_MOTOR_DIR;
                    gripInfo.state = SGripInfo::EGripState::RELEASE;
                    gripInfo.isGripped = false;
                    gripInfo.holdPosit_Grip = 0;
                    gripInfo.lastSetPosit = gripCmd.setPosit_grip;

                    // 初始化增量式 Roll 补偿
                    if (parentModule == nullptr ||  parentModule->comEnd_.componentStatus != APP_OK) {
                        return APP_ERROR;
                    }
                    rollPositAtGripInit_ = parentModule->comEnd_.endInfo.posit_Roll;
                    gripInfo.lastEndRollPosit = rollPositAtGripInit_;
                    gripInfo.rollCompAccum = 0.0f;

                    pidPosCtrl.ResetPidController();
                    pidSpdCtrl.ResetPidController();
                    Component_FSMFlag_ = FSM_CTRL;
                    componentStatus = APP_OK;
                    return APP_OK;
                }
                gripCmd.setPosit_grip += 500;
                return _UpdateOutput(static_cast<float_t>(gripCmd.setPosit_grip), 0);
            }

            case FSM_CTRL:{
                constexpr int32_t gripSpeedStep = static_cast<int32_t>(
                    ARM_GRIP_MANUAL_SPEED_MM_S / 1000.0f * ARM_END_GRIP_MOTOR_RATIO);

                if (gripCmd.cmdClose) {
                    if (gripInfo.state != SGripInfo::EGripState::HOLD) {
                        gripCmd.setPosit_grip -= gripSpeedStep;
                    }
                } else if (gripCmd.cmdOpen) {
                    gripCmd.setPosit_grip += gripSpeedStep;
                }
                

                gripCmd.setPosit_grip = std::clamp<int32_t>(gripCmd.setPosit_grip, static_cast<int32_t>(0), rangeLimit_Grip);

                // 更新增量式Roll补偿
                _UpdateRollCompensation(endRollPosit);

                /* ---- 二次夹紧：从当前位置重新闭合（任何状态均可触发）---- */
                if(gripCmd.cmdReGrip){
                    gripCmd.cmdReGrip = false;
                    gripInfo.state = SGripInfo::EGripState::RELEASE;
                    gripInfo.isGripped = false;
                    gripCmd.setPosit_grip = gripInfo.posit_grip;
                    gripCmd.cmdClose = true;
                    gripCmd.cmdOpen = false;
                    pidPosCtrl.ResetPidController();
                    pidSpdCtrl.ResetPidController();
                }

                /* ---- 双状态控制 ---- */
                // 命令位置，HOLD 状态会覆写为 holdPosit_Grip
                int32_t effectiveTarget = gripCmd.setPosit_grip;

                switch(gripInfo.state) {
                    case SGripInfo::EGripState::RELEASE: {
                        // RELEASE：effectiveTarget 已是 setPosit_grip

                        // --- 硬件堵转检测：电机驱动层检测 ---
                        bool nearMaxOpen = (gripInfo.posit_grip > rangeLimit_Grip - 8192);
                        bool commandWantClose = (gripCmd.setPosit_grip < gripInfo.posit_grip);
                        if(motor->motorStatus == CDevMtr::EMotorStatus::STALL
                           && commandWantClose && !nearMaxOpen){
                            gripInfo.state = SGripInfo::EGripState::HOLD;
                            gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                            gripInfo.isGripped = true;
                            effectiveTarget = gripInfo.holdPosit_Grip;
                            gripInfo.softStallCount = 0;
                            pidPosCtrl.ResetPidController();
                            pidSpdCtrl.ResetPidController();
                            break;
                        }

                        // --- 软件堵转检测：补充硬件检测在低力矩/零点附近的不足 ---
                        // 条件：闭合命令 + 不在最大张开处 + 位置停滞 + 被阻挡无法到达目标
                        if (gripCmd.cmdClose && !nearMaxOpen) {
                            bool positionStuck = (abs(gripInfo.posit_grip - gripInfo.lastPositForSoftStall) < 200);
                            bool blockedFromTarget = (gripInfo.posit_grip > gripCmd.setPosit_grip + 500)
                                || (gripCmd.setPosit_grip == 0);  // 目标已到最小值但仍未到达

                            if (positionStuck && blockedFromTarget) {
                                gripInfo.softStallCount++;
                                if (gripInfo.softStallCount > 100) {  
                                    gripInfo.state = SGripInfo::EGripState::HOLD;
                                    gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                                    gripInfo.isGripped = true;
                                    effectiveTarget = gripInfo.holdPosit_Grip;
                                    gripInfo.softStallCount = 0;
                                    pidPosCtrl.ResetPidController();
                                    pidSpdCtrl.ResetPidController();
                                    break;
                                }
                            } else {
                                gripInfo.softStallCount = 0;
                            }
                        } else {
                            gripInfo.softStallCount = 0;
                        }

                        gripInfo.lastPositForSoftStall = gripInfo.posit_grip;
                        break;
                    }

                    case SGripInfo::EGripState::HOLD: {
                        gripInfo.softStallCount = 0;
                        // HOLD：收到明确张开命令立即退出，避免 reGrip 后目标仍在0导致无法释放
                        if(gripCmd.cmdOpen){
                            gripCmd.setPosit_grip = std::max(
                                gripCmd.setPosit_grip,
                                std::min(gripInfo.holdPosit_Grip + gripSpeedStep, rangeLimit_Grip));
                            effectiveTarget = gripCmd.setPosit_grip;
                            gripInfo.state = SGripInfo::EGripState::RELEASE;
                            gripInfo.isGripped = false;
                            pidPosCtrl.ResetPidController();
                            pidSpdCtrl.ResetPidController();
                        }
                        // 自动任务/目标位置控制：目标明显大于保持位置时退出
                        else if(gripCmd.setPosit_grip > gripInfo.holdPosit_Grip + 819){
                            gripInfo.state = SGripInfo::EGripState::RELEASE;
                            gripInfo.isGripped = false;
                            pidPosCtrl.ResetPidController();
                            pidSpdCtrl.ResetPidController();
                        }
                        else {
                            // 保持：目标固定在原始堵转位置，不随外力拖动
                            effectiveTarget = gripInfo.holdPosit_Grip;
                        }
                        break;
                    }
                }

                gripInfo.lastSetPosit = effectiveTarget;
                return _UpdateOutput(static_cast<float_t>(effectiveTarget), endRollPosit);
            }

            default:{
                StopComponent();
                mtrOutputBuffer = 0;
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                componentStatus = APP_ERROR;
                return APP_ERROR;
            }

        }
        return APP_OK;
}

/**
 * @brief 输出更新函数（Roll耦合补偿）
 */
EAppStatus CModArm::CComGrip::_UpdateOutput(float gripTarget, int32_t endRollPosit) {
    DataBuffer<float_t> gripPos = {
        static_cast<float_t>(gripTarget)
    };

    float_t actualGripPosit = static_cast<float_t>(motor->motorData[CDevMtr::DATA_POSIT])
                              + gripInfo.rollCompAccum;
    DataBuffer<float_t> gripPosMeasured = {actualGripPosit};

    auto pidPosOutput = pidPosCtrl.UpdatePidController(gripPos, gripPosMeasured);

    DataBuffer<float_t> gripSpdMeasured = {static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED])};

    auto Output = pidSpdCtrl.UpdatePidController(pidPosOutput, gripSpdMeasured);

    mtrOutputBuffer = static_cast<int16_t>(Output[0]);

    return APP_OK;
}

/**
 * @brief Roll补偿累积更新函数
 */
void CModArm::CComGrip::_UpdateRollCompensation(int32_t endRollPosit) {
    int32_t deltaRoll = endRollPosit - gripInfo.lastEndRollPosit;

    if (deltaRoll > ARM_END_ROLL_HALF_TURN) {
        deltaRoll -= ARM_END_ROLL_ONE_TURN;
    } else if (deltaRoll < -ARM_END_ROLL_HALF_TURN) {
        deltaRoll += ARM_END_ROLL_ONE_TURN;
    }

    gripInfo.rollCompAccum += static_cast<float_t>(deltaRoll) * ARM_ROLL_GRIP_COUPLING_RATIO;
    gripInfo.lastEndRollPosit = endRollPosit;
}

/**
 * @brief 物理位置转换为电机位置
 *
 * @param phyPosit
 * @return int32_t
 */
int32_t CModArm::CComGrip::PhyPositToMtrPosit(float_t phyPosit) {
    const int32_t zeroOffset = 0;
    const float_t ratio = ARM_END_GRIP_MOTOR_RATIO;
    return static_cast<int32_t>(phyPosit * ratio) + zeroOffset;
}

/**
 * @brief 电机位置转换为物理位置
 */
float_t CModArm::CComGrip::MtrPositToPhyPosit(float_t mtrPosit) {
    const int32_t zeroOffset = 0; ///< 夹爪电机零点偏移
    const float_t ratio = ARM_END_GRIP_MOTOR_RATIO; ///< 夹爪电机与物理位置转换比（单位mm）
    return (static_cast<float_t>(mtrPosit - zeroOffset) / ratio);
}

}//namespace my_engineer
