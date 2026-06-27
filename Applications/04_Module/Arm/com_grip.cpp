/**
 * @file com_grip.cpp
 * @author Ciallo(1002046597@qq.com)
 * @brief 机械臂夹爪组件
 * @version 5.0
 * @date 2025-12-11
 * @lastedit：2026-06-04
 * @details V5.0
 *
 * @copyright Copyright (c) 2026
 */

 #include "mod_arm.hpp"
 #include "algo_other.hpp"
int gripMoveDir = 0;  // -1=CLOSE, 0=IDLE, 1=OPEN
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

    // 初始化速度PID参数
    armParam.GripSpdPidParam.threadNum = 1;
    pidSpdCtrl.InitPID(&armParam.GripSpdPidParam);

    // 夹取检测参数
    gripDetect_.closeTorqueThresh = armParam.GripDetectParam.closeTorqueThresh;
    gripDetect_.closeTorqueRange  = armParam.GripDetectParam.closeTorqueRange;
    gripDetect_.detectTorque      = armParam.GripDetectParam.detectTorque;
    gripDetect_.filterAlpha       = armParam.GripDetectParam.filterAlpha;

    mtrOutputBuffer = 0;

    Component_FSMFlag_ = FSM_RESET;
    componentStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 更新组件
 */
EAppStatus CModArm::CComGrip::UpdateComponent() {
    if (componentStatus == APP_RESET) {
        mtrOutputBuffer = 0;
        return APP_ERROR;
    }

    // 更新夹爪当前位置（夹爪逻辑位置：张开=正，闭合端=0）
    gripInfo.posit_grip = motor->motorData[CDevMtr::DATA_POSIT] * ARM_GRIP_POS_DIR;

    gripDetect_.filteredTorque = LowPassFilter(
                gripDetect_.filteredTorque,
                fabsf(static_cast<float_t>(motor->motorData[CDevMtr::DATA_CURRENT])),
                gripDetect_.filterAlpha);
    switch (Component_FSMFlag_) {

        case FSM_RESET: {
            mtrOutputBuffer = 0;
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            return APP_OK;
        }

        case FSM_PREINIT: {
            mtrOutputBuffer = 0;
            motor->motorData[CDevMtr::DATA_POSIT] = 0;
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            initStallCnt_ = 0;
            gripDetect_.filteredTorque = 0.0f;   ///< 清零滤波力矩，防止上电电流尖峰残留导致误判堵转
            rampSpd_.Reset(0.0f);
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT: {
            // 组件层堵转判断
            if (gripDetect_.filteredTorque > GRIP_INIT_STALL_CURRENT) {
                initStallCnt_++;
                if (initStallCnt_ >= GRIP_INIT_STALL_COUNT) {
                    // 堵转确认，标定完成
                    gripCmd = SGripCmd();
                    motor->motorData[CDevMtr::DATA_POSIT] = 0;
                    gripInfo.state = SGripInfo::EGripState::RELEASE;
                    gripInfo.isGripped = false;
                    gripInfo.holdPosit_Grip = 0;
                    pidPosCtrl.ResetPidController();
                    pidSpdCtrl.ResetPidController();
                    rampSpd_.Reset(0.0f);   ///< 堵转确认后立即归零斜坡，避免 FSM_CTRL 阶段死磕
                    gripDetect_.edgeReady = false;
                    initStallCnt_ = 0;
                    Component_FSMFlag_ = FSM_CTRL;
                    componentStatus = APP_OK;
                    return APP_OK;
                }
            } else {
                initStallCnt_ = 0;
            }
            // 恒定低速标定，速度环斜坡驱动
            rampSpd_.SetTarget(-GRIP_INIT_SPEED, GRIP_SPEED_RAMP_STEP);
            return _UpdateOutputSpd(rampSpd_.Update());
        }

        case FSM_CTRL: {
            EGripMoveDir moveDir = DeriveMoveDir();
            gripMoveDir = static_cast<int>(moveDir);
            // 切换方向时重置速度环 PID
            static EGripMoveDir lastMoveDirForReset_ = EGripMoveDir::IDLE;
            if (moveDir != lastMoveDirForReset_) {
                pidSpdCtrl.ResetPidController();
                lastMoveDirForReset_ = moveDir;
            }
            // 闭合到位检测
            if (moveDir == EGripMoveDir::CLOSE &&
                gripInfo.posit_grip <= gripCloseStopPosit &&
                gripInfo.state != SGripInfo::EGripState::HOLD) {
                gripInfo.state = SGripInfo::EGripState::HOLD;
                gripInfo.isGripped = true;
                gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                gripDetect_.edgeReady = false;
            }

            // 二次夹紧请求
            if (gripCmd.cmdReGrip) {
                gripCmd.cmdReGrip = false;
                gripCmd.cmdClose = false;
                gripCmd.cmdOpen  = false;

                if (gripInfo.state == SGripInfo::EGripState::HOLD) {
                    gripCmd.regripPulse = true;
                    gripCmd.outTime_tick = HAL_GetTick();
                    gripCmd.regripStableCnt = 0;
                }
            }

            // 状态切换
            switch (gripInfo.state) {
                case SGripInfo::EGripState::RELEASE:
                    HandleStateRelease(moveDir, gripCtrlOutput_);
                    break;
                case SGripInfo::EGripState::HOLD:
                    HandleStateHold(moveDir, gripCtrlOutput_);
                    break;
            }

            return ApplyGripOutput(gripCtrlOutput_);
        }

        default: {
            StopComponent();
            mtrOutputBuffer = 0;
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            componentStatus = APP_ERROR;
            return APP_ERROR;
        }
    }
}

/*------------------------------------------------------------------------------------*/
/**
 * @brief 闭合方向力矩线性减速
 */
float_t CModArm::CComGrip::ApplyTorqueSpeedLimit(float_t spd) const {
    if (gripDetect_.filteredTorque > gripDetect_.closeTorqueThresh) {
        float_t factor = std::max(0.0f,
            1.0f - (gripDetect_.filteredTorque - gripDetect_.closeTorqueThresh)
                  / gripDetect_.closeTorqueRange);
        spd *= factor;
    }
    return spd;
}

/**
 * @brief 速度环输出更新函数
 */
EAppStatus CModArm::CComGrip::_UpdateOutputSpd(float_t speedTarget) {
    DataBuffer<float_t> gripSpd = { speedTarget };
    DataBuffer<float_t> gripSpdMeasured = {
        static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED]) * ARM_GRIP_SPD_DIR
    };

    auto Output = pidSpdCtrl.UpdatePidController(gripSpd, gripSpdMeasured);
    mtrOutputBuffer = static_cast<int16_t>(Output[0] * ARM_GRIP_SPD_DIR);

    // 输出限幅
    if (mtrOutputBuffer > GRIP_OUTPUT_LIMIT) mtrOutputBuffer = GRIP_OUTPUT_LIMIT;
    else if (mtrOutputBuffer < -GRIP_OUTPUT_LIMIT) mtrOutputBuffer = -GRIP_OUTPUT_LIMIT;

    return APP_OK;
}

/*------------------------------------------------------------------------------------*/
/**
 * @brief 位置环输出更新函数
 */
EAppStatus CModArm::CComGrip::_UpdateOutput(float_t gripTarget) {
    DataBuffer<float_t> gripPos = { static_cast<float_t>(gripTarget) };

    DataBuffer<float_t> gripPosMeasured = {
        static_cast<float_t>(motor->motorData[CDevMtr::DATA_POSIT]) * ARM_GRIP_POS_DIR
    };

    auto pidPosOutput = pidPosCtrl.UpdatePidController(gripPos, gripPosMeasured);

    DataBuffer<float_t> gripSpdMeasured = {
        static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED]) * ARM_GRIP_SPD_DIR
    };

    auto Output = pidSpdCtrl.UpdatePidController(pidPosOutput, gripSpdMeasured);

    mtrOutputBuffer = static_cast<int16_t>(Output[0] * ARM_GRIP_SPD_DIR);

    return APP_OK;
}

/**
 * @brief 输出模式选择
 */
EAppStatus CModArm::CComGrip::ApplyGripOutput(const SGripCtrlOutput& output) {
    if (output.mode != lastCtrlMode_) {
        pidSpdCtrl.ResetPidController();
        lastCtrlMode_ = output.mode;
    }

    switch (output.mode) {
        case SGripCtrlOutput::EGripCtrlMode::SPEED:
            rampSpd_.SetTarget(output.targetSpeed, GRIP_SPEED_RAMP_STEP);
            return _UpdateOutputSpd(rampSpd_.Update());
        case SGripCtrlOutput::EGripCtrlMode::STOP:
        default:
            rampSpd_.SetTarget(0.0f, GRIP_SPEED_RAMP_STEP);
            if (rampSpd_.IsArrived()) {
                mtrOutputBuffer = 0;
                return APP_OK;
            }
            return _UpdateOutputSpd(rampSpd_.Update());
    }
}

/*------------------------------------------------------------------------------------*/
/**
 * @brief 判断夹爪的运动方向（速度方向）
 * @note 键鼠控制优先用cmdClose/cmdOpen标志，否则通过设定速度正负判断方向
 */
CModArm::CComGrip::EGripMoveDir CModArm::CComGrip::DeriveMoveDir() const {
    // 键鼠控制：cmdClose/cmdOpen 标志优先
    if (gripCmd.cmdClose && !gripCmd.cmdOpen) return EGripMoveDir::CLOSE;
    if (gripCmd.cmdOpen && !gripCmd.cmdClose) return EGripMoveDir::OPEN;

    // 速度模式：通过设定速度的正负判断方向
    const float_t deadband = 100.0f; // 速度死区
    if (gripCmd.setSpeed_grip > deadband)  return EGripMoveDir::OPEN;
    if (gripCmd.setSpeed_grip < -deadband) return EGripMoveDir::CLOSE;

    return EGripMoveDir::IDLE;
}

/*------------------------------------------------------------------------------------*/
/**
 * @brief 松开状态
 */
void CModArm::CComGrip::HandleStateRelease(EGripMoveDir moveDir, SGripCtrlOutput& out) {
    const bool isClosing = (moveDir == EGripMoveDir::CLOSE);

    out = SGripCtrlOutput{};
    if (moveDir == EGripMoveDir::IDLE) {
        gripDetect_.gripHoldCnt = 0;
        closeStartCnt_ = 0;
        return;
    }

    // 位置限幅：到达边界则停止
    if (isClosing && gripInfo.posit_grip <= 0) {
        gripDetect_.gripHoldCnt = 0;
        closeStartCnt_ = 0;
        return;
    }
    if (!isClosing && gripInfo.posit_grip >= rangeLimit_Grip) {
        gripDetect_.gripHoldCnt = 0;
        closeStartCnt_ = 0;
        return;
    }

    // 闭合距离判断
    const float_t closedis = isClosing ? static_cast<float_t>(gripInfo.posit_grip) : static_cast<float_t>(rangeLimit_Grip - gripInfo.posit_grip);

    float_t targetSpeed;
    if (closedis > static_cast<float_t>(GRIP_SLOW_MACH)) {
        targetSpeed = GRIP_OPEN_SPEED;   // 中段：全速
    } else {
        // 减速带
        const float_t ratio = closedis / static_cast<float_t>(GRIP_SLOW_MACH);
        targetSpeed = GRIP_INIT_SPEED + ratio * (GRIP_OPEN_SPEED - GRIP_INIT_SPEED);
    }

    // 夹取检测
    if (isClosing) {
        if (closeStartCnt_ < GRIP_CLOSE_START_GRACE) {
            closeStartCnt_++;
        } else {
            const float_t actualSpeed = fabsf(static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED]));
            const bool torqueHigh    = (gripDetect_.filteredTorque > gripDetect_.detectTorque);
            const bool speedStalled  = (actualSpeed < GRIP_STALL_SPEED_THRESH);

            if (torqueHigh && speedStalled) {
                gripDetect_.gripHoldCnt++;
                if (gripDetect_.gripHoldCnt >= GRIP_GRIP_HOLD_COUNT) {
                    gripInfo.state = SGripInfo::EGripState::HOLD;
                    gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                    gripInfo.isGripped = true;
                    gripDetect_.gripHoldCnt = 0;
                    closeStartCnt_ = 0;

                    out = SGripCtrlOutput{};
                    return;
                }
            } else {
                gripDetect_.gripHoldCnt = 0;
            }
        }
    } else {
        gripDetect_.gripHoldCnt = 0;
        closeStartCnt_ = 0;
    }

    // 速度环：根据方向输出速度
    out.mode = SGripCtrlOutput::EGripCtrlMode::SPEED;
    if (isClosing) {
        float_t closeSpeed = -targetSpeed;
        if (closeStartCnt_ >= GRIP_CLOSE_START_GRACE) {
            closeSpeed = ApplyTorqueSpeedLimit(closeSpeed);
        }
        out.targetSpeed = closeSpeed;
    } else {
        out.targetSpeed = targetSpeed;
    }
}

/**
 * @brief 夹持保持状态
 */
void CModArm::CComGrip::HandleStateHold(EGripMoveDir moveDir, SGripCtrlOutput& out) {
    // 张开切回 RELEASE
    if (moveDir == EGripMoveDir::OPEN) {
        gripInfo.state = SGripInfo::EGripState::RELEASE;
        gripInfo.isGripped = false;
        gripInfo.holdPosit_Grip = 0;
        gripCmd.regripPulse = false;
        gripCmd.regripStableCnt = 0;
        out = SGripCtrlOutput{};
        return;
    }

    // 二次夹紧
    if (gripCmd.regripPulse) {
        // 超时停止
        if (HAL_GetTick() - static_cast<uint32_t>(gripCmd.outTime_tick) > 250) {
            gripCmd.regripPulse = false;
            gripCmd.regripStableCnt = 0;
            gripInfo.holdPosit_Grip = gripInfo.posit_grip;
            out = SGripCtrlOutput{};
            return;
        }

        // 行程保护：超出夹爪最大行程则停止
        if (gripInfo.posit_grip > rangeLimit_Grip) {
            gripCmd.regripPulse = false;
            gripCmd.regripStableCnt = 0;
            gripInfo.holdPosit_Grip = gripInfo.posit_grip;
            out = SGripCtrlOutput{};
            return;
        }

        out.mode = SGripCtrlOutput::EGripCtrlMode::SPEED;
        out.targetSpeed = -8000;

        // 力矩稳定停止
        const bool torqueHigh = (gripDetect_.filteredTorque > gripDetect_.closeTorqueThresh);
        if (torqueHigh) {
            gripCmd.regripStableCnt++;
            if (gripCmd.regripStableCnt >= 20) {
                gripCmd.regripPulse = false;
                gripCmd.regripStableCnt = 0;
                gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                out = SGripCtrlOutput{};
                return;
            }
        } else {
            gripCmd.regripStableCnt = 0;
        }
        return;
    }

    // 正常 HOLD：零输出，丝杆自锁
    out = SGripCtrlOutput{};
}

/*------------------------------------------------------------------------------------*/
// 物理位置转换为电机位置
int32_t CModArm::CComGrip::PhyPositToMtrPosit(float_t phyPosit) {
    const int32_t zeroOffset = 0;
    const float_t ratio = ARM_END_GRIP_MOTOR_RATIO;
    return static_cast<int32_t>(phyPosit * ratio) + zeroOffset;
}

/**
 * @brief 电机位置转换为物理位置
 */
float_t CModArm::CComGrip::MtrPositToPhyPosit(float_t mtrPosit) {
    const int32_t zeroOffset = 0;
    const float_t ratio = ARM_END_GRIP_MOTOR_RATIO;
    return (static_cast<float_t>(mtrPosit - zeroOffset) / ratio);
}

}//namespace my_engineer
