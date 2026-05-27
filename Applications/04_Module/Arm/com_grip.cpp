/**
 * @file com_grip.cpp
 * @author Ciallo(1002046597@qq.com)
 * @brief 机械臂夹爪组件
 * @version 3.0
 * @date 2025-12-11
 * @lastedit：2026-05-27
 * @details V3.0: 
 * 1. 速度环控制：由于丝杆的导程过长，为了提高夹爪的移动速度
 * 2. 力矩反馈检测：为了防止丝杆的完全卡死的情况要在靠近上次的位置的时候开始减速，在闭合的时候重置初始化标志位
 * 3. RELEASE/HOLD/SAVE 多状态机，为了防止丝杆的自锁现象导致完全的卡死情况加入了自救挣脱模式，通过提高目标位置瞬间输出超大扭矩掰回来
 *
 * @copyright Copyright (c) 2026
 */

 #include "mod_arm.hpp"
 #include "algo_other.hpp"

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
    gripDetect_.closeSpeedMin     = armParam.GripDetectParam.closeSpeedMin;
    gripDetect_.detectTorque      = armParam.GripDetectParam.detectTorque;
    gripDetect_.filterAlpha       = armParam.GripDetectParam.filterAlpha;

    // 自救距离计算
    rescueParam_.distanceEnc = static_cast<int32_t>(rescueParam_.distancePhy * ARM_END_GRIP_MOTOR_RATIO);

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

    // 更新夹爪当前位置
    gripInfo.posit_grip = motor->motorData[CDevMtr::DATA_POSIT] * ARM_GRIP_MOTOR_DIR;

    switch (Component_FSMFlag_) {

        case FSM_RESET: {
            mtrOutputBuffer = 0;
            rescueParam_.detectCnt = 0;
            rescueParam_.elapsedTick = 0;
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            return APP_OK;
        }

        case FSM_PREINIT: {
            mtrOutputBuffer = 0;
            rescueParam_.detectCnt = 0;
            rescueParam_.elapsedTick = 0;
            motor->motorData[CDevMtr::DATA_POSIT] = 0;
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT: {
            if (motor->motorStatus == CDevMtr::EMotorStatus::STALL) {
                gripCmd = SGripCmd();///<堵转之后设置目标值
                motor->motorData[CDevMtr::DATA_POSIT] =
                    static_cast<int32_t>(0.1 * 65535 + rangeLimit_Grip) * ARM_GRIP_MOTOR_DIR;///<堵转零点超量标定
                gripInfo.state = SGripInfo::EGripState::RELEASE;
                gripInfo.isGripped = false;
                gripInfo.holdPosit_Grip = 0;
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                gripDetect_.edgeReady = false;///< 禁用边沿检测，等进入CTRL后滤波力矩低值激活
                Component_FSMFlag_ = FSM_CTRL;
                componentStatus = APP_OK;
                return APP_OK;
            }
            gripCmd.setPosit_grip += 1000;
            return _UpdateOutput(gripCmd.setPosit_grip);
        }

        case FSM_CTRL: {
            gripDetect_.filteredTorque = LowPassFilter(
                gripDetect_.filteredTorque,
                fabsf(static_cast<float_t>(motor->motorData[CDevMtr::DATA_TORQUE])),
                gripDetect_.filterAlpha);

            EGripMoveDir moveDir = DeriveMoveDir();

            // 夹持到位检测
            if (moveDir == EGripMoveDir::CLOSE &&
                gripInfo.posit_grip <= gripCloseStopPosit &&
                gripInfo.state != SGripInfo::EGripState::HOLD) {
                gripInfo.state = SGripInfo::EGripState::HOLD;
                gripInfo.isGripped = true;
                gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                gripDetect_.edgeReady = false;
                gripInfo.repeatInit = false;
            }

            // 张开堵转重新标定
            if (moveDir == EGripMoveDir::OPEN &&
                gripInfo.posit_grip >= gripOpenStopPosit &&
                motor->motorStatus == CDevMtr::EMotorStatus::STALL) {
                motor->motorData[CDevMtr::DATA_POSIT] =
                    static_cast<int32_t>(0.1f * 65535 + rangeLimit_Grip) * ARM_GRIP_MOTOR_DIR;
                gripCmd = SGripCmd();
                gripInfo.state = SGripInfo::EGripState::RELEASE;
                gripInfo.isGripped = false;
                gripInfo.holdPosit_Grip = 0;
                gripDetect_.edgeReady = false;
                gripInfo.repeatInit = true;
                pidSpdCtrl.ResetPidController();
                gripCtrlOutput_ = SGripCtrlOutput{};
                moveDir = EGripMoveDir::IDLE;
            }

            // 二次夹紧请求
            if (gripCmd.cmdReGrip) {
                gripCmd.cmdReGrip = false;
                gripCmd.cmdClose = false;
                gripCmd.cmdOpen  = false;
                gripInfo.repeatInit = false;
                gripCmd.setSpeed_grip = 0.0f;
                pidSpdCtrl.ResetPidController();

                if (gripInfo.state == SGripInfo::EGripState::HOLD) {
                    gripCmd.regripPulse = true;
                    gripCmd.outTime_tick = HAL_GetTick();
                    gripCmd.regripStableCnt = 0;
                } else {
                    gripCmd.setSpeed_grip = -6000;
                    gripCtrlOutput_.mode = SGripCtrlOutput::EGripCtrlMode::SPEED;
                    gripCtrlOutput_.speed = gripCmd.setSpeed_grip;
                    moveDir = EGripMoveDir::CLOSE;
                }
            }

            // 卡死检测与自救触发
            if (gripInfo.state != SGripInfo::EGripState::SAVE &&
                moveDir != EGripMoveDir::IDLE &&
                gripInfo.posit_grip >= gripOpenSlowPosit) {
                float_t absMeasuredSpd = fabsf(static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED]));
                if (absMeasuredSpd < rescueParam_.stuckThresh) {
                    if (++rescueParam_.detectCnt >= rescueParam_.triggerTime) {
                        rescueParam_.targetPosit = gripInfo.posit_grip
                            + (moveDir == EGripMoveDir::CLOSE ? -1 : 1) * rescueParam_.distanceEnc;
                        gripInfo.state = SGripInfo::EGripState::SAVE;
                        rescueParam_.detectCnt = 0;
                        rescueParam_.elapsedTick = 0;
                        pidPosCtrl.ResetPidController();
                        pidSpdCtrl.ResetPidController();
                        gripCtrlOutput_ = SGripCtrlOutput{};
                    }
                } else {
                    rescueParam_.detectCnt = 0;
                }
            }

            // 状态检查
            switch (gripInfo.state) {
                case SGripInfo::EGripState::RELEASE:
                    HandleStateRelease(moveDir, gripCtrlOutput_);
                    break;
                case SGripInfo::EGripState::HOLD:
                    HandleStateHold(moveDir, gripCtrlOutput_);
                    break;
                case SGripInfo::EGripState::SAVE:
                    HandleStateSave(moveDir, gripCtrlOutput_);
                    break;
            }

            return ApplyGripOutput(gripCtrlOutput_);
        }

        default: {
            StopComponent();
            mtrOutputBuffer = 0;
            rescueParam_.detectCnt = 0;
            rescueParam_.elapsedTick = 0;
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            componentStatus = APP_ERROR;
            return APP_ERROR;
        }
    }
}

/*------------------------------------------------------------------------------------*/
// 输出更新函数（位置环）
EAppStatus CModArm::CComGrip::_UpdateOutput(float_t gripTarget) {
    DataBuffer<float_t> gripPos = { static_cast<float_t>(gripTarget) };

    DataBuffer<float_t> gripPosMeasured = {
        static_cast<float_t>(motor->motorData[CDevMtr::DATA_POSIT])
    };

    auto pidPosOutput = pidPosCtrl.UpdatePidController(gripPos, gripPosMeasured);///<角度环

    DataBuffer<float_t> gripSpdMeasured = {
        static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED])
    };

    auto Output = pidSpdCtrl.UpdatePidController(pidPosOutput, gripSpdMeasured);///<速度环

    mtrOutputBuffer = static_cast<int16_t>(Output[0]);

    if (gripInfo.state != SGripInfo::EGripState::SAVE) {
        if (mtrOutputBuffer > GRIP_OUTPUT_LIMIT) mtrOutputBuffer = GRIP_OUTPUT_LIMIT;
        else if (mtrOutputBuffer < -GRIP_OUTPUT_LIMIT) mtrOutputBuffer = -GRIP_OUTPUT_LIMIT;
    } else {
        if (mtrOutputBuffer > GRIP_RESCUE_OUTPUT_LIMIT) mtrOutputBuffer = GRIP_RESCUE_OUTPUT_LIMIT;
        else if (mtrOutputBuffer < -GRIP_RESCUE_OUTPUT_LIMIT) mtrOutputBuffer = -GRIP_RESCUE_OUTPUT_LIMIT;
    }

    return APP_OK;
}

/**
 * @brief 速度环输出更新函数
 * @note 丝杆自锁特性：speedTarget=0时电机停转，夹爪机械保持
 */
EAppStatus CModArm::CComGrip::_UpdateOutputSpd(float_t speedTarget) {
    const float_t motorSpeedTarget = speedTarget * ARM_GRIP_MOTOR_DIR;

    DataBuffer<float_t> spdRef = { static_cast<float_t>(motorSpeedTarget) };

    DataBuffer<float_t> spdMeasured = {
        static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED])
    };

    auto output = pidSpdCtrl.UpdatePidController(spdRef, spdMeasured);

    mtrOutputBuffer = static_cast<int16_t>(output[0]);

    if (gripInfo.state != SGripInfo::EGripState::SAVE) {
        if (mtrOutputBuffer > GRIP_OUTPUT_LIMIT) mtrOutputBuffer = GRIP_OUTPUT_LIMIT;
        else if (mtrOutputBuffer < -GRIP_OUTPUT_LIMIT) mtrOutputBuffer = -GRIP_OUTPUT_LIMIT;
    } else {
        ///< SAVE状态放宽限幅，允许大扭矩挣脱
        if (mtrOutputBuffer > GRIP_RESCUE_OUTPUT_LIMIT) mtrOutputBuffer = GRIP_RESCUE_OUTPUT_LIMIT;
        else if (mtrOutputBuffer < -GRIP_RESCUE_OUTPUT_LIMIT) mtrOutputBuffer = -GRIP_RESCUE_OUTPUT_LIMIT;
    }

    return APP_OK;
}

/**
 * @brief 速度环与位置环统一调用函数
 */
EAppStatus CModArm::CComGrip::ApplyGripOutput(const SGripCtrlOutput& output) {
    if (output.mode != lastCtrlMode_) {
        pidPosCtrl.ResetPidController();
        pidSpdCtrl.ResetPidController();
        lastCtrlMode_ = output.mode;
    }

    switch (output.mode) {
        case SGripCtrlOutput::EGripCtrlMode::POSITION: {
            const float_t motorTarget =
                static_cast<float_t>(output.targetPosit) * ARM_GRIP_MOTOR_DIR;
            return _UpdateOutput(motorTarget);
        }
        case SGripCtrlOutput::EGripCtrlMode::SPEED: {
            return _UpdateOutputSpd(output.speed);
        }
        case SGripCtrlOutput::EGripCtrlMode::STOP:
        default: {
            return _UpdateOutputSpd(0.0f);
        }
    }
}

/*------------------------------------------------------------------------------------*/
/**
 * @brief 判断夹爪的开合方向
 */
CModArm::CComGrip::EGripMoveDir CModArm::CComGrip::DeriveMoveDir() const {
    // 键鼠控制
    if (gripCmd.cmdClose && !gripCmd.cmdOpen) return EGripMoveDir::CLOSE;
    if (gripCmd.cmdOpen && !gripCmd.cmdClose) return EGripMoveDir::OPEN;
    // 遥控器控制
    if (!gripCmd.cmdClose && !gripCmd.cmdOpen) {
        if (gripCmd.setSpeed_grip < 0.0f) return EGripMoveDir::CLOSE;
        if (gripCmd.setSpeed_grip > 0.0f) return EGripMoveDir::OPEN;
    }
    return EGripMoveDir::IDLE;
}

/**
 * @brief 力矩减速限幅
 */
float_t CModArm::CComGrip::ApplyTorqueSpeedLimit(float_t spd) const {
    const float_t excess = gripDetect_.filteredTorque - gripDetect_.closeTorqueThresh;
    spd *= (excess < gripDetect_.closeTorqueRange)
        ? (1.0f - excess / gripDetect_.closeTorqueRange)
        : 0.0f;
    if (spd > -gripDetect_.closeSpeedMin) spd = -gripDetect_.closeSpeedMin;
    return spd;
}

/*------------------------------------------------------------------------------------*/
/**
 * @brief 松开状态
 */
void CModArm::CComGrip::HandleStateRelease(EGripMoveDir moveDir, SGripCtrlOutput& out) {
    const bool isClosing = (moveDir == EGripMoveDir::CLOSE);

    // 力矩边沿检测夹取（仅在闭合时激活，其余方向直接清除）
    if (isClosing) {
        if (gripDetect_.filteredTorque < gripDetect_.detectTorque) {
            gripDetect_.edgeReady = true;
        } else if (gripDetect_.edgeReady && gripDetect_.filteredTorque > gripDetect_.detectTorque) {
            gripInfo.state = SGripInfo::EGripState::HOLD;
            gripInfo.holdPosit_Grip = gripInfo.posit_grip;
            gripInfo.isGripped = true;
            gripDetect_.edgeReady = false;
            out = SGripCtrlOutput{};
            return;
        }
    } else {
        gripDetect_.edgeReady = false;
    }

    /*--- 以下为运动规划：位置环/速度环切换 ---*/
    out = SGripCtrlOutput{};///< 清零：力矩检测没触发，从这里开始规划输出
    if (moveDir == EGripMoveDir::IDLE) return;

    if (isClosing) {
        if (gripInfo.posit_grip <= gripCloseSlowPosit) {
            out.mode = SGripCtrlOutput::EGripCtrlMode::POSITION;//切换为位置环
            out.targetPosit = gripCloseStopPosit;
            return;
        }
        out.mode = SGripCtrlOutput::EGripCtrlMode::SPEED;
        out.speed = gripCmd.cmdClose ? -GRIP_CLOSE_SPEED : gripCmd.setSpeed_grip;// 遥控器接口
        if (gripDetect_.filteredTorque > gripDetect_.closeTorqueThresh) {
            out.speed = ApplyTorqueSpeedLimit(out.speed);
        }
        return;
    }

    // OPEN
    if (gripInfo.posit_grip >= gripOpenStopPosit) {
        if (!gripInfo.repeatInit) {
            out.mode = SGripCtrlOutput::EGripCtrlMode::SPEED;
            out.speed = GRIP_OPEN_SPEED_MIN;// 张开的时候重新标定
        }
        return;
    }
    if (gripInfo.posit_grip >= gripOpenSlowPosit) {
        out.mode = SGripCtrlOutput::EGripCtrlMode::POSITION;// 张开位置环
        out.targetPosit = gripOpenStopPosit;
        return;
    }
    out.mode = SGripCtrlOutput::EGripCtrlMode::SPEED;
    out.speed = gripCmd.cmdOpen ? GRIP_OPEN_SPEED : gripCmd.setSpeed_grip;
}

/**
 * @brief 夹持保持状态
 */
void CModArm::CComGrip::HandleStateHold(EGripMoveDir moveDir, SGripCtrlOutput& out) {
    if (moveDir == EGripMoveDir::OPEN) {
        gripInfo.state = SGripInfo::EGripState::RELEASE;
        gripInfo.isGripped = false;
        gripCmd.regripPulse = false;
        gripInfo.repeatInit = false;
        gripCmd.regripStableCnt = 0;
        out = SGripCtrlOutput{};
        return;
    }

    if (gripCmd.regripPulse) {
        if (HAL_GetTick() - static_cast<uint32_t>(gripCmd.outTime_tick) > 500) {  // 二次夹取的处理时长
            gripCmd.regripPulse = false;
            gripCmd.regripStableCnt = 0;
            gripInfo.holdPosit_Grip = gripInfo.posit_grip;
            out = SGripCtrlOutput{};
            return;
        }

        float_t spd = -GRIP_CLOSE_SPEED;
        if (gripDetect_.filteredTorque > gripDetect_.closeTorqueThresh) {
            spd = ApplyTorqueSpeedLimit(spd);
        }

        const bool spdAtFloor = (spd >= -gripDetect_.closeSpeedMin * 1.05f);
        const bool torqueHigh = (gripDetect_.filteredTorque > gripDetect_.closeTorqueThresh);// 这个两个判断条件是低速高扭矩则判断重复夹紧
        if (spdAtFloor && torqueHigh) {
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

        out.mode = SGripCtrlOutput::EGripCtrlMode::SPEED;
        out.speed = spd;
        return;
    }

    out = SGripCtrlOutput{};
}

/**
 * @brief 自救挣脱状态
 */
void CModArm::CComGrip::HandleStateSave(EGripMoveDir moveDir, SGripCtrlOutput& out) {
    out.mode = SGripCtrlOutput::EGripCtrlMode::POSITION;
    out.targetPosit = rescueParam_.targetPosit;

    const int32_t posError = gripInfo.posit_grip - rescueParam_.targetPosit;
    const int32_t absPosError = posError < 0 ? -posError : posError;
    const bool arrived = (absPosError < static_cast<int32_t>(ARM_END_GRIP_MOTOR_RATIO * 1.0f));
    const bool timeout = (++rescueParam_.elapsedTick > rescueParam_.timeout);

    if (arrived || timeout) { // 如果到达了位置或者是超过自救挣脱的倒计时之后切换为释放模式
        gripInfo.state = SGripInfo::EGripState::RELEASE;
        rescueParam_.elapsedTick = 0;
        pidPosCtrl.ResetPidController();
        pidSpdCtrl.ResetPidController();
        out = SGripCtrlOutput{};// 本周期停止输出，下一周期由 RELEASE 重新规划
    }
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
