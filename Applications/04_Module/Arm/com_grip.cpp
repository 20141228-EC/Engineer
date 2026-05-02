/**
 * @file com_grip.cpp
 * @author Ciallo～(∠·ω< )⌒☆(1002046597@qq.com)
 * @brief 机械臂夹爪组件
 * @version 2.0
 * @date 2025-12-11
 * @lastedit：2026-04-30
 * @details V2.0: 机械结构更换为2006丝杆直驱，移除Roll轴耦合补偿
 *          速度环控制 + 力矩反馈检测 + RELEASE/HOLD 双状态机
 *
 * @copyright Copyright (c) 2025
 *
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

    // 初始化力矩反馈减速参数
    GripinitParam_.initSpeedMax_     = armParam.GripInitParam.initSpeedMax_;
    GripinitParam_.initSpeedMin_     = armParam.GripInitParam.initSpeedMin_;
    GripinitParam_.initTorqueThresh_ = armParam.GripInitParam.initTorqueThresh_;
    GripinitParam_.initTorqueRange_  = armParam.GripInitParam.initTorqueRange_;

    // 夹取检测参数
    gripDetect_.closeTorqueThresh = armParam.GripDetectParam.closeTorqueThresh;
    gripDetect_.closeTorqueRange  = armParam.GripDetectParam.closeTorqueRange;
    gripDetect_.closeSpeedMin     = armParam.GripDetectParam.closeSpeedMin;
    gripDetect_.detectTorque      = armParam.GripDetectParam.detectTorque;
    gripDetect_.filterAlpha       = armParam.GripDetectParam.filterAlpha;

    //输出缓冲区清零
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

    // 获取力矩绝对值
    auto getAbsTorque = [this]() -> float_t {
        return static_cast<float_t>(
            motor->motorData[CDevMtr::DATA_TORQUE] < 0
                ? -motor->motorData[CDevMtr::DATA_TORQUE]
                :  motor->motorData[CDevMtr::DATA_TORQUE]);
    };

    // 更新夹爪当前位置
    gripInfo.posit_grip = motor->motorData[CDevMtr::DATA_POSIT] * ARM_GRIP_MOTOR_DIR;

    switch (Component_FSMFlag_){

            case FSM_RESET:{
                // 复位状态，夹爪电机输出为0
                mtrOutputBuffer = 0;
                speedTargetFiltered_ = 0.0f;
                speedMeasuredFiltered_ = 0.0f;
                initTick_ = 0;
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                return APP_OK;
            }
            case FSM_PREINIT:{
                // 预初始化状态，夹爪电机输出为0
                mtrOutputBuffer = 0;
                speedTargetFiltered_ = 0.0f;
                speedMeasuredFiltered_ = 0.0f;
                initTick_ = 0;
                motor->motorData[CDevMtr::DATA_POSIT] = 0;
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                Component_FSMFlag_ = FSM_INIT;
                return APP_OK;
            }
            case FSM_INIT:{
                if(motor->motorStatus == CDevMtr::EMotorStatus::STALL){

                    gripCmd = SGripCmd();///<堵转之后设置目标值
                    motor->motorData[CDevMtr::DATA_POSIT] = static_cast<int32_t> (0.1*8192 + rangeLimit_Grip) * ARM_GRIP_MOTOR_DIR;///<堵转零点超量标定
                    gripInfo.state = SGripInfo::EGripState::RELEASE;
                    gripInfo.isGripped = false;
                    gripInfo.holdPosit_Grip = 0;

                    pidPosCtrl.ResetPidController();
                    pidSpdCtrl.ResetPidController();
                    speedTargetFiltered_ = 0.0f;
                    speedMeasuredFiltered_ = 0.0f;
                    initTick_ = 0;
                    gripDetect_.edgeReady = false;  ///< 禁用边沿检测，等进入CTRL后滤波力矩低值激活
                    Component_FSMFlag_ = FSM_CTRL;
                    componentStatus = APP_OK;
                    return APP_OK;
                }

                // 力矩反馈线性减速
                gripDetect_.filteredTorque = getAbsTorque();
                if (gripDetect_.filteredTorque > GripinitParam_.initTorqueThresh_) {
                    gripCmd.setSpeed_grip = GripinitParam_.initSpeedMax_  * (1.0f - (gripDetect_.filteredTorque - GripinitParam_.initTorqueThresh_) / GripinitParam_.initTorqueRange_);
                    if (gripCmd.setSpeed_grip < GripinitParam_.initSpeedMin_) gripCmd.setSpeed_grip = GripinitParam_.initSpeedMin_;
                } else {
                    gripCmd.setSpeed_grip = GripinitParam_.initSpeedMax_;
                }
                return _UpdateOutputSpd(gripCmd.setSpeed_grip);
            }
            case FSM_CTRL:{
                /// 自动控制速度常量（电机的转速rpm）
                const float_t GRIP_AUTO_SPEED = 6000.0f;

                // 力矩低通滤波
                gripDetect_.filteredTorque = LowPassFilter(
                    gripDetect_.filteredTorque, getAbsTorque(), gripDetect_.filterAlpha);

                // cmdClose/cmdOpen
                if (gripCmd.cmdClose) {
                    gripCmd.setSpeed_grip = -GRIP_AUTO_SPEED;
                } else if (gripCmd.cmdOpen) {
                    gripCmd.setSpeed_grip = GRIP_AUTO_SPEED;
                }

                /* ---- 二次夹紧：从当前位置重新闭合（任何状态均可触发）---- */
                if(gripCmd.cmdReGrip){
                    gripCmd.cmdReGrip = false;
                    gripInfo.state = SGripInfo::EGripState::RELEASE;
                    gripInfo.isGripped = false;
                    gripCmd.cmdClose = false;
                    gripCmd.cmdOpen = false;
                    gripCmd.setSpeed_grip = -GRIP_AUTO_SPEED; ///< 闭合方向
                    pidSpdCtrl.ResetPidController();
                }

                // 软件限位：基于编码器位置防止超出机械行程
                const bool isClosingCommand = gripCmd.setSpeed_grip < 0;
                if (gripInfo.posit_grip <= 0 && isClosingCommand) {
                    gripCmd.setSpeed_grip = 0;  ///< 已到闭合极限，禁止继续闭合
                }
                if (gripInfo.posit_grip >= rangeLimit_Grip && gripCmd.setSpeed_grip > 0) {
                    gripCmd.setSpeed_grip = 0;  ///< 已到张开极限，禁止继续张开
                }

                /* ---- 双状态控制 ---- */
                switch(gripInfo.state) {
                    case SGripInfo::EGripState::RELEASE: {
                        if (gripInfo.posit_grip <= 0 && isClosingCommand) {
                            gripInfo.state = SGripInfo::EGripState::HOLD;
                            gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                            gripInfo.isGripped = true;
                            gripDetect_.edgeReady = false;
                            return _UpdateOutputSpd(0);
                        }

                        // 力矩边沿检测：低->高 边沿判定夹取成功
                        if (gripDetect_.filteredTorque < gripDetect_.detectTorque) {
                            gripDetect_.edgeReady = true;
                        } else if (gripDetect_.edgeReady && gripDetect_.filteredTorque > gripDetect_.detectTorque) {
                            if (gripCmd.setSpeed_grip < 0) { ///< 仅闭合方向触发
                                gripInfo.state = SGripInfo::EGripState::HOLD;
                                gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                                gripInfo.isGripped = true;
                                gripDetect_.edgeReady = false;
                                return _UpdateOutputSpd(0);
                            }
                            gripDetect_.edgeReady = false;
                        }

                        // 闭合方向滤波力矩反馈减速，保护夹爪和工件
                        if (gripCmd.setSpeed_grip < 0 && gripDetect_.filteredTorque > gripDetect_.closeTorqueThresh) {
                            gripCmd.setSpeed_grip *= (gripDetect_.filteredTorque - gripDetect_.closeTorqueThresh < gripDetect_.closeTorqueRange)
                                ? (1.0f - (gripDetect_.filteredTorque - gripDetect_.closeTorqueThresh) / gripDetect_.closeTorqueRange)
                                : 0.0f;
                            if (gripCmd.setSpeed_grip > -gripDetect_.closeSpeedMin)
                                gripCmd.setSpeed_grip = -gripDetect_.closeSpeedMin;
                        }
                        return _UpdateOutputSpd(gripCmd.setSpeed_grip);
                    }

                    case SGripInfo::EGripState::HOLD: {
                        // 收到张开方向指令则退出夹持
                        if(gripCmd.setSpeed_grip > 0){
                            gripInfo.state = SGripInfo::EGripState::RELEASE;
                            gripInfo.isGripped = false;
                            pidSpdCtrl.ResetPidController();
                            return _UpdateOutputSpd(gripCmd.setSpeed_grip);
                        }
                        // 保持夹持：速度归零，丝杆自锁保持位置
                        return _UpdateOutputSpd(0);
                    }
                }
            }
            default:{
                StopComponent();
                mtrOutputBuffer = 0;
                speedTargetFiltered_ = 0.0f;
                speedMeasuredFiltered_ = 0.0f;
                initTick_ = 0;
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                componentStatus = APP_ERROR;
                return APP_ERROR;
            }

        }
    return APP_OK;
}

/**
 * @brief 输出更新函数（位置环）
 */
EAppStatus CModArm::CComGrip::_UpdateOutput(float_t gripTarget) {
    DataBuffer<float_t> gripPos = {
        static_cast<float_t>(gripTarget)
    };

    DataBuffer<float_t> gripPosMeasured = {
        static_cast<float_t>(motor->motorData[CDevMtr::DATA_POSIT])
    };

    auto pidPosOutput = pidPosCtrl.UpdatePidController(gripPos, gripPosMeasured);

    DataBuffer<float_t> gripSpdMeasured = {static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED])};

    auto Output = pidSpdCtrl.UpdatePidController(pidPosOutput, gripSpdMeasured);

    mtrOutputBuffer = static_cast<int16_t>(Output[0]);

    return APP_OK;
}

/**
 * @brief 速度环输出更新函数（复用内环pidSpdCtrl）
 *
 * @note 丝杆自锁特性：speedTarget=0时电机停转，夹爪机械保持
 */
EAppStatus CModArm::CComGrip::_UpdateOutputSpd(float_t speedTarget) {
    const float_t motorSpeedTarget = speedTarget * ARM_GRIP_MOTOR_DIR;

    DataBuffer<float_t> spdRef = {
        static_cast<float_t>(motorSpeedTarget)
    };

    DataBuffer<float_t> spdMeasured = {
        static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED])
    };

    auto output = pidSpdCtrl.UpdatePidController(spdRef, spdMeasured);

    mtrOutputBuffer = static_cast<int16_t>(output[0]);

    return APP_OK;
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
    const int32_t zeroOffset = 0;
    const float_t ratio = ARM_END_GRIP_MOTOR_RATIO;
    return (static_cast<float_t>(mtrPosit - zeroOffset) / ratio);
}



}//namespace my_engineer
