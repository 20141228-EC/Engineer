/**
 * @file com_grip.cpp
 * @author Ciallo～(∠·ω< )⌒☆(1002046597@qq.com)
 * @brief 机械臂夹爪组件
 * @version 2.0
 * @date 2025-12-11
 * @lastedit：2026-05-09
 * @details V3.0: 
 * 1. 速度环控制：由于丝杆的导程过长，为了提高夹爪的移动速度
 * 2. 力矩反馈检测：为了防止丝杆夹爪的完全卡死的情况要在靠近上次的位置的时候开始减速，在闭合的时候重置初始化标志位 
 * 3. RELEASE/HOLD/SAVE 多状态机，为了防止丝杆的自锁现象导致完全的卡死情况加入了自救挣脱模式，通过提高目标位置瞬间输出超大扭矩掰回来
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

    // 自救距离计算
    rescueParam_.distanceEnc = static_cast<int32_t>(rescueParam_.distancePhy * ARM_END_GRIP_MOTOR_RATIO);

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

    // 更新夹爪当前位置
    gripInfo.posit_grip = motor->motorData[CDevMtr::DATA_POSIT] * ARM_GRIP_MOTOR_DIR;

    switch (Component_FSMFlag_){

            case FSM_RESET:{
                // 复位状态，夹爪电机输出为0
                mtrOutputBuffer = 0;
                speedTargetFiltered_ = 0.0f;
                speedMeasuredFiltered_ = 0.0f;
                initTick_ = 0;
                rescueParam_.detectCnt = 0;
                rescueParam_.elapsedTick = 0;
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
                rescueParam_.detectCnt = 0;
                rescueParam_.elapsedTick = 0;
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
                gripDetect_.filteredTorque = fabsf(static_cast<float_t>(motor->motorData[CDevMtr::DATA_TORQUE]));
                if (gripDetect_.filteredTorque > GripinitParam_.initTorqueThresh_) {
                    gripCmd.setSpeed_grip = GripinitParam_.initSpeedMax_  * (1.0f - (gripDetect_.filteredTorque - GripinitParam_.initTorqueThresh_) / GripinitParam_.initTorqueRange_);
                    if (gripCmd.setSpeed_grip < GripinitParam_.initSpeedMin_) gripCmd.setSpeed_grip = GripinitParam_.initSpeedMin_;
                } else {
                    gripCmd.setSpeed_grip = GripinitParam_.initSpeedMax_;
                }
                return _UpdateOutputSpd(gripCmd.setSpeed_grip);
            }
            case FSM_CTRL:{

                // 力矩低通滤波
                gripDetect_.filteredTorque = LowPassFilter(
                    gripDetect_.filteredTorque, fabsf(static_cast<float_t>(motor->motorData[CDevMtr::DATA_TORQUE])), gripDetect_.filterAlpha);

                // cmdClose/cmdOpen核心层的写入控制张开和闭合
                if (gripCmd.cmdClose) {
                    if (gripInfo.posit_grip <= gripCloseStopPosit) {
                        // 已到/超出闭合停止位：保底速度，切HOLD，堵转标定零点
                        gripCmd.setSpeed_grip = -gripDetect_.closeSpeedMin;
                        if (gripInfo.state != SGripInfo::EGripState::HOLD) {
                            gripInfo.state = SGripInfo::EGripState::HOLD;
                            gripInfo.isGripped = true;
                            gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                            gripDetect_.edgeReady = false;
                            gripInfo.repeatInit = false;
                        }
                    } else if (gripInfo.posit_grip <= gripCloseSlowPosit) {
                        // 处于闭合减速区：按距离线性减速
                        gripCmd.setSpeed_grip = -CalcGripSlowSpeed(
                            GRIP_CLOSE_SPEED, gripDetect_.closeSpeedMin,
                            gripInfo.posit_grip - gripCloseStopPosit,
                            gripCloseSlowPosit - gripCloseStopPosit);
                    } else {
                        gripCmd.setSpeed_grip = -GRIP_CLOSE_SPEED;
                    }
                } else if (gripCmd.cmdOpen) {
                    if (gripInfo.posit_grip >= gripOpenStopPosit) {
                        // 初始化的时候只是标定一次
                        if(gripInfo.repeatInit == false){
                            gripCmd.setSpeed_grip = GRIP_OPEN_SPEED_MIN;
                        }
                        else gripCmd.setSpeed_grip = 0;//后续直接通过丝杆的位置来实现锁定，直接电机卸力

                        if (motor->motorStatus == CDevMtr::EMotorStatus::STALL) {
                            motor->motorData[CDevMtr::DATA_POSIT] =
                                static_cast<int32_t>(0.1f * 8192 + rangeLimit_Grip) * ARM_GRIP_MOTOR_DIR;
                            gripCmd = SGripCmd(); // 全部命令清零，避免下一周期继续顶限位
                            gripInfo.state = SGripInfo::EGripState::RELEASE;
                            gripInfo.isGripped = false;
                            gripInfo.holdPosit_Grip = 0;
                            gripDetect_.edgeReady = false;
                            gripInfo.repeatInit = true;
                            pidSpdCtrl.ResetPidController();
                            mtrOutputBuffer = 0;
                            return _UpdateOutputSpd(0);//堵转的时候直接设置速度0
                        }
                    } else if (gripInfo.posit_grip >= gripOpenSlowPosit && gripInfo.posit_grip < gripOpenStopPosit) {
                        // 处于张开减速区，按距离线性减速
                        gripCmd.setSpeed_grip = CalcGripSlowSpeed(
                            GRIP_OPEN_SPEED, GRIP_OPEN_SPEED_MIN,
                            gripOpenStopPosit - gripInfo.posit_grip,
                            gripOpenStopPosit - gripOpenSlowPosit);
                    } else {
                        gripCmd.setSpeed_grip = GRIP_OPEN_SPEED;
                    }
                }

                /* ---- 二次夹紧 ---- */
                if(gripCmd.cmdReGrip){
                    gripCmd.cmdReGrip = false;
                    gripCmd.cmdClose = false;
                    gripCmd.cmdOpen  = false;
                    gripInfo.repeatInit = false;//夹紧的时候直接清零重置
                    gripCmd.setSpeed_grip = 0.0f; 
                    pidSpdCtrl.ResetPidController();

                    if (gripInfo.state == SGripInfo::EGripState::HOLD) {
                        if (gripInfo.posit_grip < gripInfo.holdPosit_Grip - gripCmd.regripMaxTravel) {
                            gripInfo.isGripped = false;
                            return _UpdateOutputSpd(0);
                        }
                        gripCmd.regripPulse = true;
                        gripCmd.outTime_tick = HAL_GetTick();
                        gripCmd.regripStableCnt = 0;
                    } else {
                        // 未夹住：当作普通闭合命令，走 RELEASE 分支正常流程
                        gripCmd.setSpeed_grip = -6000;//整个速度按照实际的情况修改
                    }
                }

                // 命令方向设置
                const bool isClosingCommand = gripCmd.setSpeed_grip < 0;
                const bool isOpeningCommand = gripCmd.setSpeed_grip > 0;

                /* ---- 自锁自救检测 ---- */
                // 特别注意：仅在张开限位附近检测，自锁只发生在这个位置，中段不可能自锁，但是还有一个情况就是如果在多次夹紧闭合的过程中没有完全闭合而清除init标志位会出现夹爪的偏移的情况
                // 这个时候可能会出现gripInfo.posit_grip <= gripCloseSlowPosit自锁的情况
                if (gripInfo.posit_grip >= gripOpenSlowPosit) {
                    float_t absMeasuredSpd = fabsf(static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED]));
                    float_t absTargetSpd = fabsf(gripCmd.setSpeed_grip);
                    if (absTargetSpd > rescueParam_.speedThresh && absMeasuredSpd < rescueParam_.stuckThresh) {
                        if (++rescueParam_.detectCnt >= rescueParam_.triggerTime) {
                            rescueParam_.targetPosit = gripInfo.posit_grip
                                + (isClosingCommand ? -1 : 1) * rescueParam_.distanceEnc;
                            gripInfo.state = SGripInfo::EGripState::SAVE;
                            rescueParam_.detectCnt = 0;
                            rescueParam_.elapsedTick = 0;
                            pidPosCtrl.ResetPidController();
                            pidSpdCtrl.ResetPidController();
                            return APP_OK;
                        }
                    } else {
                        rescueParam_.detectCnt = 0;
                    }
                }

                /* ---- 双状态控制 ---- */
                switch(gripInfo.state) {
                    case SGripInfo::EGripState::RELEASE: {
                        if (gripInfo.posit_grip <= 0 && isClosingCommand ){
                            //&& griGripMotor_.cntstable > 100) { //这个定时器要慎重加
                            gripInfo.state = SGripInfo::EGripState::HOLD;
                            gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                            //griGripMotor_.cntstable = 0;
                            gripInfo.isGripped = true;
                            gripDetect_.edgeReady = false;
                            return _UpdateOutputSpd(0);
                        }
                        //griGripMotor_.cntstable ++ ;
                        // 通过检测由低到高的边沿判定夹取成功
                        if (gripDetect_.filteredTorque < gripDetect_.detectTorque) {
                            gripDetect_.edgeReady = true;
                        } else if (gripDetect_.edgeReady && gripDetect_.filteredTorque > gripDetect_.detectTorque) {
                            if (gripCmd.setSpeed_grip < 0 ) { ///< 仅闭合方向触发
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
                        if(isOpeningCommand){   //如果夹爪保持夹持的状态的时候如果改变速度指令会改变夹爪的闭合状态
                            gripInfo.state = SGripInfo::EGripState::RELEASE;
                            gripInfo.isGripped = false;
                            gripCmd.regripPulse = false;
                            gripInfo.repeatInit = false;//夹紧的时候直接清零重置
                            gripCmd.regripStableCnt = 0;
                            pidSpdCtrl.ResetPidController();
                            return _UpdateOutputSpd(gripCmd.setSpeed_grip);
                        }

                        // HOLD 状态下处理二次夹紧的组件的脉冲，同时这个状态始终保持抓紧的状态不要立刻松开
                        if (gripCmd.regripPulse) {
                            // 防止异常情况无限闭合
                            if (HAL_GetTick() - static_cast<uint32_t>(gripCmd.outTime_tick) > 500) {
                                gripCmd.regripPulse = false;
                                gripCmd.regripStableCnt = 0;
                                gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                                return _UpdateOutputSpd(0);
                            }

                            // 防止空夹丝杆过冲自锁
                            if (gripInfo.posit_grip < gripInfo.holdPosit_Grip - gripCmd.regripMaxTravel) {
                                gripCmd.regripPulse = false;
                                gripCmd.regripStableCnt = 0;
                                gripInfo.isGripped = false;  // 标记矿石丢失
                                return _UpdateOutputSpd(0);
                            }

                            // 输出闭合速度，复用 RELEASE 分支相同的力矩反馈减速逻辑
                            float_t spd = -GRIP_CLOSE_SPEED;
                            if (gripDetect_.filteredTorque > gripDetect_.closeTorqueThresh) {
                                spd *= (gripDetect_.filteredTorque - gripDetect_.closeTorqueThresh < gripDetect_.closeTorqueRange)
                                     ? (1.0f - (gripDetect_.filteredTorque - gripDetect_.closeTorqueThresh) / gripDetect_.closeTorqueRange)
                                     : 0.0f;
                                if (spd > -gripDetect_.closeSpeedMin) {
                                    spd = -gripDetect_.closeSpeedMin;
                                }
                            }

                            // 当速度被力矩压到下限附近并且力矩很高，连续 20ms 视为压紧到位
                            bool spdAtFloor = (spd >= -gripDetect_.closeSpeedMin * 1.05f);
                            bool torqueHigh = (gripDetect_.filteredTorque > gripDetect_.closeTorqueThresh);
                            if (spdAtFloor && torqueHigh) {
                                gripCmd.regripStableCnt++;
                                if (gripCmd.regripStableCnt >= 20) { // 20 × 1ms = 20ms 稳定即视为完成
                                    gripCmd.regripPulse = false;
                                    gripCmd.regripStableCnt = 0;
                                    gripInfo.holdPosit_Grip = gripInfo.posit_grip;
                                    return _UpdateOutputSpd(0);
                                }
                            } else {
                                gripCmd.regripStableCnt = 0;
                            }

                            return _UpdateOutputSpd(spd);
                        }

                        // 保持夹持：速度归零，丝杆自锁保持位置
                        return _UpdateOutputSpd(0);
                    }

                    case SGripInfo::EGripState::SAVE: {
                        // 位置环驱动自救，利用位置环PID干爆丝杆自锁摩擦力
                        _UpdateOutput(static_cast<float_t>(rescueParam_.targetPosit) * ARM_GRIP_MOTOR_DIR);

                        // 位置到达判定
                        const int32_t posError = gripInfo.posit_grip - rescueParam_.targetPosit;
                        const int32_t absPosError = posError < 0 ? -posError : posError;
                        if (absPosError < static_cast<int32_t>(ARM_END_GRIP_MOTOR_RATIO * 1.f)) { //误差超过1的时候说明已经脱离的自锁的状态
                            gripInfo.state = SGripInfo::EGripState::RELEASE;
                            pidPosCtrl.ResetPidController();
                            pidSpdCtrl.ResetPidController();
                            rescueParam_.elapsedTick = 0;
                            return _UpdateOutputSpd(gripCmd.setSpeed_grip);
                        }

                        // 超时保护
                        if (++rescueParam_.elapsedTick > rescueParam_.timeout) { //500tick的自救超时保护
                            gripInfo.state = SGripInfo::EGripState::RELEASE;
                            pidPosCtrl.ResetPidController();
                            pidSpdCtrl.ResetPidController();
                            rescueParam_.elapsedTick = 0;
                            return _UpdateOutputSpd(gripCmd.setSpeed_grip);
                        }

                        return APP_OK;
                    }
                }
            }
            default:{
                StopComponent();
                mtrOutputBuffer = 0;
                speedTargetFiltered_ = 0.0f;
                speedMeasuredFiltered_ = 0.0f;
                initTick_ = 0;
                rescueParam_.detectCnt = 0;
                rescueParam_.elapsedTick = 0;
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

    if (mtrOutputBuffer > GRIP_OUTPUT_LIMIT && (gripInfo.state != SGripInfo::EGripState::SAVE)) mtrOutputBuffer = GRIP_OUTPUT_LIMIT;
    else if (mtrOutputBuffer < -GRIP_OUTPUT_LIMIT && (gripInfo.state != SGripInfo::EGripState::SAVE)) mtrOutputBuffer = -GRIP_OUTPUT_LIMIT;
    //这里的限幅感觉在自锁的时候也没有办法释放最大的性能

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

/**
 * @brief 减速计算公式
 */
float_t CModArm::CComGrip::CalcGripSlowSpeed(float_t maxSpeed, float_t minSpeed, int32_t remainToStop, int32_t slowBand) {
    if (slowBand <= 0) return minSpeed;

    float_t speedRatio = static_cast<float_t>(remainToStop) / static_cast<float_t>(slowBand);
    if (speedRatio < 0.0f) speedRatio = 0.0f;
    if (speedRatio > 1.0f) speedRatio = 1.0f;

    const float_t speedMin = (minSpeed < maxSpeed) ? minSpeed : maxSpeed;
    return speedMin + (maxSpeed - speedMin) * speedRatio;
}


}//namespace my_engineer
