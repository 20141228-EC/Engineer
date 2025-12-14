/**
 * @file com_end.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 机械臂末端组件
 * @version 1.0
 * @date 2025-12-09
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "mod_arm.hpp"

namespace my_engineer {

/**
 * @brief 初始化机械臂末端组件(末端差速器 夹爪)
 *
 * @param param
 * @return EAppStatus
 */
EAppStatus CModArm::CComEnd::InitComponent(SModInitParam_Base &param) {
    if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    auto armParam = static_cast<SModInitParam_Arm &>(param);

    motor[L] = MotorIDMap.at(armParam.MotorID_End_L);
    motor[R] = MotorIDMap.at(armParam.MotorID_End_R);
    motor[GRIP] = MotorIDMap.at(armParam.MotorID_Grip);

    // 初始化末端CAN发送节点
    mtrCanTxNode[L] = armParam.MotorTxNode_End_L;
    mtrCanTxNode[R] = armParam.MotorTxNode_End_R;
    // 初始化夹爪CAN发送节点
    mtrCanTxNode[GRIP] = armParam.MotorTxNode_Grip;

    // 初始化末端PID参数
    armParam.endPosPidParam.threadNum = 2;
    pidPosCtrl.InitPID(&armParam.endPosPidParam);
    armParam.endSpdPidParam.threadNum = 2;
    pidSpdCtrl.InitPID(&armParam.endSpdPidParam);

    // 初始化夹爪PID参数
    armParam.GripPosPidParam.threadNum = 1;
    pidGripPosCtrl.InitPID(&armParam.GripPosPidParam);
    armParam.GripSpdPidParam.threadNum = 1;
    pidGripSpdCtrl.InitPID(&armParam.GripSpdPidParam);

    // 输出缓冲区清零
    mtrOutputBuffer.fill(0);

    Component_FSMFlag_ = FSM_RESET;
    componentStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 更新组件
 *
 */
EAppStatus CModArm::CComEnd::UpdateComponent() {
    if (componentStatus == APP_RESET) return APP_ERROR;

    // -------------------------- 末端Pitch/Roll更新逻辑 --------------------------
    endInfo.posit_Roll  =  (motor[L]->motorData[CDevMtr::DATA_POSIT] + motor[R]->motorData[CDevMtr::DATA_POSIT]) / 2;
    endInfo.posit_Pitch = ((motor[R]->motorData[CDevMtr::DATA_POSIT] - endInfo.posit_Roll) - (motor[L]->motorData[CDevMtr::DATA_POSIT] - endInfo.posit_Roll))/2.0;

    endInfo.isPositArrived_Pitch = (abs(endCmd.setPosit_Pitch - endInfo.posit_Pitch) < 8192 * 2);
    endInfo.isPositArrived_Roll = (abs(endCmd.setPosit_Roll - endInfo.posit_Roll) < 8192 * 2);

    // -------------------------- 夹爪更新逻辑 --------------------------
    endInfo.posit_grip = motor[GRIP]->motorData[CDevMtr::DATA_POSIT] * ARM_GRIP_MOTOR_DIR;
    endInfo.isPositArrived_Grip = (abs(endCmd.setPosit_grip - endInfo.posit_grip) < 819); ///< 位置误差小于1度认为到达目标

    switch (Component_FSMFlag_) {
        case FSM_RESET: {
            // 复位状态，所有电机输出为0
            mtrOutputBuffer.fill(0);
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            pidGripPosCtrl.ResetPidController();
            pidGripSpdCtrl.ResetPidController();
            return APP_OK;
        }

        case FSM_PREINIT: {
            // 预初始化状态
            endCmd.setPosit_Pitch = 0;
            motor[L]->motorData[CDevMtr::DATA_POSIT] = 0;
            motor[R]->motorData[CDevMtr::DATA_POSIT] = 0;
            motor[GRIP]->motorData[CDevMtr::DATA_POSIT] = 0;

            mtrOutputBuffer.fill(0);
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            pidGripPosCtrl.ResetPidController();
            pidGripSpdCtrl.ResetPidController();
            
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT: {

            // 末端初始化逻辑
            bool isEndMotorStall = (motor[L]->motorStatus == CDevMtr::EMotorStatus::STALL || 
                                    motor[R]->motorStatus == CDevMtr::EMotorStatus::STALL);
            if (isEndMotorStall) {
                motor[L]->motorData[CDevMtr::DATA_POSIT] = -(static_cast<int32_t>(0.5 * 8192) + rangeLimit_Pitch);
                motor[R]->motorData[CDevMtr::DATA_POSIT] = (static_cast<int32_t>(0.5 * 8192) + rangeLimit_Pitch);
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
            }

            // 夹爪初始化逻辑
            bool isGripMotorStall = (motor[GRIP]->motorStatus == CDevMtr::EMotorStatus::STALL);
            if(isGripMotorStall) {
                endCmd.setPosit_grip = 0;         ///<堵转之后设置目标值
                motor[GRIP]->motorData[CDevMtr::DATA_POSIT] = static_cast<int32_t> (0.1*8192 + rangeLimit_Grip) * ARM_GRIP_MOTOR_DIR;///<堵转零点超量标定
                endInfo.isGripped = false;        ///<重置夹持状态
                endInfo.holdPosit_Grip = 0;       ///<清空夹持记忆位置
                pidGripPosCtrl.ResetPidController();
                pidGripSpdCtrl.ResetPidController();
            }

            if (isEndMotorStall || isGripMotorStall) { ///< 只要有一个堵转就进入ctrl
                Component_FSMFlag_ = FSM_CTRL;
                componentStatus = APP_OK;
                return APP_OK;
            }

            // 逐步增加目标位置完成初始化
            endCmd.setPosit_Pitch += 200;
            endCmd.setPosit_grip += 500;
            
            // 更新末端和夹爪输出
            _UpdateOutput(static_cast<float_t>(endCmd.setPosit_Pitch), static_cast<float_t>(0));
            _UpdateOutput_Grip(static_cast<float_t>(endCmd.setPosit_grip));
            return APP_OK;
        }

        case FSM_CTRL: {

            // 末端控制逻辑
            _UpdateOutput(static_cast<float_t>(endCmd.setPosit_Pitch), static_cast<float_t>(endCmd.setPosit_Roll));

            // 夹爪控制逻辑
            endCmd.setPosit_grip = std::clamp<int32_t>(endCmd.setPosit_grip, static_cast<int32_t>(0), rangeLimit_Grip);

            if(endInfo.isGripped) {
                // 夹持模式
                if(endCmd.setPosit_grip > endInfo.holdPosit_Grip + 819) { ///<目标位置向张开方向移动超过阈值，退出夹持
                    endInfo.isGripped = false;
                    _UpdateOutput_Grip(static_cast<float_t>(endCmd.setPosit_grip));
                } 
                else {
                    _UpdateOutput_Grip(static_cast<float_t>(endInfo.holdPosit_Grip)); ///<保持夹持位置
                }
            } 
            else {
                // 正常模式
                bool isClosingDirection = (endCmd.setPosit_grip < endInfo.posit_grip);
                
                // 向闭合方向运动时堵转 即成功夹取到物体
                if(motor[GRIP]->motorStatus == CDevMtr::EMotorStatus::STALL && isClosingDirection) {
                    endInfo.isGripped = true; ///< 进入夹持模式
                    endInfo.holdPosit_Grip = endCmd.setPosit_grip; ///<记忆夹持位置
                }
                _UpdateOutput_Grip(static_cast<float_t>(endCmd.setPosit_grip));
            }
            return APP_OK;
        }

        default: {
            StopComponent();
            mtrOutputBuffer.fill(0);
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            pidGripPosCtrl.ResetPidController();
            pidGripSpdCtrl.ResetPidController();
            componentStatus = APP_ERROR;
            return APP_ERROR;
        }
    }

    return APP_OK;
}

/**
 * @brief 物理位置转换为电机位置: Pitch
 *
 * @param phyPosit
 * @return int32_t
 */
int32_t CModArm::CComEnd::PhyPositToMtrPosit_Pitch(float_t phyPosit) {
    const int32_t zeroOffset = ARM_END_PITCH_MOTOR_OFFSET;
    const float_t scale = 1590.2117f;

    return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}

/**
 * @brief 电机位置转换为物理位置: Pitch
 *
 * @param mtrPosit
 * @return float_t
 */
float_t CModArm::CComEnd::MtrPositToPhyPosit_Pitch(int32_t mtrPosit) {
    const int32_t zeroOffset = ARM_END_PITCH_MOTOR_OFFSET;
    const float_t scale = 1590.2117f;

    return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}

/**
 * @brief 物理位置转换为电机位置: Roll
 *
 * @param phyPosit
 * @return int32_t
 */
int32_t CModArm::CComEnd::PhyPositToMtrPosit_Roll(float_t phyPosit) {
    const int32_t zeroOffset = 0;
    const float_t scale = ARM_END_ROLL_MOTOR_RATIO;

    return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
}

/**
 * @brief 电机位置转换为物理位置: Roll
 *
 * @param mtrPosit
 * @return float_t
 */
float_t CModArm::CComEnd::MtrPositToPhyPosit_Roll(int32_t mtrPosit) {
    const int32_t zeroOffset = 0;
    const float_t scale = ARM_END_ROLL_MOTOR_RATIO;

    return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
}

/**
 * @brief 电机位置转换为物理位置： grip
 *
 * @param mtrPosit
 * @return float_t
 */
float_t CModArm::CComEnd::MtrPositToPhyPosit_Grip(float_t mtrPosit) {
    const int32_t zeroOffset = 0; // 夹爪电机零点偏移
    const float_t ratio = ARM_END_GRIP_MOTOR_RATIO; // 夹爪电机与物理位置转换比（单位mm）
    return (static_cast<float_t>(mtrPosit - zeroOffset) / ratio);
}

/**
 * @brief 物理位置转换为电机位置： grip
 *
 * @param phyPosit
 * @return int32_t
 */
int32_t CModArm::CComEnd::PhyPositToMtrPosit_Grip(float_t phyPosit) {
    const int32_t zeroOffset = 0; // 夹爪电机零点偏移
    const float_t ratio = ARM_END_GRIP_MOTOR_RATIO; // 夹爪电机与物理位置转换比（单位mm）
    return static_cast<int32_t>(phyPosit * ratio) + zeroOffset;
}

/**
 * @brief 末端Pitch/Roll输出更新函数
 *
 * @param posit_Pitch
 * @param posit_Roll
 * @return EAppStatus
 */
EAppStatus CModArm::CComEnd::_UpdateOutput(float_t posit_Pitch, float_t posit_Roll) {
    DataBuffer<float_t> endPos = {
        posit_Roll - posit_Pitch,
        posit_Roll + posit_Pitch,
    };

    DataBuffer<float_t> endPosMeasure = {
        static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_POSIT]),
        static_cast<float_t>(motor[R]->motorData[CDevMtr::DATA_POSIT])
    };

    auto endSpd = pidPosCtrl.UpdatePidController(endPos, endPosMeasure);

    DataBuffer<float_t> endSpdMeasure = {
        static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float_t>(motor[R]->motorData[CDevMtr::DATA_SPEED])
    };

    auto output = pidSpdCtrl.UpdatePidController(endSpd, endSpdMeasure);

    mtrOutputBuffer = {
        static_cast<int16_t>(output[L]),
        static_cast<int16_t>(output[R]),
        mtrOutputBuffer[GRIP] // 保留夹爪输出值
    };

    return APP_OK;
}

/**
 * @brief 夹爪输出更新函数
 *
 * @param posit_Grip
 * @return EAppStatus
 */
EAppStatus CModArm::CComEnd::_UpdateOutput_Grip(float posit_Grip) {
    DataBuffer<float_t> gripPos = {
        static_cast<float_t>(posit_Grip) * ARM_GRIP_MOTOR_DIR
    };

    DataBuffer<float_t> gripPosMeasured = {static_cast<float_t>(motor[GRIP]->motorData[CDevMtr::DATA_POSIT])};

    auto pidPosOutput = pidGripPosCtrl.UpdatePidController(gripPos, gripPosMeasured);

    DataBuffer<float_t> gripSpdMeasured = {static_cast<float_t>(motor[GRIP]->motorData[CDevMtr::DATA_SPEED])};

    auto Output = pidGripSpdCtrl.UpdatePidController(pidPosOutput, gripSpdMeasured);

    mtrOutputBuffer[GRIP] = static_cast<int16_t>(Output[0]);

    return APP_OK;
}

/**
 * @brief 统一输出更新函数（包含末端+夹爪）
 *
 * @param posit_Pitch
 * @param posit_Roll
 * @param posit_grip
 * @return EAppStatus
 */
EAppStatus CModArm::CComEnd::_UpdateOutput_All(float_t posit_Pitch, float_t posit_Roll, float_t posit_grip) {
    _UpdateOutput(posit_Pitch, posit_Roll);
    _UpdateOutput_Grip(posit_grip);
    return APP_OK;
}

} // namespace my_engineer